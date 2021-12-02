/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<vector>
#include<queue>
#include<thread>
#include<mutex>

#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/Imu.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"
#include"../include/ImuTypes.h"

#include<Eigen/Dense>
//新增发布Odometry
#include<nav_msgs/Odometry.h>

using namespace std;
using namespace ros;

ros::Publisher pub_odometry;

//定义第一帧flag，用于时间戳作差
bool first_image_flag = true;
double first_image_time;

class ImuGrabber
{
public:
    ImuGrabber(){};
    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);

    queue<sensor_msgs::ImuConstPtr> imuBuf;
    std::mutex mBufMutex;
};

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM, ImuGrabber *pImuGb, const bool bClahe): mpSLAM(pSLAM), mpImuGb(pImuGb), mbClahe(bClahe){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);
    void SyncWithImu();

    queue<sensor_msgs::ImageConstPtr> img0Buf;
    std::mutex mBufMutex;
   
    ORB_SLAM3::System* mpSLAM;
    ImuGrabber *mpImuGb;

    const bool mbClahe;
    cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "Mono_Inertial");
  ros::NodeHandle n("~");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
  bool bEqual = false;

  //发布odometry话题
  pub_odometry = n.advertise<nav_msgs::Odometry>("odometry",1000);

  if(argc < 3 || argc > 4)
  {
    cerr << endl << "Usage: rosrun ORB_SLAM3 Mono_Inertial path_to_vocabulary path_to_settings [do_equalize]" << endl;
    ros::shutdown();
    return 1;
  }


  if(argc==4)
  {
    std::string sbEqual(argv[3]);
    if(sbEqual == "true")
      bEqual = true;
  }

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_MONOCULAR,true);

  ImuGrabber imugb;
  ImageGrabber igb(&SLAM,&imugb,bEqual); // TODO
  
  // Maximum delay, 5 seconds
  ros::Subscriber sub_imu = n.subscribe("/camera/imu", 1000, &ImuGrabber::GrabImu, &imugb); 
  ros::Subscriber sub_img0 = n.subscribe("/camera/image_raw", 100, &ImageGrabber::GrabImage,&igb);

  std::thread sync_thread(&ImageGrabber::SyncWithImu,&igb);

  ros::spin();

  return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr &img_msg)
{
  mBufMutex.lock();
  if (!img0Buf.empty())
    img0Buf.pop();
  img0Buf.push(img_msg);
  mBufMutex.unlock();

  //判断是否第一帧
  if(first_image_flag)
  {
    first_image_flag = false;
    first_image_time = img_msg->header.stamp.toSec();
    return;
  }
  ofstream f("/home/user/Documents/stamp.txt", ios::app);
  f.setf(ios::fixed, ios::floatfield);
  f.precision(10);
  f << img_msg->header.stamp.toSec() - first_image_time << endl;
  f.close();
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg)
{
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  
  if(cv_ptr->image.type()==0)
  {
    return cv_ptr->image.clone();
  }
  else
  {
    std::cout << "Error type" << std::endl;
    return cv_ptr->image.clone();
  }
}


//新增pubOdometry用于发布里程计数据
void pubOdometry(const vector<float> &t, const vector<float> &q, const std_msgs::Header &header)
{
  nav_msgs::Odometry odometry;
  odometry.header = header;
  odometry.header.frame_id = "vio";
  //odometry.child_fram_id = "base_link";
  odometry.pose.pose.position.x = t[0];
  odometry.pose.pose.position.y = t[1];
  odometry.pose.pose.position.z = t[2];
  odometry.pose.pose.orientation.x = q[0];
  odometry.pose.pose.orientation.y = q[1];
  odometry.pose.pose.orientation.z = q[2];
  odometry.pose.pose.orientation.w = q[3];

  pub_odometry.publish(odometry);

}

void ImageGrabber::SyncWithImu()
{
  while(1)
  {
    cv::Mat im;
    double tIm = 0;
    if (!img0Buf.empty()&&!mpImuGb->imuBuf.empty())
    {
      std_msgs::Header header = img0Buf.front()->header;
      tIm = img0Buf.front()->header.stamp.toSec();
      if(tIm>mpImuGb->imuBuf.back()->header.stamp.toSec())
          continue;
      {
      this->mBufMutex.lock();
      im = GetImage(img0Buf.front());
      img0Buf.pop();
      this->mBufMutex.unlock();
      }

      vector<ORB_SLAM3::IMU::Point> vImuMeas;
      mpImuGb->mBufMutex.lock();
      if(!mpImuGb->imuBuf.empty())
      {
        // Load imu measurements from buffer
        vImuMeas.clear();
        while(!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec()<=tIm)
        {
          double t = mpImuGb->imuBuf.front()->header.stamp.toSec();
          cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x, mpImuGb->imuBuf.front()->linear_acceleration.y, mpImuGb->imuBuf.front()->linear_acceleration.z);
          cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z);
          vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc,gyr,t));
          mpImuGb->imuBuf.pop();
        }
      }
      mpImuGb->mBufMutex.unlock();
      if(mbClahe)
        mClahe->apply(im,im);

      // mpSLAM->TrackMonocular(im,tIm,vImuMeas);

      //获得cam在word系下的表达 tcw
      cv::Mat tmp_tcw = cv::Mat::eye(4,4,CV_32F);
      tmp_tcw = mpSLAM->TrackMonocular(im,tIm,vImuMeas);

      //当tracking faild时tcw为空
      if(tmp_tcw.empty()) 
      {
        cout << "Tcw empty" << endl;
      }
      else
      {
        //将 tcw 中的旋转部分拆开，得到rcw
        cv::Mat tmp_rcw = cv::Mat::eye(3,3,CV_32F); 
        tmp_rcw = tmp_tcw.rowRange(0,3).colRange(0,3).t();
        //将rcw矩阵形式转换成四元数
        Eigen::Matrix<double,3,3> M;
        M << tmp_rcw.at<float>(0,0), tmp_rcw.at<float>(0,1), tmp_rcw.at<float>(0,2),
            tmp_rcw.at<float>(1,0), tmp_rcw.at<float>(1,1), tmp_rcw.at<float>(1,2),
            tmp_rcw.at<float>(2,0), tmp_rcw.at<float>(2,1), tmp_rcw.at<float>(2,2);

        Eigen::Quaterniond q(M);

        std::vector<float> v(4);
        v[0] = q.x();
        v[1] = q.y();
        v[2] = q.z();
        v[3] = q.w();
        
        //将 tcw 中的平移部分拆开，得到t
        std::vector<float > t(3);
        t[0] = tmp_tcw.at<float>(0,3);
        t[1] = tmp_tcw.at<float>(1,3);
        t[2] = tmp_tcw.at<float>(2,3);

        pubOdometry(t, v, header);
      }

    }

    std::chrono::milliseconds tSleep(1);
    std::this_thread::sleep_for(tSleep);
  }
}

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
  mBufMutex.lock();
  imuBuf.push(imu_msg);
  mBufMutex.unlock();
  return;
}


