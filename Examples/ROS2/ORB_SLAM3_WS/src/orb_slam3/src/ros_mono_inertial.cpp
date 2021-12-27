#include <functional>
#include <queue>
#include <thread>
#include <mutex>
#include <memory>
#include <chrono>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
//#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <System.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <opti_track/msg/track_data.hpp>
#include <memory>
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

class ORBSLAM3Subscriber : public rclcpp::Node
{
 public:

	std::shared_ptr<std::thread> orbslam3_thread_ptr_;

	bool startSlam() {
		orbslam3_thread_ptr_ = std::make_shared<std::thread>(
			std::bind(&ORBSLAM3Subscriber::runSLAM, this));
		return true;
	}

	ORBSLAM3Subscriber(
//		ORB_SLAM3::System* pSLAM,
		const string &strVocFile,
		const string &strSettingsFile,
		const ORB_SLAM3::System::eSensor sensor,
		const bool bUseViewer,
		const int initFr
	)
		: Node("mono_inertial_node"), mpSLAM(std::make_shared<ORB_SLAM3::System>(strVocFile, strSettingsFile, sensor, bUseViewer, initFr)), sensorType(sensor)
	{
		rclcpp::QoS video_qos(10);
		video_qos.keep_last(10);
		video_qos.best_effort();
		video_qos.durability_volatile();

		imuSubscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
			"/camera/imu", 1000, std::bind(&ORBSLAM3Subscriber::imu_callback, this, _1)
		);

		irLeftSubscription_ = this->create_subscription<sensor_msgs::msg::Image>(
			"/camera/fisheye1/image_raw", video_qos, std::bind(&ORBSLAM3Subscriber::irLeft_callback, this, _1)
		);

		optiTrackSubscription_ = this->create_subscription<opti_track::msg::TrackData>(
			"/opti_track_data", 1000, std::bind(&ORBSLAM3Subscriber::optiTrack_callback, this, _1)
		);

	}

	void runSLAM();


	~ORBSLAM3Subscriber() override {
		// Stop all threads
		mpSLAM->Shutdown();

		// Save camera trajectory
		mpSLAM->SaveTrajectoryEuRoCAndOptiTrack("/midea_robot/ros2_ws/orbslam.txt", "/midea_robot/ros2_ws/optitrack.txt");
	}

 private:
	void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
	{
//		RCLCPP_INFO(this->get_logger(), "receive imu msg");
		std::lock_guard<std::mutex> guard{ mBufMutex };
		imuBuf.push(msg);
	}

	void irLeft_callback(const sensor_msgs::msg::Image::SharedPtr msg)
	{
		std::lock_guard<std::mutex> lock{ mBufMutexLeft };
//		RCLCPP_INFO(this->get_logger(), "receive irLeft msg");
		if (!imgLeftBuf.empty())
			imgLeftBuf.pop();
		imgLeftBuf.push(msg);
	}

	void irRight_callback(const sensor_msgs::msg::Image::SharedPtr msg)
	{
		std::lock_guard<std::mutex> lock{ mBufMutexRight };
		if (!imgRightBuf.empty())
			imgRightBuf.pop();
		imgRightBuf.push(msg);
	}

	void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
	{
//		RCLCPP_INFO(this->get_logger(), "receive imu msg");
		std::lock_guard<std::mutex> guard{ mOdomMutex };
		odomBuf.push_back(msg);
	}

	void optiTrack_callback(const opti_track::msg::TrackData::SharedPtr msg)
	{
//		RCLCPP_INFO(this->get_logger(), "receive optiTrack msg");
		if(msg->rigid_bodies.empty())
		{
			RCLCPP_WARN(this->get_logger(), "optiTrack msg rigid_bodies is empty()!");
		}
		else
		{
			OptiTrackData data;
			data.pose_x = msg->rigid_bodies[0].pose.position.x;
			data.pose_y = msg->rigid_bodies[0].pose.position.y;
			data.pose_z = msg->rigid_bodies[0].pose.position.z;
			data.quat_x = msg->rigid_bodies[0].pose.orientation.x;
			data.quat_y = msg->rigid_bodies[0].pose.orientation.y;
			data.quat_z = msg->rigid_bodies[0].pose.orientation.z;
			data.quat_w = msg->rigid_bodies[0].pose.orientation.w;

			mpSLAM->mMutexOptiTrackDataQue.lock();
			mpSLAM->mlOptiTrackPosesQue.push_back(data);
			if(mpSLAM->mlOptiTrackPosesQue.size() > 500)
			{
				mpSLAM->mlOptiTrackPosesQue.pop_front();
			}
			mpSLAM->mMutexOptiTrackDataQue.unlock();
		}
	}

	void imuSLAM();

	// Subscriptions
	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSubscription_;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr irLeftSubscription_;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr irRightSubscription_;
	rclcpp::Subscription<opti_track::msg::TrackData>::SharedPtr optiTrackSubscription_;

	// Queues
	std::queue<sensor_msgs::msg::Imu::SharedPtr> imuBuf;
	std::queue<sensor_msgs::msg::Image::SharedPtr> imgLeftBuf, imgRightBuf;
	std::deque<std::shared_ptr<nav_msgs::msg::Odometry>> odomBuf;

	// Mutex
	std::mutex mBufMutex;
	std::mutex mBufMutexLeft, mBufMutexRight;
	std::mutex mOdomMutex;

	std::shared_ptr<ORB_SLAM3::System> mpSLAM;
	ORB_SLAM3::System::eSensor sensorType;

};

void ORBSLAM3Subscriber::runSLAM()
{
	imuSLAM();
}

void cvToImageMsg(const sensor_msgs::msg::Image::SharedPtr imgRos, cv::Mat& imgCv)
{
//	imgRos->header.stamp = rclcpp::Clock().now();
	imgRos->header.frame_id = "map";
	imgRos->height = imgCv.rows;
	imgRos->width = imgCv.cols;
	int num = 1; // for endianness detection
	imgRos->is_bigendian = !(*(char*)&num == 1);
	imgRos->step = imgCv.step;
	size_t size = imgRos->step * imgRos->height;
	imgRos->data.resize(size);
	imgRos->encoding = "mono8";
	memcpy((char*)(&imgRos->data[0]), &imgCv.data[0], size);
}

void ORBSLAM3Subscriber::imuSLAM()
{
	while (true)
	{
		if (!imgLeftBuf.empty()
			&& !imuBuf.empty())
		{
			double tIm = imgLeftBuf.back()->header.stamp.sec + imgLeftBuf.back()->header.stamp.nanosec * 1e-9;
			if (tIm > (imuBuf.back()->header.stamp.sec + imuBuf.back()->header.stamp.nanosec * 1e-9))
				continue;

			cv::Mat im;
//			cv_bridge::CvImageConstPtr cv_ptrLeft;
			{
				std::unique_lock<std::mutex> lockL{ mBufMutexLeft };
//				cvToImageMsg(imgLeftBuf.front(), im);
				im = cv::Mat(imgLeftBuf.front()->height,
					imgLeftBuf.front()->width, CV_8UC1);
//				im.data = (uchar*)imgLeftBuf.front()->data.data();
				memcpy (im.data, imgLeftBuf.front()->data.data(), imgLeftBuf.front()->data.size());
//				RCLCPP_INFO(this->get_logger(), "imageMsgToCv finished");
				imgLeftBuf.pop();
				lockL.unlock();
			}

			std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
			{
				std::unique_lock<std::mutex> lock{ mBufMutex };
//				mBufMutex.lock();
				if (!imuBuf.empty())
				{
					// Load imu measurements from buffer
					vImuMeas.clear();
					while (!imuBuf.empty()
						&& (imuBuf.front()->header.stamp.sec + imuBuf.front()->header.stamp.nanosec * 1e-9) <= tIm)
					{
//						RCLCPP_INFO(this->get_logger(), "Recording imu measure");
						double t = imuBuf.front()->header.stamp.sec + imuBuf.front()->header.stamp.nanosec * 1e-9;
						cv::Point3f acc(imuBuf.front()->linear_acceleration.x,
							imuBuf.front()->linear_acceleration.y,
							imuBuf.front()->linear_acceleration.z);
						cv::Point3f gyr(imuBuf.front()->angular_velocity.x,
							imuBuf.front()->angular_velocity.y,
							imuBuf.front()->angular_velocity.z);
						vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
						imuBuf.pop();
					}
				}
				else
				{
					RCLCPP_INFO(this->get_logger(), "imuBuf is empty!");
				}
//				lock.unlock();
//				mBufMutex.unlock();
			}

//			cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
//			if(mbClahe)
//				mClahe->apply(im, im);

			if (!vImuMeas.empty())
			{
				mpSLAM->TrackMonocular(im, tIm, vImuMeas);
//				RCLCPP_INFO(this->get_logger(), "TrackMonocular");
			}
			else
			{
				RCLCPP_INFO(this->get_logger(), "vImuMeas is empty!");
			}
		}

		std::chrono::milliseconds tSleep(1);
		std::this_thread::sleep_for(tSleep);
	}
}

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);

	ORB_SLAM3::System::eSensor sensorType;

	sensorType = ORB_SLAM3::System::IMU_MONOCULAR;

	std::string strVocFile = "/midea_robot/ros2_ws/ORB_SLAM3_Fixed/Vocabulary/ORBvoc.txt";
	std::string strSettingsFile = "/midea_robot/ros2_ws/ORB_SLAM3_Fixed/Examples/ROS2/ORB_SLAM3_WS/src/orb_slam3/config/camera.yaml";

//	ORB_SLAM3::System SLAM(strVocFile,
//		strSettingsFile,
//		sensorType,
//		true,
//		false);

	auto nodePtr = std::make_shared<ORBSLAM3Subscriber>(strVocFile, strSettingsFile, sensorType, true, false);
	nodePtr->startSlam();
//	std::thread sync_thread(&ORBSLAM3Subscriber::runSLAM, &(*nodePtr));

	rclcpp::spin(nodePtr);

//	SLAM.SaveTrajectoryEuRoCAndOptiTrack("/midea_robot/ros2_ws/orbslam.txt", "/midea_robot/ros2_ws/optitrack.txt");

//	std::chrono::milliseconds tSleep(1000);
//	std::this_thread::sleep_for(tSleep);

	rclcpp::shutdown();

//	cout << "****************" << sync_thread.joinable() << endl;
//	sync_thread.join();

	return 0;
}
