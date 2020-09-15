//
// Created by sph on 2020/9/15.
//

#include <iostream>
#include <Eigen/Dense>

#include <math.h>

using namespace Eigen;
using namespace std;

int main()
{
  /* 三维笛卡尔坐标系
   * 1）一个物体在我们的正前方；朝向我们的方向为X正，绕着X方向的旋转为翻滚角;YZ坐标系顺时针为正
   * 2）物体右手侧的方向为Y；绕着Y方向的旋转为俯仰角；ZX坐标系顺时针为正；
   * 3）物体上方的方向为Z；绕着Z方向的旋转为偏航角；XY坐标系顺时针为正；
  */

  /* 验证翻滚角 */
  double rollPrevious = M_PI/6;
  double pitchPrevious = 0;
  double yawPrevious = 0;

  Vector3d eulerAngulePrevious(rollPrevious, pitchPrevious, yawPrevious);

  Matrix3f matrixPrevious;
  matrixPrevious = AngleAxisf(eulerAngulePrevious(0), Vector3f::UnitX()) * AngleAxisf(eulerAngulePrevious(1), Vector3f::UnitY()) * AngleAxisf(eulerAngulePrevious(2), Vector3f::UnitZ());

  double rollCurrent = M_PI/3;
  double pitchCurrent = 0;
  double yawCurrent = 0;

  Vector3d eulerAnguleCurrent(rollCurrent, pitchCurrent, yawCurrent);

  Matrix3f matrixCurrent;
  matrixCurrent = AngleAxisf(eulerAnguleCurrent(0), Vector3f::UnitX()) * AngleAxisf(eulerAnguleCurrent(1), Vector3f::UnitY()) * AngleAxisf(eulerAnguleCurrent(2), Vector3f::UnitZ());

  Matrix3f rotationMatrix = matrixCurrent * matrixPrevious.inverse();
  cout << "绕X轴旋转PI/6的旋转矩阵:\n" << rotationMatrix << endl;

  /*绕X轴的旋转矩阵
   * 1        0               0
   * 0      cos(roll)   -sin(roll)
   * 0      sin(roll)    cos(roll)
  */
  cout <<  "旋转角度： " << -asin(rotationMatrix(1, 2)) * 180 / M_PI << "度" << endl;
  cout << endl;

  /* 验证俯仰角 */
  rollPrevious = 0;
  pitchPrevious = M_PI/6;
  yawPrevious = 0;

  eulerAngulePrevious(0) = rollPrevious;
  eulerAngulePrevious(1) = pitchPrevious;
  eulerAngulePrevious(2) = yawPrevious;

  matrixPrevious = AngleAxisf(eulerAngulePrevious(0), Vector3f::UnitX()) * AngleAxisf(eulerAngulePrevious(1), Vector3f::UnitY()) * AngleAxisf(eulerAngulePrevious(2), Vector3f::UnitZ());

  rollCurrent = 0;
  pitchCurrent = M_PI/3;
  yawCurrent = 0;

  eulerAnguleCurrent(0) = rollCurrent;
  eulerAnguleCurrent(1) = pitchCurrent;
  eulerAnguleCurrent(2) = yawCurrent;

  matrixCurrent = AngleAxisf(eulerAnguleCurrent(0), Vector3f::UnitX()) * AngleAxisf(eulerAnguleCurrent(1), Vector3f::UnitY()) * AngleAxisf(eulerAnguleCurrent(2), Vector3f::UnitZ());

  rotationMatrix = matrixCurrent * matrixPrevious.inverse();
  cout << "绕Y轴旋转PI/6的旋转矩阵:\n" << rotationMatrix << endl;

  /*绕Y轴的旋转矩阵
   * cos(pitch)        0          sin(pitch)
   * 0                 1               0
   * -sin(pitch)       0         -cos(pitch)
  */
  cout <<  "旋转角度： " << asin(rotationMatrix(0, 2)) * 180 / M_PI << "度" << endl;
  cout << endl;


  /* 验证偏航角 */
  rollPrevious = 0;
  pitchPrevious = 0;
  yawPrevious = M_PI/6;

  eulerAngulePrevious(0) = rollPrevious;
  eulerAngulePrevious(1) = pitchPrevious;
  eulerAngulePrevious(2) = yawPrevious;

  matrixPrevious = AngleAxisf(eulerAngulePrevious(0), Vector3f::UnitX()) * AngleAxisf(eulerAngulePrevious(1), Vector3f::UnitY()) * AngleAxisf(eulerAngulePrevious(2), Vector3f::UnitZ());

  rollCurrent = 0;
  pitchCurrent = 0;
  yawCurrent = M_PI/3;

  eulerAnguleCurrent(0) = rollCurrent;
  eulerAnguleCurrent(1) = pitchCurrent;
  eulerAnguleCurrent(2) = yawCurrent;

  matrixCurrent = AngleAxisf(eulerAnguleCurrent(0), Vector3f::UnitX()) * AngleAxisf(eulerAnguleCurrent(1), Vector3f::UnitY()) * AngleAxisf(eulerAnguleCurrent(2), Vector3f::UnitZ());

  rotationMatrix = matrixCurrent * matrixPrevious.inverse();
  cout << "绕Z轴旋转PI/6的旋转矩阵:\n" << rotationMatrix << endl;

  /*绕Z轴的旋转矩阵
   * cos(yaw)        -sin(yaw)          0
   * sin(yaw)        cos(yaw)           0
   * 0                  0               1
  */
  cout <<  "旋转角度： " << -asin(rotationMatrix(0, 1)) * 180 / M_PI << "度" << endl;
  cout << endl;

  /* 按照元素相乘 */
  Eigen::Vector3d temp1(-1, 0, 1);
  Eigen::Vector3d temp2(-2, 0, 2);

  cout << temp1.cwiseProduct(temp2) << endl;
  cout << endl;

}
