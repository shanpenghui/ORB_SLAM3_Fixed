//
// Created by sph on 2020/9/15.
//

#include <iostream>

using namespace std;

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

int main() {

  // 四元数初始化
  double Q1_angle_ = M_PI / 4;
  Eigen::Quaterniond Q1(cos(Q1_angle_ / 2), 0 * sin(Q1_angle_ / 2), 0 * sin(Q1_angle_ / 2), 1 * sin(Q1_angle_ / 2));

  double Q2_angle_ = M_PI / 4 + M_PI / 4;
  Eigen::Quaterniond Q2(cos(Q2_angle_ / 2), 0 * sin(Q2_angle_ / 2), 0 * sin(Q2_angle_ / 2), 1 * sin(Q2_angle_ / 2));

  double Q3_angle_ = M_PI / 4 - M_PI / 4;
  Eigen::Quaterniond Q3(cos(Q3_angle_ / 2), 0 * sin(Q3_angle_ / 2), 0 * sin(Q3_angle_ / 2), 1 * sin(Q3_angle_ / 2));

  // quat to rotation_matrix
  Eigen::Matrix3d rotation_matrix_Q1_ = Q1.matrix();
  cout << "rotation_matrix_Q1_ = \n" << rotation_matrix_Q1_ << endl;

  Eigen::Matrix3d rotation_matrix_Q2_ = Q2.matrix();
  cout << "rotation_matrix_Q2_ = \n" << rotation_matrix_Q2_ << endl;
  Eigen::Matrix3d rotation_matrix_Q2_ed_ = rotation_matrix_Q2_ * rotation_matrix_Q1_.inverse();

  cout <<  "旋转角度： " << -asin(rotation_matrix_Q2_ed_(0, 1)) * 180 / M_PI << "度" << endl;
  cout << endl;

  Eigen::Matrix3d rotation_matrix_Q3_ = Q3.matrix();
  cout << "rotation_matrix_Q3_ = \n" << rotation_matrix_Q3_ << endl;

//  //平移向量
//  Eigen::Vector3d t1 = Eigen::Vector3d(0.3, 0.1, 0.1);
//  Eigen::Vector3d t2 = Eigen::Vector3d(-0.1, 0.5, 0.3);
//
//  //目标向量
//  Eigen::Vector3d p1 = Eigen::Vector3d(0.5, 0, 0.2);
//  Eigen::Vector3d p2;
//
//  //打印输出
//  // cout << q1.coeffs() << "\n"
//  //      << q2.coeffs() << "\n"
//  //      << t1.transpose() << "\n"
//  //      << t2.transpose() << endl;
//
//  //四元数求解
//  p2 = q2 * q1.inverse() * (p1 - t1) + t2;
//  cout << p2.transpose() << endl;
//
//  //欧拉矩阵
//  Eigen::Isometry3d T1 = Eigen::Isometry3d::Identity();
//  Eigen::Isometry3d T2 = Eigen::Isometry3d::Identity();
//  T1.rotate(q1.toRotationMatrix());
//  T1.pretranslate(t1);
//  T2.rotate(q2.toRotationMatrix());
//  T2.pretranslate(t2);
//
//  // cout << T1.matrix() << endl;
//  // cout << T2.matrix() << endl;
//
//  //欧拉矩阵求解
//  p2 = T2 * T1.inverse() * p1;
//  cout << p2.transpose() << endl;
}