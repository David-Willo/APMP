/***
 * @Author: David-Willo davidwillo@foxmail.com
 * @Date: 2023-07-25 06:37:49
 * @LastEditTime: 2023-07-30 18:14:56
 * @LastEditors: David-Willo
 * @Jinhao HE (David Willo), IADC HKUST(GZ)
 * @Copyright (c) 2023 by davidwillo@foxmail.com, All Rights Reserved.
 */
#pragma once

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2_ros/transform_broadcaster.h>

#include <boost/circular_buffer.hpp>

#include "xslam/common/arg.h"
#include "xslam/common/common.h"
#include "xslam/common/logging.h"
#include "xslam/visual-types/visual-frame.h"

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

namespace xslam {

class GeoObserver {
 public:
  using Ptr = std::shared_ptr<GeoObserver>;
  using ConstPtr = std::shared_ptr<const GeoObserver>;

  struct Options {
    ADD_ARG(std::string, calib_file) = "";
  };

 public:
  explicit GeoObserver(ros::NodeHandle nh, Options options);
  virtual ~GeoObserver() = default;

 private:
  struct State;
  struct ImuMeas;

  bool gravity_align_ = false;
  bool calibrate_accel_ = true;
  bool calibrate_gyro_ = true;
  int imu_calib_time_ = 8;
  int converge_time_ = 5;
  int imu_buffer_size_ = 5000;
  Eigen::Matrix3f imu_accel_sm_;
  //   std::vector<double> imu_rates;
  double geo_Kp_ = 4.5;
  double geo_Kv_ = 11.25;
  double geo_Kq_ = 4.0;
  double geo_Kab_ = 2.25;
  double geo_Kgb_ = 1.0;
  double geo_abias_max_ = 6;
  double geo_gbias_max_ = 1;

 public:
  ros::NodeHandle nh_;
  ros::Publisher pub_odom_prior_, pub_odom_imu_, pub_imu_raw, pub_imu_filtered, pub_path_imu_, pub_path_prior_;
  nav_msgs::Path path_imu_msg_, path_prior_msg_;
  
  //   states
  std::atomic<bool> imu_calibrated;
  std::atomic<bool> first_imu_received;
  std::atomic<bool> first_img_valid;
  std::atomic<bool> observer_inited_;

  inline bool isObserverReady() {
    return first_cam_stamp > 0 &&
           cur_cam_stamp - first_cam_stamp > converge_time_;
  }

  void consumeImu(const sensor_msgs::Imu::ConstPtr& imu);
  void calibrateImu(const sensor_msgs::Imu::ConstPtr& imu);

  bool imuMeasFromTimeRange(
      double start_time, double end_time,
      boost::circular_buffer<ImuMeas>::reverse_iterator& begin_imu_it,
      boost::circular_buffer<ImuMeas>::reverse_iterator& end_imu_it);
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
  integrateImu(double start_time, Eigen::Quaternionf q_init,
               Eigen::Vector3f p_init, Eigen::Vector3f v_init,
               const std::vector<double>& sorted_timestamps);
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
  integrateImuInternal(
      Eigen::Quaternionf q_init, Eigen::Vector3f p_init, Eigen::Vector3f v_init,
      const std::vector<double>& sorted_timestamps,
      boost::circular_buffer<ImuMeas>::reverse_iterator begin_imu_it,
      boost::circular_buffer<ImuMeas>::reverse_iterator end_imu_it);
  sensor_msgs::Imu::Ptr transformImu(const sensor_msgs::Imu::ConstPtr& imu);

  void initialize();

  void propagateCamera(const VisualFrame::Ptr& frame, bool force = false);
  void propagateState();
  void updateState();
  void updateHistory();
  void getCameraPrior(const VisualFrame::Ptr& frame_last,
                      const VisualFrame::Ptr& frame_cur, SE3& T_prior);
  void updateGravity();

  //   double gravity_=9.80665;
  double gravity_ = 9.7833;  // guangzhou
  Eigen::Vector3f grav_vec;  // bodyframe
  Eigen::Vector3f grav_vec_w;

 private:
  struct Extrinsics {
    struct SE3 {
      Eigen::Vector3f t;
      Eigen::Matrix3f R;
    };
    SE3 baselink2imu;
    SE3 cam2lidar;
    // Eigen::Matrix4f baselink2imu_T;
    // Eigen::Matrix4f baselink2cam_T;
  };
  Extrinsics extrinsics;

  // IMU
  double first_imu_stamp;
  double first_cam_stamp;
  double prev_imu_stamp;
  double prev_cam_stamp;
  double cur_imu_stamp;
  double cur_cam_stamp;
  double imu_dp, imu_dq_deg;

  struct ImuMeas {
    double stamp;
    double dt;  // defined as the difference between the current and the
                // previous measurement
    Eigen::Vector3f ang_vel;
    Eigen::Vector3f lin_accel;
  };
  ImuMeas imu_meas;

  boost::circular_buffer<ImuMeas> imu_buffer;
  std::mutex mtx_imu;
  std::condition_variable cv_imu_stamp;

  Eigen::VectorXf imu2data(ImuMeas& m) {
    Eigen::VectorXf result = Eigen::VectorXf::Zero(6);
    result.head(3) = m.ang_vel;
    result.tail(3) = m.lin_accel;
    return result;
  }

  ImuMeas data2imu(Eigen::VectorXf& d, double stamp, double dt) {
    ImuMeas m;
    m.ang_vel = d.head(3);
    m.lin_accel = d.tail(3);
    m.stamp = stamp;
    m.dt = dt;
    return m;
  }

  static bool comparatorImu(ImuMeas m1, ImuMeas m2) {
    return (m1.stamp < m2.stamp);
  };

  // Geometric Observer
  struct Geo {
    bool first_opt_done;
    std::mutex mtx;
    double dp;
    double dq_deg;
    Eigen::Vector3f prev_p;
    Eigen::Quaternionf prev_q;
    Eigen::Vector3f prev_vel;
  };
  Geo geo;

  // State Vector
  struct ImuBias {
    Eigen::Vector3f gyro;
    Eigen::Vector3f accel;
  };

  struct Frames {
    Eigen::Vector3f b;
    Eigen::Vector3f w;
  };

  struct Velocity {
    Frames lin;
    Frames ang;
  };

  struct State {
    Eigen::Vector3f p;     // position in world frame
    Eigen::Quaternionf q;  // orientation in world frame
    Velocity v;
    ImuBias b;  // imu biases in body frame
  };
  State state;

  struct Pose {
    Eigen::Vector3f p;     // position in world frame
    Eigen::Quaternionf q;  // orientation in world frame
  };
  // Pose lidarPose;
  Pose imuPose;
  Pose camPose;




  public:
    State getCurState() {
      return state;
    }
};

template <typename T>
std::string to_string_with_precision(const T a_value, const int n = 6) {
  std::ostringstream out;
  out.precision(n);
  out << std::fixed << a_value;
  return out.str();
}

// Function to perform quaternion-scalar multiplication
inline Eigen::Quaternionf scale_quat(const Eigen::Quaternionf& q,
                                     float scalar) {
  return Eigen::Quaternionf(q.w() * scalar, q.x() * scalar, q.y() * scalar,
                            q.z() * scalar);
}

inline Eigen::Quaternionf add_quat(const Eigen::Quaternionf& q1,
                                   const Eigen::Quaternionf& q2) {
  Eigen::Quaternionf quat_final;
  quat_final.w() = q1.w() + q2.w();
  quat_final.vec() = q1.vec() + q2.vec();
  return quat_final;
}

// Function to calculate the quaternion derivative
inline Eigen::Quaternionf quat_derivative(
    const Eigen::Quaternionf& orientation_quat,
    const Eigen::Vector3f& angular_velocity) {
  Eigen::Quaternionf angular_velocity_quat(
      0.0f, angular_velocity.x(), angular_velocity.y(), angular_velocity.z());
  return scale_quat(orientation_quat * angular_velocity_quat, 0.5);
}

// Function to perform RK4 update step
inline Eigen::Quaternionf rk4_step(const Eigen::Quaternionf& orientation_quat,
                                   const Eigen::Vector3f& angular_velocity,
                                   float dt) {
  Eigen::Quaternionf k1 = quat_derivative(orientation_quat, angular_velocity);
  Eigen::Quaternionf k2 = quat_derivative(
      add_quat(orientation_quat, scale_quat(k1, 0.5f * dt)), angular_velocity);
  Eigen::Quaternionf k3 = quat_derivative(
      add_quat(orientation_quat, scale_quat(k2, 0.5f * dt)), angular_velocity);
  Eigen::Quaternionf k4 = quat_derivative(
      add_quat(orientation_quat, scale_quat(k3, 0.5)), angular_velocity);

  return add_quat(orientation_quat,
                  scale_quat(add_quat((k1, scale_quat(k2, 2.0f)),
                                      add_quat(scale_quat(k3, 2.0f), k4)),
                             (dt / 6.0f)));
}

}  // namespace xslam