/*
 * @Author: David-Willo davidwillo@foxmail.com
 * @Date: 2023-07-25 06:50:01
 * @LastEditTime: 2023-08-18 13:26:27
 * @LastEditors: David-Willo
 * Jinhao HE (David Willo), IADC HKUST(GZ)
 * Copyright (c) 2023 by davidwillo@foxmail.com, All Rights Reserved. 
 */
#include "xslam/visual-localization/geo_observer.h"
#include "xslam/visual-localization/wavelet/CWavelet.h"


namespace xslam {



class SequenceFilter {
public:
    SequenceFilter(float noise_variance, float length_scale)
        : noise_variance_(noise_variance), length_scale_(length_scale) {}

    // Add data to the Gaussian Process
    void addData(float timestamp, const Eigen::VectorXf imu_data) {
        timestamps_.push_back(timestamp);
        imu_data_.push_back(imu_data);
    }

    // Compute the kernel function between two timestamps
    float kernel(float t1, float t2) const {
        float time_diff_squared = (t1 - t2) * (t1 - t2);
        return std::exp(-time_diff_squared / (2.0f * length_scale_ * length_scale_));
    }

    void denoise() {
      // std::cout << "Start denoise signal with lenght " << imu_data_.size() << "\n\n";
      wavelet::CWavelet cw(3);
      // std::cout << "Done1\n";
      cw.InitDecInfo(imu_data_.size(), 3, 3);
      // std::cout << "Done2\n";
      for (size_t i = 0; i < 6; ++i) {
        std::vector<double> raw, denoised(imu_data_.size());
        imuDataVectorToMatrixRaw(i, raw);
        // DW: soft threhold seems better for sudden move
        bool succ = cw.thrDenoise(raw.data(), denoised.data(), false);
        // std::cout << "denoise success ? " << succ << " ...\n\n";
        raw2DataVector(i, denoised);
      }
      // std::cout << "Done denoise\n";
    }

    bool getDenoised(float t_tgt,  Eigen::VectorXf& data) {
      
      for(size_t i=0; i < timestamps_.size(); i++) {
        if (std::fabs(timestamps_[i] == t_tgt)) {
          data = imu_data_[i];
          return true;
        }
      }
      std::cout << "Fail to get denoised data at " << t_tgt << "\n\n";
      return false;
    }

    // Predict the mean and variance at a new timestamp t_pred
    void predict(float t_pred, Eigen::VectorXf& mean, float& variance) const {
        int n = timestamps_.size();

        // Compute the covariance matrix K(X, X) and K(X, t_pred)
        Eigen::MatrixXf KXX(n, n);
        Eigen::VectorXf KX_pred(n);

        for (int i = 0; i < n; ++i) {
            KX_pred(i) = kernel(timestamps_[i], t_pred);

            for (int j = 0; j < n; ++j) {
                KXX(i, j) = kernel(timestamps_[i], timestamps_[j]);
            }
        }

        // Add noise variance to the diagonal elements
        for (int i = 0; i < n; ++i) {
            KXX(i, i) += noise_variance_;
        }

        // Compute the predictive mean and variance
        Eigen::MatrixXf inv_KXX = KXX.inverse();
        float kX_pred_X_pred = kernel(t_pred, t_pred);
        variance = kX_pred_X_pred - KX_pred.transpose() * inv_KXX * KX_pred;
        mean = KX_pred.transpose() * inv_KXX * imuDataVectorToMatrix();
    }

private:
    float noise_variance_; // Variance of the noise in the data
    float length_scale_;   // Length scale hyperparameter
    std::vector<float> timestamps_;      // Vector to store timestamps
    std::deque<Eigen::VectorXf> imu_data_; // Vector to store IMU data vectors

    // Helper function to concatenate IMU data vectors into a matrix
    Eigen::MatrixXf imuDataVectorToMatrix() const {
        int n = imu_data_.size();
        int data_dim = imu_data_[0].size();
        Eigen::MatrixXf imu_data_matrix(n, data_dim);

        for (int i = 0; i < n; ++i) {
            imu_data_matrix.row(i) = imu_data_[i];
        }

        return imu_data_matrix;
    }
    void imuDataVectorToMatrixRaw(size_t dim, std::vector<double>& data) const {
      int n = imu_data_.size();
      data.clear();
      for (int i = 0; i < n; ++i) {
          data.push_back(imu_data_[i][dim]);
      }
    }

    void raw2DataVector(size_t dim, std::vector<double>& data)  {
      int n = imu_data_.size();
      for (int i = 0; i < n; ++i) {
          imu_data_[i][dim] = data[i];
      }
    }
};



GeoObserver::GeoObserver(ros::NodeHandle nh, Options options) : nh_(nh) {
    // imu related
  this->first_imu_received = false;
  this->first_img_valid = false;
  this->observer_inited_ = false;
  
  this->state.p = Eigen::Vector3f(0., 0., 0.);
  this->state.q = Eigen::Quaternionf(1., 0., 0., 0.);
  this->state.v.lin.b = Eigen::Vector3f(0., 0., 0.);
  this->state.v.lin.w = Eigen::Vector3f(0., 0., 0.);
  this->state.v.ang.b = Eigen::Vector3f(0., 0., 0.);
  this->state.v.ang.w = Eigen::Vector3f(0., 0., 0.);

  this->imu_meas.stamp = 0.;
  this->imu_meas.ang_vel[0] = 0.;
  this->imu_meas.ang_vel[1] = 0.;
  this->imu_meas.ang_vel[2] = 0.;
  this->imu_meas.lin_accel[0] = 0.;
  this->imu_meas.lin_accel[1] = 0.;
  this->imu_meas.lin_accel[2] = 0.;

  this->imu_buffer.set_capacity(this->imu_buffer_size_);
  this->first_imu_stamp = 0.;
  this->first_cam_stamp = 0.;
  this->prev_imu_stamp = 0.;
  this->state.b.accel = Eigen::Vector3f(0., 0., 0.);
  this->state.b.gyro = Eigen::Vector3f(0., 0., 0.);
  this->imu_accel_sm_ = Eigen::Matrix3f::Identity();

  this->imu_calibrated = false;
//   this->extrinsics.baselink2imu_T = Eigen::Matrix4f::Identity();

  // here baselink is camera frame, imu is the same as lidar frame
  this->extrinsics.baselink2imu.t =
      Eigen::Vector3f(-0.0742048, -0.492484, 0.0471882);
  this->extrinsics.baselink2imu.R = Eigen::Matrix3f::Identity();
  this->extrinsics.baselink2imu.R << 0.0225817, -0.999662, 0.0129064, 0.301787,
      -0.00549139, -0.953359, 0.953108, 0.0254235, 0.301561;

  
  Eigen::Vector3f imu_diff(0.011, 0.2329, -0.04412);
  this->extrinsics.baselink2imu.t+=this->extrinsics.baselink2imu.R*imu_diff;


  Eigen::Matrix3f Rcb;
  Rcb <<  0.0171327,    -0.99982, -0.00809888,
           0.0685358, 0.00925536,  -0.997606,
           0.997502,  0.0165366,   0.068682;
  Eigen::Vector3f tcb(-0.0710505,  -0.344159,  -0.0341766);
  this->extrinsics.cam2lidar.R = Rcb;
  this->extrinsics.cam2lidar.t = tcb;
  // this->extrinsics.baselink2cam_T.block(0, 3, 3, 1) = this->extrinsics.baselink2cam.t;
  // this->extrinsics.baselink2cam_T.block(0, 0, 3, 3) = this->extrinsics.baselink2cam.R;
  // this->extrinsics.baselink2imu.R = Rcb.transpose()*this->extrinsics.baselink2imu.R;
  // this->extrinsics.baselink2imu.t = Rcb.transpose()*this->extrinsics.baselink2imu.t-Rcb.transpose()*tcb;
//   this->extrinsics.baselink2imu_T.block(0, 3, 3, 1) =
//       this->extrinsics.baselink2imu.t;
//   this->extrinsics.baselink2imu_T.block(0, 0, 3, 3) =
//       this->extrinsics.baselink2imu.R;

  this->grav_vec = Eigen::Vector3f(0, 0, gravity_);
  this->grav_vec_w = Eigen::Vector3f(0, 0, gravity_);

  // ros
  pub_odom_prior_ = nh_.advertise<nav_msgs::Odometry>("/current_odom_prior", 10);
  pub_odom_imu_ = nh_.advertise<nav_msgs::Odometry>("/current_odom_imu", 10);
  pub_imu_raw = nh_.advertise<sensor_msgs::Imu>("/imu_data_raw", 10);
  pub_imu_filtered = nh_.advertise<sensor_msgs::Imu>("/imu_data_filtered", 10);
  pub_path_imu_ = nh_.advertise<nav_msgs::Path>("/path_imu",1);
  pub_path_prior_ = nh_.advertise<nav_msgs::Path>("/path_prior",1);
}

sensor_msgs::Imu::Ptr GeoObserver::transformImu(
    const sensor_msgs::Imu::ConstPtr& imu_raw) {
  sensor_msgs::Imu::Ptr imu(new sensor_msgs::Imu);

  // Copy header
  imu->header = imu_raw->header;

  static double prev_stamp = imu->header.stamp.toSec();
  double dt = imu->header.stamp.toSec() - prev_stamp;
  prev_stamp = imu->header.stamp.toSec();

  if (dt == 0) {
    dt = 1.0 / 200.0;
  }

  // Transform angular velocity (will be the same on a rigid body, so just
  // rotate to ROS convention)
  Eigen::Vector3f ang_vel(imu_raw->angular_velocity.x,
                          imu_raw->angular_velocity.y,
                          imu_raw->angular_velocity.z);

  Eigen::Vector3f ang_vel_cg = this->extrinsics.baselink2imu.R * ang_vel;

  imu->angular_velocity.x = ang_vel_cg[0];
  imu->angular_velocity.y = ang_vel_cg[1];
  imu->angular_velocity.z = ang_vel_cg[2];

  static Eigen::Vector3f ang_vel_cg_prev = ang_vel_cg;

  // Transform linear acceleration (need to account for component due to
  // translational difference)
  Eigen::Vector3f lin_accel(imu_raw->linear_acceleration.x,
                            imu_raw->linear_acceleration.y,
                            imu_raw->linear_acceleration.z);
  // the dimension of the acceleration data from the build-in imu in Livox MID-360 is g(gravity) instead of m/s^2
  Eigen::Vector3f lin_accel_cg = this->extrinsics.baselink2imu.R * (lin_accel *this->gravity_);

  lin_accel_cg =
      lin_accel_cg +
      ((ang_vel_cg - ang_vel_cg_prev) / dt)
          .cross(-this->extrinsics.baselink2imu.t) +
      ang_vel_cg.cross(ang_vel_cg.cross(-this->extrinsics.baselink2imu.t));

  ang_vel_cg_prev = ang_vel_cg;

  imu->linear_acceleration.x = lin_accel_cg[0];
  imu->linear_acceleration.y = lin_accel_cg[1];
  imu->linear_acceleration.z = lin_accel_cg[2];

  return imu;
}

void GeoObserver::calibrateImu(const sensor_msgs::Imu::ConstPtr& imu) {
    static int num_samples = 0;
    static Eigen::Vector3f gyro_avg(0., 0., 0.);
    static Eigen::Vector3f accel_avg(0., 0., 0.);
    static bool print = true;

    Eigen::Vector3f lin_accel;
    Eigen::Vector3f ang_vel;

    // Get IMU samples
    ang_vel[0] = imu->angular_velocity.x;
    ang_vel[1] = imu->angular_velocity.y;
    ang_vel[2] = imu->angular_velocity.z;

    lin_accel[0] = imu->linear_acceleration.x;
    lin_accel[1] = imu->linear_acceleration.y;
    lin_accel[2] = imu->linear_acceleration.z;

    if ((imu->header.stamp.toSec() - this->first_imu_stamp) <
        this->imu_calib_time_) {
      num_samples++;

      gyro_avg[0] += ang_vel[0];
      gyro_avg[1] += ang_vel[1];
      gyro_avg[2] += ang_vel[2];

      accel_avg[0] += lin_accel[0];
      accel_avg[1] += lin_accel[1];
      accel_avg[2] += lin_accel[2];

      if (print) {
        WARNLOG(" \nCalibrating IMU for {} seconds... ", this->imu_calib_time_);
        print = false;
      }

    } else {
      WARNLOG("done\n\n");

      gyro_avg /= num_samples;
      accel_avg /= num_samples;

      // assume global grav_vec downward facing
      this->grav_vec << 0., 0., this->gravity_;
      // Eigen::Vector3f grav_vec( 0.,this->gravity_, 0.);

      // Estimated gravity vector - Only approximate if biases have not been pre-calibrated
      this->grav_vec = (accel_avg - this->state.b.accel).normalized() * abs(this->gravity_);
      updateGravity();

      if (this->gravity_align_) {

        // Estimate gravity vector - Only approximate if biases have not been pre-calibrated
        grav_vec = (accel_avg - this->state.b.accel).normalized() * abs(this->gravity_);
        Eigen::Quaternionf grav_q = Eigen::Quaternionf::FromTwoVectors(grav_vec, Eigen::Vector3f(0., 0., this->gravity_));

        // set gravity aligned orientation
        this->state.q = grav_q;
        // this->T.block(0,0,3,3) = this->state.q.toRotationMatrix();
        this->camPose.q = this->state.q;

        // rpy
        auto euler = grav_q.toRotationMatrix().eulerAngles(2, 1, 0);
        double yaw = euler[0] * (180.0/M_PI);
        double pitch = euler[1] * (180.0/M_PI);
        double roll = euler[2] * (180.0/M_PI);

        // use alternate representation if the yaw is smaller
        if (abs(remainder(yaw + 180.0, 360.0)) < abs(yaw)) {
          yaw   = remainder(yaw + 180.0,   360.0);
          pitch = remainder(180.0 - pitch, 360.0);
          roll  = remainder(roll + 180.0,  360.0);
        }
        std::cout << " Estimated initial attitude:" << std::endl;
        std::cout << "   Roll  [deg]: " << to_string_with_precision(roll, 4) << std::endl;
        std::cout << "   Pitch [deg]: " << to_string_with_precision(pitch, 4) << std::endl;
        std::cout << "   Yaw   [deg]: " << to_string_with_precision(yaw, 4) << std::endl;
        std::cout << std::endl;
      }

      if (this->calibrate_accel_) {
        // subtract gravity from avg accel to get bias
        this->state.b.accel = accel_avg - grav_vec;

        std::cout << " Accel biases [xyz]: "
                  << to_string_with_precision(this->state.b.accel[0], 8) << ", "
                  << to_string_with_precision(this->state.b.accel[1], 8) << ", "
                  << to_string_with_precision(this->state.b.accel[2], 8)
                  << std::endl;
      }

      if (this->calibrate_gyro_) {
        this->state.b.gyro = gyro_avg;

        std::cout << " Gyro biases  [xyz]: "
                  << to_string_with_precision(this->state.b.gyro[0], 8) << ", "
                  << to_string_with_precision(this->state.b.gyro[1], 8) << ", "
                  << to_string_with_precision(this->state.b.gyro[2], 8)
                  << std::endl;
      }

      this->imu_calibrated = true;
    }
}

void GeoObserver::consumeImu(
    const sensor_msgs::Imu::ConstPtr& imu_raw) {
  this->first_imu_received = true;
  // all to body frame
  sensor_msgs::Imu::Ptr imu = this->transformImu(imu_raw);

  if (this->first_imu_stamp == 0.) {
    this->first_imu_stamp = imu->header.stamp.toSec();
  }

  Eigen::Vector3f lin_accel;
  Eigen::Vector3f ang_vel;

  // Get IMU samples
  ang_vel[0] = imu->angular_velocity.x;
  ang_vel[1] = imu->angular_velocity.y;
  ang_vel[2] = imu->angular_velocity.z;

  lin_accel[0] = imu->linear_acceleration.x;
  lin_accel[1] = imu->linear_acceleration.y;
  lin_accel[2] = imu->linear_acceleration.z;

  // IMU calibration procedure - do for three seconds
  if (!this->imu_calibrated) {
    calibrateImu(imu);
  } else {
    if (!this->observer_inited_) {
      WARNLOG("skip consume, observer not ready");
      return;
    }
    double dt = imu->header.stamp.toSec() - this->prev_imu_stamp;
    if (dt == 0 || this->prev_imu_stamp==0) {
      dt = 1.0 / 200.0;
    }
    // WARNLOG("IMU rate {}",  1./dt );
    // this->imu_rates.push_back(1. / dt);

    // Apply the calibrated bias to the new IMU measurements
    this->imu_meas.stamp = imu->header.stamp.toSec();
    this->imu_meas.dt = dt;
    this->prev_imu_stamp = this->imu_meas.stamp;

    Eigen::Vector3f lin_accel_corrected =
        (this->imu_accel_sm_ * lin_accel) - this->state.b.accel;
    Eigen::Vector3f ang_vel_corrected = ang_vel - this->state.b.gyro;

    this->imu_meas.lin_accel = lin_accel_corrected;
    this->imu_meas.ang_vel = ang_vel_corrected;

    // Store calibrated IMU measurements into imu buffer for manual integration
    // later.
    // WARNLOG("imu to buffer");
    this->mtx_imu.lock();
    this->imu_buffer.push_front(this->imu_meas);
    this->mtx_imu.unlock();
    // WARNLOG("imu to buffer done {}", this->imu_meas.stamp);
    // Notify the callbackimage thread that IMU data exists for this time
    this->cv_imu_stamp.notify_one();

    if (this->geo.first_opt_done && imu->header.stamp.toSec() > cur_cam_stamp) {
      // only propagate state when first pose ready and imu faster than cur camera,
      // when system relocalized, may propagate on a future state with old imu measurements
      this->propagateState();
    }
  }
  
}

void GeoObserver::updateGravity() {
  // Transform gravity from body to world frame
  this->grav_vec_w =  this->state.q._transformVector(this->grav_vec);
  WARNLOG("\n\n\n body g{} {} {}", this->grav_vec.x(), this->grav_vec.y(), this->grav_vec.z());
  WARNLOG("\n\n\n world g {} {} {}", this->grav_vec_w.x(), this->grav_vec_w.y(), this->grav_vec_w.z())
  // throw std::runtime_error("gravity");
}

void GeoObserver::propagateCamera(const VisualFrame::Ptr& frame, bool force) {
  cur_cam_stamp = frame->timestamp_second();
  // this->camPose.p = frame->getTwb().rotationMatrix().cast<float>()*this->extrinsics.cam2baselink.t+frame->getTwb().translation().cast<float>() ;
  // this->camPose.q = frame->getTwb().rotationMatrix().cast<float>()*this->extrinsics.cam2baselink.R;
  this->camPose.p = frame->getTwb().translation().cast<float>();
  this->camPose.q = frame->getTwb().rotationMatrix().cast<float>();
  if (force) {
    // for reset system states, the geometry observer may be unstable
    this->state.p = this->camPose.p;
    this->state.q = this->camPose.q;
    // this->state.v.lin.b = Eigen::Vector3f(0., 0., 0.);
    // this->state.v.lin.w = Eigen::Vector3f(0., 0., 0.);
    // this->state.v.ang.b = Eigen::Vector3f(0., 0., 0.);
    // this->state.v.ang.w = Eigen::Vector3f(0., 0., 0.);
    this->first_cam_stamp = frame->timestamp_second();
  }
  
}

void GeoObserver::updateHistory() {
  prev_cam_stamp = cur_cam_stamp;
  geo.first_opt_done = true;
}

void GeoObserver::propagateState() {
//   WARNLOG("start propagate state");
  // Lock thread to prevent state from being accessed by UpdateState
  std::lock_guard<std::mutex> lock(this->geo.mtx);

  double dt = this->imu_meas.dt;

  Eigen::Quaternionf qhat = this->state.q, omega;
  Eigen::Vector3f world_accel;

  // Transform accel from body to world frame
  world_accel = qhat._transformVector(this->imu_meas.lin_accel);

  // Accel propogation
  // TODO(DW) should be world gravity   
  // this->state.p[0] +=
  //     this->state.v.lin.w[0] * dt + 0.5 * dt * dt * world_accel[0];
  // this->state.p[1] +=
  //     this->state.v.lin.w[1] * dt + 0.5 * dt * dt * world_accel[1];
  // this->state.p[2] += this->state.v.lin.w[2] * dt +
  //                     0.5 * dt * dt * (world_accel[2] - this->gravity_);

  // Accel propogation
  // remove gravity
  world_accel -= this->grav_vec_w;
  // update p
  this->state.p[0] +=
      this->state.v.lin.w[0] * dt + 0.5 * dt * dt * world_accel[0];
  this->state.p[1] +=
      this->state.v.lin.w[1] * dt + 0.5 * dt * dt * world_accel[1];
  this->state.p[2] +=
      this->state.v.lin.w[2] * dt + 0.5 * dt * dt * world_accel[2];
  // this->state.p +=  this->state.v.lin.w *  dt+ 0.5 * dt * dt * world_accel;

  // update v
  this->state.v.lin.w[0] += world_accel[0] * dt;
  this->state.v.lin.w[1] += world_accel[1] * dt;
  this->state.v.lin.w[2] += world_accel[2] * dt;
  this->state.v.lin.b =
      this->state.q.toRotationMatrix().inverse() * this->state.v.lin.w;

  // this->state.v.lin.w += world_accel * dt;
  // this->state.v.lin.b =
  //     this->state.q.toRotationMatrix().inverse() * this->state.v.lin.w;

  // Gyro propogation
  // rk1 update   
  omega.w() = 0;
  omega.vec() = this->imu_meas.ang_vel;
  Eigen::Quaternionf tmp = qhat * omega;
  this->state.q.w() += 0.5 * dt * tmp.w();
  this->state.q.vec() += 0.5 * dt * tmp.vec();
  // rk4 update 
  // this->state.q = rk4_step(this->state.q,  this->imu_meas.ang_vel, dt);
  
  // Ensure quaternion is properly normalized
  this->state.q.normalize();

  this->state.v.ang.b = this->imu_meas.ang_vel;
  this->state.v.ang.w = this->state.q.toRotationMatrix() * this->state.v.ang.b;

  static tf2_ros::TransformBroadcaster br;
  {
    // SE3 pose = this->T_prior;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "imu";

    Vector3 t_w_c = this->state.p.cast<double>();  // pose.translation();
    Quaternion rot_w_c =
        this->state.q.cast<double>();  // pose.unit_quaternion();
    transformStamped.transform.translation.x = t_w_c.x();
    transformStamped.transform.translation.y = t_w_c.y();
    transformStamped.transform.translation.z = t_w_c.z();

    transformStamped.transform.rotation.x = rot_w_c.x();
    transformStamped.transform.rotation.y = rot_w_c.y();
    transformStamped.transform.rotation.z = rot_w_c.z();
    transformStamped.transform.rotation.w = rot_w_c.w();
    br.sendTransform(transformStamped);


    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "map";

    odom.pose.pose.position.x = t_w_c.x();
    odom.pose.pose.position.y = t_w_c.y();
    odom.pose.pose.position.z = t_w_c.z();

    odom.pose.pose.orientation.x = rot_w_c.x();
    odom.pose.pose.orientation.y = rot_w_c.y();
    odom.pose.pose.orientation.z = rot_w_c.z();
    odom.pose.pose.orientation.w = rot_w_c.w();

    pub_odom_imu_.publish(odom);

    path_imu_msg_.header.stamp = ros::Time::now();
    path_imu_msg_.header.frame_id = "map";
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.header.frame_id = "imu"; // Replace with your desired frame ID

    pose_stamped.pose.position.x = t_w_c.x();
    pose_stamped.pose.position.y = t_w_c.y();
    pose_stamped.pose.position.z = t_w_c.z();

    path_imu_msg_.poses.push_back(pose_stamped);
    pub_path_imu_.publish(path_imu_msg_);
  }
//   WARNLOG("done propagate state");
}

void GeoObserver::updateState() {

  if (!this->observer_inited_) {
    WARNLOG("skip update, observer not ready");
    return;
  }

  // Lock thread to prevent state from being accessed by PropagateState
  std::lock_guard<std::mutex> lock(this->geo.mtx);

  Eigen::Vector3f pin = this->camPose.p;
  Eigen::Quaternionf qin = this->camPose.q;
  double dt = this->cur_cam_stamp - this->prev_cam_stamp;

  Eigen::Quaternionf qe, qhat, qcorr;
  qhat = this->state.q;

  // Constuct error quaternion
  qe = qhat.conjugate() * qin;

  double sgn = 1.;
  if (qe.w() < 0) {
    sgn = -1;
  }

  // Construct quaternion correction
  qcorr.w() = 1 - abs(qe.w());
  qcorr.vec() = sgn * qe.vec();
  qcorr = qhat * qcorr;

  Eigen::Vector3f err = pin - this->state.p;
  Eigen::Vector3f err_body;

  err_body = qhat.conjugate()._transformVector(err);

  double abias_max = this->geo_abias_max_;
  double gbias_max = this->geo_gbias_max_;

  // Update accel bias
  this->state.b.accel -= dt * this->geo_Kab_ * err_body;
  this->state.b.accel =
      this->state.b.accel.array().min(abias_max).max(-abias_max);

  // Update gyro bias
  this->state.b.gyro[0] -= dt * this->geo_Kgb_ * qe.w() * qe.x();
  this->state.b.gyro[1] -= dt * this->geo_Kgb_ * qe.w() * qe.y();
  this->state.b.gyro[2] -= dt * this->geo_Kgb_ * qe.w() * qe.z();
  this->state.b.gyro =
      this->state.b.gyro.array().min(gbias_max).max(-gbias_max);

  // Update state
  this->state.p += dt * this->geo_Kp_ * err;
  this->state.v.lin.w += dt * this->geo_Kv_ * err;

  this->state.q.w() += dt * this->geo_Kq_ * qcorr.w();
  this->state.q.x() += dt * this->geo_Kq_ * qcorr.x();
  this->state.q.y() += dt * this->geo_Kq_ * qcorr.y();
  this->state.q.z() += dt * this->geo_Kq_ * qcorr.z();
  this->state.q.normalize();

  // store previous pose, orientation, and velocity
  this->geo.prev_p = this->state.p;
  this->geo.prev_q = this->state.q;
  this->geo.prev_vel = this->state.v.lin.w;

  // WARNLOG("done update state");
}

bool GeoObserver::imuMeasFromTimeRange(
    double start_time, double end_time,
    boost::circular_buffer<ImuMeas>::reverse_iterator& begin_imu_it,
    boost::circular_buffer<ImuMeas>::reverse_iterator& end_imu_it) {
  if (this->imu_buffer.empty() || this->imu_buffer.front().stamp < end_time) {
    // Wait for the latest IMU data
    WARNLOG("waiting imu, end_time {} but front imu {}", end_time,
            this->imu_buffer.front().stamp);
    std::unique_lock<decltype(this->mtx_imu)> lock(this->mtx_imu);
    this->cv_imu_stamp.wait(lock, [this, &end_time] {
      return this->imu_buffer.front().stamp >= end_time;
    });
  }

  // WARNLOG("Try search imu measurements between {} {} from {} buffer {}-{}",
  //         start_time, end_time, imu_buffer.size(), imu_buffer.back().stamp,
  //         imu_buffer.front().stamp);
 
  auto imu_it = this->imu_buffer.begin();
  auto last_imu_it = imu_it;
  imu_it++;
  while (imu_it != this->imu_buffer.end() && imu_it->stamp >= end_time) {
    last_imu_it = imu_it;
    imu_it++;
  }

  while (imu_it != this->imu_buffer.end() && imu_it->stamp >= start_time) {
    imu_it++;
  }

  if (imu_it == this->imu_buffer.end()) {
    WARNLOG("not enough imu measurements");
    // not enough IMU measurements, return false
    return false;
  }
  imu_it++;

  // Set reverse iterators (to iterate forward in time)
  end_imu_it = boost::circular_buffer<ImuMeas>::reverse_iterator(last_imu_it);
  begin_imu_it = boost::circular_buffer<ImuMeas>::reverse_iterator(imu_it);

  return true;
}

std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
GeoObserver::integrateImu(double start_time, Eigen::Quaternionf q_init,
                                 Eigen::Vector3f p_init, Eigen::Vector3f v_init,
                                 const std::vector<double>& sorted_timestamps) {
  const std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
      empty;

  if (sorted_timestamps.empty() || start_time > sorted_timestamps.front()) {
    // invalid input, return empty vector
    return empty;
  }

  boost::circular_buffer<ImuMeas>::reverse_iterator begin_imu_it;
  boost::circular_buffer<ImuMeas>::reverse_iterator end_imu_it;
  if (this->imuMeasFromTimeRange(start_time, sorted_timestamps.back(),
                                 begin_imu_it, end_imu_it) == false) {
    // not enough IMU measurements, return empty vector
    return empty;
  }

  // Backwards integration to find pose at first IMU sample
  const ImuMeas& f1 = *begin_imu_it;
  const ImuMeas& f2 = *(begin_imu_it + 1);

  // Time between first two IMU samples
  double dt = f2.dt;

  // Time between first IMU sample and start_time
  double idt = start_time - f1.stamp;

  // Angular acceleration between first two IMU samples
  Eigen::Vector3f alpha_dt = f2.ang_vel - f1.ang_vel;
  Eigen::Vector3f alpha = alpha_dt / dt;

  // Average angular velocity (reversed) between first IMU sample and start_time
  Eigen::Vector3f omega_i = -(f1.ang_vel + 0.5 * alpha * idt);

  // Set q_init to orientation at first IMU sample
  q_init = Eigen::Quaternionf(
      q_init.w() - 0.5 *
                       (q_init.x() * omega_i[0] + q_init.y() * omega_i[1] +
                        q_init.z() * omega_i[2]) *
                       idt,
      q_init.x() + 0.5 *
                       (q_init.w() * omega_i[0] - q_init.z() * omega_i[1] +
                        q_init.y() * omega_i[2]) *
                       idt,
      q_init.y() + 0.5 *
                       (q_init.z() * omega_i[0] + q_init.w() * omega_i[1] -
                        q_init.x() * omega_i[2]) *
                       idt,
      q_init.z() + 0.5 *
                       (q_init.x() * omega_i[1] - q_init.y() * omega_i[0] +
                        q_init.w() * omega_i[2]) *
                       idt);
  q_init.normalize();

  // Average angular velocity between first two IMU samples
  Eigen::Vector3f omega = f1.ang_vel + 0.5 * alpha_dt;

  // Orientation at second IMU sample
  Eigen::Quaternionf q2(
      q_init.w() - 0.5 *
                       (q_init.x() * omega[0] + q_init.y() * omega[1] +
                        q_init.z() * omega[2]) *
                       dt,
      q_init.x() + 0.5 *
                       (q_init.w() * omega[0] - q_init.z() * omega[1] +
                        q_init.y() * omega[2]) *
                       dt,
      q_init.y() + 0.5 *
                       (q_init.z() * omega[0] + q_init.w() * omega[1] -
                        q_init.x() * omega[2]) *
                       dt,
      q_init.z() + 0.5 *
                       (q_init.x() * omega[1] - q_init.y() * omega[0] +
                        q_init.w() * omega[2]) *
                       dt);
  q2.normalize();

  // Acceleration at first IMU sample
  Eigen::Vector3f a1 = q_init._transformVector(f1.lin_accel);
  // a1[2] -= this->gravity_;
  a1 -= this->grav_vec_w;

  // Acceleration at second IMU sample
  Eigen::Vector3f a2 = q2._transformVector(f2.lin_accel);
  // a2[2] -= this->gravity_;
  a2 -= this->grav_vec_w;

  // Jerk between first two IMU samples
  Eigen::Vector3f j = (a2 - a1) / dt;

  // Set v_init to velocity at first IMU sample (go backwards from start_time)
  v_init -= a1 * idt + 0.5 * j * idt * idt;

  // Set p_init to position at first IMU sample (go backwards from start_time)
  p_init -=
      v_init * idt + 0.5 * a1 * idt * idt + (1 / 6.) * j * idt * idt * idt;

  return this->integrateImuInternal(q_init, p_init, v_init, sorted_timestamps,
                                    begin_imu_it, end_imu_it);
}

std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
GeoObserver::integrateImuInternal(
    Eigen::Quaternionf q_init, Eigen::Vector3f p_init, Eigen::Vector3f v_init,
    const std::vector<double>& sorted_timestamps,
    boost::circular_buffer<ImuMeas>::reverse_iterator begin_imu_it,
    boost::circular_buffer<ImuMeas>::reverse_iterator end_imu_it) {
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
      imu_se3;

  // Initialization
  Eigen::Quaternionf q = q_init;
  Eigen::Vector3f p = p_init;
  Eigen::Vector3f v = v_init;
  Eigen::Vector3f a = q._transformVector(begin_imu_it->lin_accel);
  // a[2] -= this->gravity_;
  a -= this->grav_vec_w;

  // Iterate over IMU measurements and timestamps
  auto prev_imu_it = begin_imu_it;
  auto imu_it = prev_imu_it + 1;
  

  auto stamp_it = sorted_timestamps.begin();
  size_t imu_candidates = 0;

  // Convert noise values to appropriate units and square them to get variances
  float gyro_noise_rms = 4.5f / 1000.0f; // Convert mdps to degrees per second
  // float accel_noise_rms = (100.0f / 1000000.0f) * 9.81f; // Convert μg to m/s^2
  float accel_noise_rms = (100.0f / 1000000.0f) * this->gravity_; // Convert μg to m/s^2
  float gyro_variance = gyro_noise_rms * gyro_noise_rms;
  float accel_variance = accel_noise_rms * accel_noise_rms;

  // Scale the variances by the reciprocal of the sampling frequency (1/Fs)
  float sampling_frequency = 200.0f; // Hz
  gyro_variance /= sampling_frequency;
  accel_variance /= sampling_frequency;
  // WARNLOG("{} {}", gyro_variance, accel_variance);

  SequenceFilter gp(0.00001, 0.01);
  gp.addData(prev_imu_it->stamp, imu2data(*prev_imu_it));
  for (; imu_it != end_imu_it; imu_it++) {
    gp.addData(imu_it->stamp, imu2data(*imu_it));
  }
  // reset iterator
  prev_imu_it = begin_imu_it;
  imu_it = prev_imu_it + 1;
  gp.denoise();
  for (; imu_it != end_imu_it; imu_it++) {
    imu_candidates++;
    Eigen::VectorXf mean;
    float variance;
    // sensor_msgs::Imu imu_msg;
    // imu_msg.header.stamp = ros::Time::now();
    // imu_msg.header.frame_id = "current_lidar";
    // imu_msg.angular_velocity.x = prev_imu_it->ang_vel.x();
    // imu_msg.angular_velocity.y = prev_imu_it->ang_vel.y();
    // imu_msg.angular_velocity.z = prev_imu_it->ang_vel.z();
    // imu_msg.linear_acceleration.x = prev_imu_it->lin_accel.x();
    // imu_msg.linear_acceleration.y =  prev_imu_it->lin_accel.y();
    // imu_msg.linear_acceleration.z =  prev_imu_it->lin_accel.z();
    // pub_imu_raw.publish(imu_msg);

    // gp.getDenoised(prev_imu_it->stamp, mean);
    // ImuMeas f0 = data2imu(mean, prev_imu_it->stamp, prev_imu_it->dt);

    // imu_msg.angular_velocity.x = f0.ang_vel.x();
    // imu_msg.angular_velocity.y = f0.ang_vel.y();
    // imu_msg.angular_velocity.z = f0.ang_vel.z();
    // imu_msg.linear_acceleration.x = f0.lin_accel.x();
    // imu_msg.linear_acceleration.y =  f0.lin_accel.y();
    // imu_msg.linear_acceleration.z =  f0.lin_accel.z();
    // pub_imu_filtered.publish(imu_msg);


    // gp.getDenoised(imu_it->stamp, mean);
    // ImuMeas f = data2imu(mean, imu_it->stamp, imu_it->dt);

    const ImuMeas& f0 = *prev_imu_it;
    const ImuMeas& f = *imu_it;

    // Time between IMU samples
    double dt = f.dt;

    // Angular acceleration
    Eigen::Vector3f alpha_dt = f.ang_vel - f0.ang_vel;
    Eigen::Vector3f alpha = alpha_dt / dt;

    // Average angular velocity
    Eigen::Vector3f omega = f0.ang_vel + 0.5 * alpha_dt;

    // Orientation
    q = Eigen::Quaternionf(
        q.w() -
            0.5 * (q.x() * omega[0] + q.y() * omega[1] + q.z() * omega[2]) * dt,
        q.x() +
            0.5 * (q.w() * omega[0] - q.z() * omega[1] + q.y() * omega[2]) * dt,
        q.y() +
            0.5 * (q.z() * omega[0] + q.w() * omega[1] - q.x() * omega[2]) * dt,
        q.z() + 0.5 * (q.x() * omega[1] - q.y() * omega[0] + q.w() * omega[2]) *
                    dt);
    q.normalize();

    // Acceleration
    Eigen::Vector3f a0 = a;
    a = q._transformVector(f.lin_accel);
    // a[2] -= this->gravity_;
    a -= this->grav_vec_w;

    // Jerk
    Eigen::Vector3f j_dt = a - a0;
    Eigen::Vector3f j = j_dt / dt;

    // Interpolate for given timestamps
    while (stamp_it != sorted_timestamps.end() && *stamp_it <= f.stamp) {
      // Time between previous IMU sample and given timestamp
      double idt = *stamp_it - f0.stamp;

      // Average angular velocity
      Eigen::Vector3f omega_i = f0.ang_vel + 0.5 * alpha * idt;

      // Orientation
      Eigen::Quaternionf q_i(
          q.w() - 0.5 *
                      (q.x() * omega_i[0] + q.y() * omega_i[1] +
                       q.z() * omega_i[2]) *
                      idt,
          q.x() + 0.5 *
                      (q.w() * omega_i[0] - q.z() * omega_i[1] +
                       q.y() * omega_i[2]) *
                      idt,
          q.y() + 0.5 *
                      (q.z() * omega_i[0] + q.w() * omega_i[1] -
                       q.x() * omega_i[2]) *
                      idt,
          q.z() + 0.5 *
                      (q.x() * omega_i[1] - q.y() * omega_i[0] +
                       q.w() * omega_i[2]) *
                      idt);
      q_i.normalize();

      // Position
      Eigen::Vector3f p_i =
          p + v * idt + 0.5 * a0 * idt * idt + (1 / 6.) * j * idt * idt * idt;

      // Transformation
      Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
      T.block(0, 0, 3, 3) = q_i.toRotationMatrix();
      T.block(0, 3, 3, 1) = p_i;

      imu_se3.push_back(T);

      stamp_it++;
    }

    // Position
    p += v * dt + 0.5 * a0 * dt * dt + (1 / 6.) * j_dt * dt * dt;

    // Velocity
    v += a0 * dt + 0.5 * j_dt * dt;

    prev_imu_it = imu_it;
  }
  WARNLOG("get imu candidates: {} for priors", imu_candidates);

  return imu_se3;
}

void GeoObserver::getCameraPrior(const VisualFrame::Ptr& frame_last, const VisualFrame::Ptr& frame_cur, SE3& T_prior) {
    // don't process scans until IMU data is present
    if (!first_img_valid) {
        if (imu_buffer.empty() ||
            frame_cur->timestamp_second() <= imu_buffer.back().stamp) {
          WARNLOG("first img not ready");
        return;
        }
        first_img_valid = true;
        T_prior = frame_last->getTwb();  // assume no motion for the first img

    } else {
        // IMU prior for second scan onwards
        std::vector<Eigen::Matrix4f,
                    Eigen::aligned_allocator<Eigen::Matrix4f>>
            imu_frames;

        imu_frames = this->integrateImu(
            frame_last->timestamp_second(), this->camPose.q, this->camPose.p,
            this->geo.prev_vel.cast<float>(), {frame_cur->timestamp_second()});

        
        if (imu_frames.size() > 0) {
            Eigen::Matrix4f imu_last = imu_frames.back();
            Eigen::Matrix3f imu_R = imu_last.topLeftCorner(3, 3);
            Eigen::Quaterniond imu_q(imu_R.cast<double>());
            Eigen::Vector3f imu_t = imu_last.topRightCorner(3, 1);
            T_prior = SE3(imu_q, imu_t.cast<double>());
        
            static tf2_ros::TransformBroadcaster br;
            {
                // SE3 pose = this->T_prior;
                geometry_msgs::TransformStamped transformStamped;

                transformStamped.header.stamp = ros::Time::now();
                transformStamped.header.frame_id = "map";
                transformStamped.child_frame_id = "prior";

                Vector3 t_w_c = T_prior.translation();
                Quaternion rot_w_c =
                    T_prior.unit_quaternion();
                transformStamped.transform.translation.x = t_w_c.x();
                transformStamped.transform.translation.y = t_w_c.y();
                transformStamped.transform.translation.z = t_w_c.z();

                transformStamped.transform.rotation.x = rot_w_c.x();
                transformStamped.transform.rotation.y = rot_w_c.y();
                transformStamped.transform.rotation.z = rot_w_c.z();
                transformStamped.transform.rotation.w = rot_w_c.w();
                br.sendTransform(transformStamped);

                nav_msgs::Odometry odom;
                odom.header.stamp = ros::Time::now();
                odom.header.frame_id = "map";

                odom.pose.pose.position.x = t_w_c.x();
                odom.pose.pose.position.y = t_w_c.y();
                odom.pose.pose.position.z = t_w_c.z();

                odom.pose.pose.orientation.x = rot_w_c.x();
                odom.pose.pose.orientation.y = rot_w_c.y();
                odom.pose.pose.orientation.z = rot_w_c.z();
                odom.pose.pose.orientation.w = rot_w_c.w();

                pub_odom_prior_.publish(odom);


                path_prior_msg_.header.stamp = ros::Time::now();
                path_prior_msg_.header.frame_id = "map";
                geometry_msgs::PoseStamped pose_stamped;
                pose_stamped.header.stamp = ros::Time::now();
                pose_stamped.header.frame_id = "prior"; // Replace with your desired frame ID

                pose_stamped.pose.position.x = t_w_c.x();
                pose_stamped.pose.position.y = t_w_c.y();
                pose_stamped.pose.position.z = t_w_c.z();

                path_prior_msg_.poses.push_back(pose_stamped);
                pub_path_prior_.publish(path_prior_msg_);

            }
        } else {
        // do not update prior
        return;
        }
    }
}


void GeoObserver::initialize() {
  // Wait for IMU
  if (!this->first_imu_received || !this->imu_calibrated) {
    WARNLOG("Observer not ready to initialized!");
    return;
  }

  this->observer_inited_ = true;
  WARNLOG("Observer initialized!");
}


} // namespace xslam