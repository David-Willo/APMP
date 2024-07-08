/*
 * @Author: David-Willo davidwillo@foxmail.com
 * @Date: 2023-04-13 08:32:50
 * @LastEditTime: 2024-07-02 07:10:33
 * @LastEditors: David-Willo
 * Jinhao HE (David Willo), IADC HKUST(GZ)
 * Copyright (c) 2024 by davidwillo@foxmail.com, All Rights Reserved. 
 */
#include "xslam/visual-localization/visual-localization.h"

#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_broadcaster.h>

#include "xslam/common/logging.h"
#include "xslam/common/timing.h"
#include "xslam/visual-features/feature-factory.h"
#include "xslam/visual-features/torch-helper.h"
#include "xslam/visual-localization/map-io.h"

// # define RELOC_EXP 1

namespace xslam {

// std::vector<VisualFrame::Ptr> VisualLocalization::getClusterCovisible(
//     const VisualFrame::ConstPtr& visual_frame) {
//   //
// }
DEFINE_string(image_topic, "/stereo/frame_left/image_raw/compressed",
              "image topic");
DEFINE_string(imu_topic, "", "imu topic");
DEFINE_string(sensor_calibration_file, "", "Path to sensor calibration yaml.");
DEFINE_string(reconstruction_path, "", "Path to colmap reconstruction.");
DEFINE_string(local_feature_path, "", "Path to local image features.");
DEFINE_string(global_feature_path, "", "Path to global image features.");

DEFINE_string(local_feature_model, "", "Model of local feature.");
DEFINE_string(global_feature_model, "", "Model of global feature.");
DEFINE_string(local_feature_model_path, "", "Model path of local feature.");
DEFINE_string(global_feature_model_path, "", "Model path of global feature.");

DEFINE_bool(resize, false, "whether to resize image");
DEFINE_bool(use_fp16, false, "whether to use half precision");
DEFINE_int32(resize_width, 1280, "resized image width");
DEFINE_int32(resize_height, 720, "resized image height");
DEFINE_string(trajectory_path, "", "path for saving trajectory");
DEFINE_string(calib_path, "", "path for extrinsics");

using namespace std;

VisualLocalization::VisualLocalization(ros::NodeHandle& nh,
                                       const Options& options)
    : options_(options), nh_(nh), is_relocalized_(false), is_inited_(false) {

  // load visual map
  auto map_io_options = MapIO::Options()
                            .reconstruction_path(FLAGS_reconstruction_path)
                            .global_feature_path(FLAGS_global_feature_path)
                            .local_feature_path(FLAGS_local_feature_path);
  map_ = MapIO::loadVisualMap(map_io_options);

  // TODO: initialize camera from config file

  camera_ = map_->getAllFrames()[0]->camera();

  // initialize feature extraction
  local_feature_extraction_ = FeatureFactory::createLocalFeature(
      LocalFeature::Options()
          .feature_type(
              FeatureFactory::getLocalFeatureType(FLAGS_local_feature_model))
          .model_path(FLAGS_local_feature_model_path)
          .threshold(0.015)
          .use_fp16(FLAGS_use_fp16));
  INFOLOG("init local extractor");

  global_feature_extraction_ = FeatureFactory::createGlobalFeature(
      GlobalFeature::Options()
          .feature_type(
              FeatureFactory::getGlobalFeatureType(FLAGS_global_feature_model))
          .model_path(FLAGS_global_feature_model_path));
  INFOLOG("global extractor ready");

  relocalization_ = make_shared<Relocalization>(Relocalization::Options());
  relocalization_->setVisualMap(map_);

  INFOLOG("relocalization ready");

  tracking_ = make_shared<Tracking>(Tracking::Options().tracking_on_init(true));
  tracking_->set_visual_map(map_);
  INFOLOG("tracking ready");

  geo_observer_ = make_shared<GeoObserver>(
      nh_, GeoObserver::Options().calib_file(FLAGS_calib_path));
  INFOLOG("geo_observer ready");

  // initialize visualization
  visualization_ = make_shared<VisualizationVLoc>(
      nh_,
      VisualizationVLoc::Options().image_scale(2.0f).image_point_size(2.0f));
  visualization_->setVisualMap(map_);

  thread_visualization_ =
      make_unique<std::thread>(&VisualizationVLoc::spin, visualization_.get());
  unique_ptr<thread>(new std::thread());

  // it_ = std::unique_ptr<image_transport::ImageTransport>(
  //     new image_transport::ImageTransport(nh_));
  // sub_img_ =
  //     it_->subscribe("/camera/image_raw", 1, &LocTrackROS::imageCb, this);

  // image_transport_ = std::unique_ptr<image_transport::ImageTransport>(
  //     new image_transport::ImageTransport(nh_));
  // subscriber_image_ = image_transport_->subscribe(
  //     "/camera/image_raw", 1, &VisualLocalization::imageCallBack, this);
  subscriber_image_ = nh_.subscribe(
      FLAGS_image_topic, 1, &VisualLocalization::compressedImageCallBack, this,
                                      ros::TransportHints().tcpNoDelay());
  if (!FLAGS_imu_topic.empty()) {
    subscriber_imu_ = nh_.subscribe(FLAGS_imu_topic, 1000,
                                        &VisualLocalization::imuCallBack, this,
                                        ros::TransportHints().tcpNoDelay());
    options_.use_imu = true;
  } else {
    options_.use_imu = false;
  }
}

VisualLocalization::~VisualLocalization() {
  // stop viewer
  if (visualization_) {
    visualization_->stop();

    if (thread_visualization_) {
      thread_visualization_->join();
    }
  }
}

void VisualLocalization::compressedImageCallBack(
    const sensor_msgs::CompressedImageConstPtr& msg) {
  cv::Mat image;
  try {
    cv_bridge::CvImagePtr cv_ptr =
        cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    image = cv_ptr->image;

    // cv::imshow("input", image);
    // cv::waitKey(1);
  } catch (const cv_bridge::Exception& e) {
    // ERRLO
    // std::cerr << e.what() << '\n';
  }

  cv::Mat image_inp;
  if (FLAGS_resize) {
    cv::resize(image, image_inp,
               cv::Size(FLAGS_resize_width, FLAGS_resize_height));
  } else {
    image_inp = image;
  }
  // WARNLOG("image callback");
  localize(image_inp, msg->header.stamp);
  // WARNLOG("image callback done");
}

void VisualLocalization::imuCallBack(const sensor_msgs::ImuConstPtr& msg) {
  // DW: no need block here, need to do gravity calibration at the beginning
  // if (!is_inited_) {
  //   return;
  // }

  geo_observer_->consumeImu(msg);
}

void VisualLocalization::localize(const cv::Mat& image,
                                  const ros::Time& timestamp) {
  bool b_extract_global = is_relocalized_ ? false : true;

  // TODO: depend on image input
  timing::Timer timer_extraction(std::string("extraction ") +
                                 (b_extract_global ? "global" : "local"));
  auto frame = createFrameFromImageInput(image, b_extract_global, true);
  frame->set_timestamp(timestamp.toNSec());
  timer_extraction.Stop();



  if (!is_relocalized_) {
    timing::Timer timer_reloc("relocalization");
    auto reloc_result = relocalization_->relocalize(frame);
    if (reloc_result.success) {
      WARNLOG("relocalization succeed with {} inliers.",
              reloc_result.pnp_inliers);
      tracking_->initialize(frame, reloc_result.reference_frame);

      is_relocalized_ = true;
      
      // set global pose observasion and force reset all states
      // if not init yet, also update the gravity
      geo_observer_->propagateCamera(frame, true);
      
      visualization_->publishTransform(frame);

      // for experiment log
      reloc_counter++;

    } else {
      WARNLOG("relocalization failed.");
    }
    timer_reloc.Stop();
  } else {
    SE3 T_prior = frame_last_->getTwb() * delta_pose_;
    
    if (this->options_.use_imu && is_inited_) {
      if (!geo_observer_->observer_inited_) {
        // wait for calibration
        geo_observer_->initialize();
      } else if (geo_observer_->isObserverReady()) {
        // replace prior from observer
        geo_observer_->getCameraPrior(frame_last_, frame, T_prior);
      }
    }
    frame->setTwb(T_prior);

    timing::Timer timer_tracking("tracking");
    // for experiment, random fail

    // only use prior when all module ready, if no imu, just skip
    if (!tracking_->track(frame, this->options_.use_imu && is_inited_ && is_relocalized_ && this->geo_observer_->isObserverReady())) {
      is_relocalized_ = false;
      delta_pose_ = SE3();
      WARNLOG("tracking failed");
      if (!is_inited_) {
        // at the beginning, restart counter
        track_counter = 0;
        reloc_counter = 0;
      }
      
    } else {
      delta_pose_ = frame_last_->getTbw() * frame->getTwb();
      visualization_->publishTransform(frame, is_inited_);
      // for log
      track_counter ++;
      // for kitti
      if (!is_inited_ && track_counter > 20) {
      // for ustgz
      // if (!is_inited_ && track_counter > 80) {
        ERRLOG("visual loc inited!");
        is_inited_ = true;
      }
      // skip the beginning frames for evaluation
      if (is_inited_) {
        frames_.push_back(frame);
        // set global pose observasion
        geo_observer_->propagateCamera(frame);
        geo_observer_->updateState();
      } 

#ifdef RELOC_EXP
      {
        // for relocalization exp
        static xslam::VisualFrame::Ptr last_fail_inject = nullptr;
        static xslam::VisualFrame::Ptr last_fail_recover = nullptr;
        static std::vector<float> recover_distance;
        // for track fail injection experiment
        if (last_fail_inject == nullptr && last_fail_recover == nullptr) {
          last_fail_recover = frame;
        } else if (last_fail_inject != nullptr) {
          recover_distance.push_back(
              (last_fail_inject->getTbw() * frame->getTwb())
                  .translation()
                  .norm());
          WARNLOG(
              "Recover distance {} m  Recover in seconds {} s",
              recover_distance.back(),
              frame->timestamp_second() - last_fail_inject->timestamp_second());
          last_fail_recover = frame;
          last_fail_inject = nullptr;
        } else if (last_fail_inject == nullptr &&
                   (frame->timestamp_second() -
                        last_fail_recover->timestamp_second() >=
                    10)) {
          WARNLOG("inject lost");
          is_relocalized_ = false;
          last_fail_inject = frame;
          last_fail_recover = nullptr;
        }

        average_recover_distance =
            std::accumulate(
                recover_distance.begin(), recover_distance.end(), 0.0,
                [](double sum, double element) { return sum + element; }) /
            static_cast<double>(recover_distance.size());
        WARNLOG("Average recover distance: {} m", average_recover_distance);
      }
#endif
    }
    timer_tracking.Stop();
    

    
  }


  // if (is_inited_ && is_relocalized_) {
    // Geometric observer update, skip when not relocalized because cam pose may
    // not be stable
    // geo_observer_->updateState();
  // }

  visualization_->visualizeFrameInfo(frame);

  // frame->releaseImage();
  // keep one image for optical flow check
  if (frame_last_ != nullptr) {
    frame_last_->releaseImage();
    frame_last_->releaseDescriptors();
  }
  frame_last_ = frame;

  geo_observer_->updateHistory();

  auto timing_info = timing::Timing::Print();
  WARNLOG("{}", timing_info);

  std::stringstream counter_info;
  counter_info << "Counter_info \ntrack_cnt: " << track_counter
               << "\nreloc_cnt: " << reloc_counter << " \ntrack_ratio: "
               << track_counter * 100. / (track_counter + reloc_counter);
  counter_info << "\nTotal point_track_count: "
               << tracking_->get_total_track_points();
  counter_info << "\nTotal point_detect_count: "
               << tracking_->get_total_detect_points();
  WARNLOG("{}", counter_info.str());
}

void VisualLocalization::finish() {
  if (!FLAGS_trajectory_path.empty()) {
    saveTrajectory(FLAGS_trajectory_path);
  }
}

void VisualLocalization::saveTrajectory(const std::string& traj_file) {
  ofstream fs(traj_file);
  if (!fs.is_open()) {
    ERRLOG("{} open failed ", traj_file);
  }
  fs << std::fixed;
  // for (auto &&info : loc_info_) {
  for (auto&& frame : frames_) {
    const double timestamp = frame->timestamp_second();
    const SE3& Twc = frame->getTwb();

    const auto& rot = Twc.so3().unit_quaternion();
    const auto& trans = Twc.translation();

    fs << setprecision(6) << timestamp << " " << setprecision(9) << trans.x()
       << ' ' << trans.y() << ' ' << trans.z() << ' ' << rot.x() << ' '
       << rot.y() << ' ' << rot.z() << ' ' << rot.w() << endl;
  }
  fs.close();

  ofstream fs1(traj_file + "info");
  if (fs1.is_open()) {
    timing::Timing::Print(fs1);

    fs1 << std::setprecision(4)
        << "\nCounter_info \ntrack_cnt: " << track_counter
        << " \nreloc_cnt: " << reloc_counter << " \ntrack_ratio: "
        << track_counter * 100. / (track_counter + reloc_counter);
  }

  fs1 << "\nTotal point_track_count: " << tracking_->get_total_track_points();
  fs1 << "\nTotal point_detect_count: " << tracking_->get_total_detect_points();
# ifdef RELOC_EXP
  fs1 << "\nAverage relocalization distance: " << average_recover_distance;
#endif
  fs1.close();

  tracking_->dump_detect_cnt_history(traj_file + "det");
  tracking_->dump_track_cnt_history(traj_file + "trac");

  WARNLOG("trajectory saved to: {} with {} poses", traj_file, frames_.size());
}

VisualFrame::Ptr VisualLocalization::createFrameFromImageInput(
    const cv::Mat& image, bool b_extract_global, bool b_store_image) {
  //
  auto frame = make_shared<VisualFrame>();

  local_feature_extraction_->extractFeature(image, frame->keypoints_mutable(),
                                            frame->descriptors_mutable());
  frame->set_num_keypoints(frame->keypoints().size());
  frame->mappoints_mutable().resize(frame->num_keypoints(), nullptr);

  if (b_extract_global) {
    global_feature_extraction_->extractFeature(
        image, frame->global_descriptor_mutable());
  }

  if (b_store_image) {
    frame->set_image(image);
  }

  frame->set_camera(camera_);
  // TODO: hard code
  frame->assignKeyPointsToGrid(16, 16);

  return frame;
}



}  // namespace xslam