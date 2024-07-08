/*
 * @Author: David-Willo davidwillo@foxmail.com
 * @Date: 2023-11-22 12:36:31
 * @LastEditTime: 2024-07-02 07:09:46
 * @LastEditors: David-Willo
 * Jinhao HE (David Willo), IADC HKUST(GZ)
 * Copyright (c) 2023 by davidwillo@foxmail.com, All Rights Reserved. 
 */

#include "xslam/common/logging.h"
#include "xslam/visual-localization/map-io.h"
#include "xslam/visual-localization/visualization.h"

#include "xslam/visual-types/tile.h"
#include "xslam/visual-types/mappoint.h"

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>

#include <unordered_map>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <pcl_conversions/pcl_conversions.h>
#include <filters/filter_chain.hpp>



using namespace std;
using namespace std::chrono;

// DEFINE_string(local_feature_model, "", "model type.");

// DEFINE_string(eval_reconstruction_path, "/home/host/Documents/xslam_ws/src/xslam/dependencies/xslam_test_data/test_map_sample/", "reconstruction path");
// DEFINE_string(eval_global_feature_path, "/home/host/Documents/xslam_ws/src/xslam/dependencies/xslam_test_data/test_map_sample/global_features.h5", "global feature path");
// DEFINE_string(eval_local_feature_path, "/home/host/Documents/xslam_ws/src/xslam/dependencies/xslam_test_data/test_map_sample/features.h5", "local feature path");

// DEFINE_string(eval_reconstruction_path, "/media/host/OneTouch/ust_mini/exp_360_tyg/5/mapping_0712/outputs_T2R5/gui_export", "reconstruction path");
// DEFINE_string(eval_global_feature_path, "/media/host/OneTouch/ust_mini/exp_360_tyg/5/mapping_0712/outputs_T2R5/global-feats-netvlad.h5", "global feature path");
// DEFINE_string(eval_local_feature_path, "/media/host/OneTouch/ust_mini/exp_360_tyg/5/mapping_0712/outputs_T2R5/feats-superpoint-n4096-r1024-strict.h5", "local feature path");

// DEFINE_string(export_landmark_path, "/home/host/Documents/workspace/fif_ws/src/rpg_information_field/act_map_exp/exp_data/tyg_landmarks/tyg_points.txt", "export points path");
// DEFINE_string(export_view_dirs_path, "/home/host/Documents/workspace/fif_ws/src/rpg_information_field/act_map_exp/exp_data/tyg_landmarks/tyg_view_dirs.txt", "export view dirs path");

DEFINE_string(eval_reconstruction_path, "/media/host/OneTouch/ust_mini/exp_360_cainiao/6/mapping_0712/outputs_T2R5/gui_export", "reconstruction path");
DEFINE_string(eval_global_feature_path, "/media/host/OneTouch/ust_mini/exp_360_cainiao/6/mapping_0712/outputs_T2R5/global-feats-netvlad.h5", "global feature path");
DEFINE_string(eval_local_feature_path, "/media/host/OneTouch/ust_mini/exp_360_cainiao/6/mapping_0712/outputs_T2R5/feats-superpoint-n4096-r1024-strict.h5", "local feature path");

DEFINE_string(export_landmark_path, "/home/host/Documents/workspace/fif_ws/src/rpg_information_field/act_map_exp/exp_data/cainiao_landmarks/sp_points.txt", "export points path");
DEFINE_string(export_view_dirs_path, "/home/host/Documents/workspace/fif_ws/src/rpg_information_field/act_map_exp/exp_data/cainiao_landmarks/sp_view_dirs.txt", "export view dirs path");

using namespace xslam;

   

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    google::ParseCommandLineFlags(&argc, &argv, true);
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;
    auto map_io_options = MapIO::Options()
                                .reconstruction_path(FLAGS_eval_reconstruction_path)
                                .global_feature_path(FLAGS_eval_global_feature_path)
                                .local_feature_path(FLAGS_eval_local_feature_path);
    VisualMap::Ptr map = MapIO::loadVisualMap(map_io_options);



    INFOLOG("Test map content {}",map->getAllMapPoints().size());
    
    
    // std::unordered_map<xslam::TileGrid, Tile::Ptr, xslam::hash::TileGridHash> grid_map_;
    // for (const MapPoint::Ptr& mpt: map->getAllMapPoints()) {
    //     auto cell = xslam::getTileGrid(mpt->position()[0], mpt->position()[2], 5);
    //     auto it = grid_map_.find(cell);
    //     if (it == grid_map_.end()) {
    //         grid_map_[cell] = std::make_shared<Tile>();
    //     }
    //     grid_map_[cell]->mappoints_mutable().push_back(mpt);
    //     // INFOLOG("Cell {} {} content {}", cell.xIndex, cell.yIndex,  grid_map_[cell]->mappoints().size());
    // }

    ros::init(argc, argv, "eval_map");
    ros::NodeHandle nh;
    VisualizationVLoc visualization(nh, VisualizationVLoc::Options());

    visualization.setVisualMap(map);


    // test offline fif_gen
    std::ofstream f_out_points(FLAGS_export_landmark_path, std::ios::out | std::ios::app);
    std::ofstream f_out_view_dirs(FLAGS_export_view_dirs_path, std::ios::out | std::ios::app);

    if (!f_out_points.is_open() || !f_out_view_dirs.is_open()) {
        INFOLOG("export location {} {} not exit", FLAGS_export_landmark_path, FLAGS_export_view_dirs_path);
    } else {
        INFOLOG("export to fif format {} {}", FLAGS_export_landmark_path, FLAGS_export_view_dirs_path);
        for (const auto& mpt : map->getAllMapPoints()) {
            if (!mpt->is_valid()) {
                continue;
            }
            f_out_points << mpt->position()[0] << " " << mpt->position()[1] << " "<< mpt->position()[2] << "\n";
            f_out_view_dirs << mpt->view_dir()[0] << " " << mpt->view_dir()[1] << " "<< mpt->view_dir()[2] << "\n";
        } // end iterate mpts 

        f_out_points.close();
        f_out_view_dirs.close();
    }

    
    
    // //test gridmap
    // map->cacheMapPointsPCL();
    // map->loadLidarMap();
    // // Create a grid map
    // // elevation, then angle observation
    // grid_map::GridMap gdmap({"elevation", "0", "1", "2", "3", "4", "5", "is_floor"});
    // // gdmap["elevation"] = 10;
    // gdmap.setFrameId("map");

    // // 设置grid map的大小，这里x为1.2 m，y为2.0 m，每个格子的大小为3 cm
    // // 如果出现2.0/0.03不为整数的情况，那么grid map会自动填充一个格子，并且2.0会被赋为2.01，保证整除
    // // 这里可以设置地图的中心，默认情况下地图的中心和上一步设置的坐标系重合，见2.2部分
    // gdmap.setGeometry(grid_map::Length(400, 400), 2, grid_map::Position(608, -247));

    // for (const auto& mpt : map->getAllMapPoints()) {
    //     if (!mpt->is_valid()) {
    //         continue;
    //     }
    //     grid_map::Index index;
    //     // gdmap.getIndex(grid_map::Position(mpt->position()[0], mpt->position()[2]), index);
    //     gdmap.getIndex(grid_map::Position(mpt->position()[0], mpt->position()[1]), index);
    //     if(std::isnan(gdmap.at("elevation", index))) {
    //         gdmap.at("elevation", index) = mpt->position()[2];
    //     } else {
    //         gdmap.at("elevation", index) = std::max(float(mpt->position()[2]), gdmap.at("elevation", index));
    //     }
    //     grid_map::Position debugoos;
    //     gdmap.getPosition(index, debugoos);
    //     INFOLOG("recheck position {} {}", debugoos[0], debugoos[1]);
    //     INFOLOG("see {} {} elevation {}",mpt->position()[0], mpt->position()[1],  gdmap.at("elevation", index));

    // }
    // // filter the map

    // //! Filter chain.
    // filters::FilterChain<grid_map::GridMap> filterChain_("grid_map::GridMap");

    // //! Filter chain parameters name.
    // // std::string config_path = "/home/host/Documents/xslam_ws/src/xslam/xslam_visual_localization/nodes/filter_config.yaml";
    // // ros::param::loadFile(config_path);
    // std::string filterChainParametersName_;
    // nh.param("filter_chain_parameter_name", filterChainParametersName_, std::string("grid_map_filters"));
    // filterChain_.configure(filterChainParametersName_, nh);

    // // Apply filter chain.
    // // grid_map::GridMap gdfMap;
    // if (!filterChain_.update(gdmap, gdmap)) {
    //     ROS_ERROR("Could not update the grid map filter chain!");
    //     // return;
    // }

    // // then do yaw angle assignment
    // auto pclCloud = map->getPCLLidarCloud();
    // // double max_visible = 0;
    // // Iterate through each frame  in the map
    // // for (const VisualFrame::Ptr& frame : map->getAllFrames()) {
    // //     grid_map::Position position(frame->getTwb().translation()[0], frame->getTwb().translation()[1]);
    // //     // if (gdmap.atPosition("elevation_smooth", position) > 2 || std::isnan(gdmap.atPosition("elevation", position))) {
    // //     //     continue;
    // //     // }

    // //     // Define the radius for the search (adjust as needed)
    // //     double searchRadius = 50.0;

    // //     // Perform radius search in the original point cloud around the current grid cell
    // //     std::vector<int> pointIdx = map->searchByRadiusPCL(pcl::PointXYZ(position.x(), position.y(), 0), searchRadius);
    // //     if (pointIdx.size() > 0) {
    // //         // Initialize the histogram for yaw angles
    // //         std::vector<bool> yawOccupied(360, 0);
    // //         std::vector<int> yawHistogram(6, 0);

    // //         // Iterate through the neighbors found in the radius search
    // //         for (size_t i = 0; i < pointIdx.size(); ++i) {
    // //             //skip floor
    // //             if (gdmap.atPosition("elevation_smooth", grid_map::Position(pclCloud->points[pointIdx[i]].x, pclCloud->points[pointIdx[i]].y)) < 2 ) {
    // //                 continue;
    // //             }
    // //             // Calculate the yaw angle with respect to the camera
    // //             double x_diff = pclCloud->points[pointIdx[i]].x - position.x();
    // //             double y_diff = pclCloud->points[pointIdx[i]].y - position.y();
    // //             double z_diff = std::max(float(0), pclCloud->points[pointIdx[i]].z - gdmap.atPosition("elevation_smooth", position));

    // //             double dist = x_diff*x_diff + y_diff*y_diff;
    // //             // double height = pclCloud->points[pointIdx[i]].y
    // //             // if (dist < 25) {
    // //             //     continue;
    // //             // }
    // //             double yaw_angle = std::atan2(y_diff, x_diff) * (180.0 / M_PI);
    // //             int angle_index =  static_cast<int>(yaw_angle+180.0);
    // //             // Assign the yaw angle to the corresponding histogram bin
    // //             int binIndex =  static_cast<int>((yaw_angle + 180.0) / 60.0);
    // //             // if (binIndex >= 0 && binIndex < 6 && yawOccupied[angle_index] == false) {
    // //             if (binIndex >= 0 && binIndex < 6) {
    // //                 yawHistogram[binIndex]+= z_diff;
    // //                 // yawOccupied[angle_index] = true;
    // //                 if (yawHistogram[binIndex] > max_visible) {
    // //                     max_visible = yawHistogram[binIndex];
    // //                 }
    // //             }
    // //         }

    // //         // Set the histogram values to the corresponding layers in the grid map
    // //         for (int layerIndex = 0; layerIndex < 6; ++layerIndex) {
    // //             gdmap.atPosition(std::to_string(layerIndex), position) = yawHistogram[layerIndex] ;
    // //         }
    // //     }
    // // }

    // // Iterate through each grid cell in the map
    // visualization_msgs::MarkerArray markers;
    // for (grid_map::GridMapIterator it(gdmap); !it.isPastEnd(); ++it) {
    //     double max_visible = 0;
    //     // Get the position of the current grid cell
    //     grid_map::Position position;
    //     gdmap.getPosition(*it, position);
    //     gdmap.at("is_floor", *it) = (gdmap.at("elevation_smooth", *it) < 3);
    //     if (!gdmap.at("is_floor", *it)) {
    //     // if (gdmap.at("elevation", *it) > 2 || std::isnan(gdmap.at("elevation", *it))) {
    //        continue;
    //     }

    //     // Define the radius for the search (adjust as needed)
    //     double searchRadius = 50.0;

    //     // Perform radius search in the original point cloud around the current grid cell
    //     std::vector<int> pointIdx = map->searchByRadiusPCL(pcl::PointXYZ(position.x(), position.y(), 0), searchRadius);
    //     if (pointIdx.size() > 0) {
    //         // Initialize the histogram for yaw angles
    //         std::vector<size_t> yawOccupied(360, 0);
    //         std::vector<int> yawHistogram(6, 0);

    //         // Iterate through the neighbors found in the radius search
    //         for (size_t i = 0; i < pointIdx.size(); ++i) {
    //             // Calculate the yaw angle with respect to the camera
    //             double x_diff = pclCloud->points[pointIdx[i]].x - position.x();
    //             double y_diff = pclCloud->points[pointIdx[i]].y - position.y();
    //             double z_diff = std::max(float(0), pclCloud->points[pointIdx[i]].z - gdmap.at("elevation_smooth", *it));

    //             double dist = x_diff*x_diff + y_diff*y_diff;
    //             // double height = pclCloud->points[pointIdx[i]].y
    //             // too close, maybe not visible
    //             if (dist < 9) {
    //                 continue;
    //             }
    //             double yaw_angle = std::atan2(y_diff, x_diff) * (180.0 / M_PI);
    //             // int angle_index =  static_cast<int>(yaw_angle + (yaw_angle < 0 ? 180.0 : 0)) ;
    //             int angle_index =  static_cast<int>(yaw_angle + 180.0 ) ;
    //             // Assign the yaw angle to the corresponding histogram bin
    //             int binIndex =  static_cast<int>((yaw_angle + 180.0) / 60.0);
    //             // if (binIndex >= 0 && binIndex < 6 && yawOccupied[angle_index] == false) {
    //             if (binIndex >= 0 && binIndex < 6) {
    //                 // yawHistogram[binIndex]+= z_diff;
    //                 yawHistogram[binIndex]+= 1;
    //                 yawOccupied[angle_index]+=1;
    //                 // yawOccupied[angle_index] = true;
    //                 if (yawHistogram[binIndex] > max_visible) {
    //                     max_visible = yawHistogram[binIndex];
    //                 }
    //             }
    //         }
            
    //         // Set the histogram values to the corresponding layers in the grid map
    //         for (int layerIndex = 0; layerIndex < 6; ++layerIndex) {
    //             gdmap.at(std::to_string(layerIndex), *it) = yawHistogram[layerIndex]/max_visible;

    //             if (layerIndex > 0 && layerIndex % 2 != 0) {
    //                 continue;
    //             }
    //             visualization_msgs::Marker marker;
    //             marker.header.frame_id = "map";
    //             marker.header.stamp = ros::Time();
                
    //             marker.id = markers.markers.size();
    //             marker.type = visualization_msgs::Marker::ARROW;
    //             marker.action = visualization_msgs::Marker::ADD;
    //             marker.pose.position.x = position.x(); // your x coordinate
    //             marker.pose.position.y = position.y(); // your y coordinate
    //             marker.pose.position.z = 0;

    //             // Set orientation of the arrow to point in the direction of the angle
    //             tf::Quaternion q;
    //             q.setRPY(0, 0, (layerIndex*60+30+180) * M_PI / 180.0); // Convert angle from degrees to radians
    //             marker.pose.orientation.x = q.x();
    //             marker.pose.orientation.y = q.y();
    //             marker.pose.orientation.z = q.z();
    //             marker.pose.orientation.w = q.w();

    //             marker.scale.x = 1;
    //             marker.scale.y = 0.5;
    //             marker.scale.z = 0.02;
    //             marker.color.a = 1.0; // Don't forget to set the alpha!
    //             // You can set the color based on the histogram data
    //             marker.color.r = 1- yawHistogram[layerIndex]/max_visible;
    //             marker.color.g = yawHistogram[layerIndex]/max_visible;
    //             marker.color.b = 0.0;
    //             markers.markers.push_back(marker);
                
    //         }
    //     }
    // }


    // // scale the result
    // // for (int layerIndex = 0; layerIndex < 6; ++layerIndex) {
    // //     gdmap[std::to_string(layerIndex)] /= max_visible;
    // // }


    // INFOLOG("done convert grid");
    // // Publish the grid map
    // grid_map_msgs::GridMap grid_map_msg;
    // grid_map::GridMapRosConverter::toMessage(gdmap, grid_map_msg);
    // ros::Publisher grid_map_publisher = nh.advertise<grid_map_msgs::GridMap>("map_mul_layers", 1, true);
    // grid_map_publisher.publish(grid_map_msg);
    // ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10, true);
    // marker_pub.publish(markers);
    // INFOLOG("done pub markers of size {}", markers.markers.size());
    
    
    //test marker array
    // ros::Publisher grid_pub = nh.advertise<visualization_msgs::MarkerArray>("grid_color_array", 10);
    // visualization_msgs::MarkerArray gridColorArray;
    // size_t idx = 0;
    // for (auto& [cell, tile] : grid_map_) {
    //     // Example: Add a red cube at grid cell (1, 1)
    //     visualization_msgs::Marker marker;
    //     marker.header.frame_id = "map";
    //     marker.id = idx++;
    //     marker.type = visualization_msgs::Marker::CUBE;
    //     marker.pose.position.x = cell.xIndex*5;
    //     marker.pose.position.y = 0.0;
    //     marker.pose.position.z = cell.yIndex*5;
    //     marker.scale.x = 1.0;
    //     marker.scale.y = 1.0;
    //     marker.scale.z = 1.0;
    //     marker.color.g = (tile->mappoints().size())/200.;
    //     marker.color.r = 1.0-marker.color.g;
    //     marker.color.b = 0.0;
    //     marker.color.a = 1.0;  // Alpha channel (transparency)
    //     gridColorArray.markers.push_back(marker);
    //     INFOLOG("marker array has {} cells", gridColorArray.markers.size());
    // }

    // visualization.setGridColorArray(gridColorArray);

    // then do convert
    
    INFOLOG("all done");

    auto thread_viz_ = std::unique_ptr<thread>(
        new std::thread(&VisualizationVLoc::spin, &visualization));

    ros::spin();
    visualization.stop();
    thread_viz_->join();

    return 0;

}

