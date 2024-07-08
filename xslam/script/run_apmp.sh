###
 # @Author: David-Willo davidwillo@foxmail.com
 # @Date: 2024-07-07 06:55:34
 # @LastEditTime: 2024-07-07 07:22:53
 # @LastEditors: David-Willo
 # Jinhao HE (David Willo), IADC HKUST(GZ)
 # Copyright (c) 2024 by davidwillo@foxmail.com, All Rights Reserved. 
### 
#! /bin/sh

script_dir=$(dirname $0)
# source $script_dir/utils.sh

export OMP_WAIT_POLICY=PASSIVE
export OMP_NUM_THREADS=1
#rosrun image_transport republish raw in:=/kitti/camera_color_right/image_rect compressed out:=/kitti/camera_color_right/image_rect
rosrun xslam_visual_localization visual_localization_node __name:=vloc_node __use_sim_time:=True \
	--sensor_calibration_file="" \
	--reconstruction_path /datapath/gui_export \
	--local_feature_path /datapath/feats-superpoint-n4096-r1024-strict.h5 \
	--global_feature_path /datapath/global-feats-netvlad.h5 \
	--global_feature_model netvlad \
	--local_feature_model superpoint \
	--local_feature_model_path $(rospack find xslam_test_data)/test_models/superpoint_x86_3080_1280x720_fp32.ts \
	--global_feature_model_path $(rospack find xslam_test_data)/test_models/netvlad_x86_3080_1280x720_fp32.ts \
	--image_topic /usb_cam/image_raw/compressed \
	--resize=true \
	--resize_width=1280 \
	--resize_height=720 #\
	# --use_fp16=true


wait