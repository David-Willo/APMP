/*** 
 * @Author: David-Willo davidwillo@foxmail.com
 * @Date: 2023-09-20 13:41:34
 * @LastEditTime: 2023-09-20 13:47:43
 * @LastEditors: David-Willo
 * @Jinhao HE (David Willo), IADC HKUST(GZ)
 * @Copyright (c) 2023 by davidwillo@foxmail.com, All Rights Reserved. 
 */
#ifndef DEBUG_VISUALIZE_H
#define DEBUG_VISUALIZE_H

#include <opencv2/opencv.hpp>

namespace xslam {
    namespace debug_vis {
        extern cv::Mat vis_lk;
        extern cv::Mat vis_track;
        extern cv::Mat vis_match;
        extern cv::Mat vis_reload;

    }
}
#endif