/*** 
 * @Author: David-Willo davidwillo@foxmail.com
 * @Date: 2023-04-13 08:32:50
 * @LastEditTime: 2024-07-02 07:22:14
 * @LastEditors: David-Willo
 * @Jinhao HE (David Willo), IADC HKUST(GZ)
 * @Copyright (c) 2024 by davidwillo@foxmail.com, All Rights Reserved. 
 */

#pragma once

#include "visual-frame.h"

namespace xslam {

class KeyFrame : protected VisualFrame {
 public:
  KeyFrame(/* args */);
  ~KeyFrame();

 private:
  /* data */
};

}  // namespace xslam
