/*
 * @Author: David-Willo davidwillo@foxmail.com
 * @Date: 2023-11-23 04:42:46
 * @LastEditTime: 2023-11-23 05:05:26
 * @LastEditors: David-Willo
 * Jinhao HE (David Willo), IADC HKUST(GZ)
 * Copyright (c) 2023 by davidwillo@foxmail.com, All Rights Reserved. 
 */
#include "xslam/visual-types/tile.h"



namespace xslam {

Tile::Tile() {
    aslam::generateId(&id_);
}

}  // namespace xslam
