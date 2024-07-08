#pragma once

#include "xslam/common/common.h"
#include "xslam/common/eigen_types.h"

namespace xslam {

// TODO(hhy): do we really need to inherit from baseframe?
class InertialFrame {
 public:
  InertialFrame(/* args */);
  ~InertialFrame();

 protected:
  Vector3 ba_, bg_;

  SE3 Tbi_;
};

}  // namespace xslam
