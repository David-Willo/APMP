#include "xslam/common/math.h"

namespace xslam {

namespace math {
Matrix3d skew(const Vector3d& v) {
  Matrix3d m;
  m.fill(0.);
  m(0, 1) = -v(2);
  m(0, 2) = v(1);
  m(1, 2) = -v(0);
  m(1, 0) = v(2);
  m(2, 0) = -v(1);
  m(2, 1) = v(0);
  return m;
}
}  // namespace math

}  // namespace xslam