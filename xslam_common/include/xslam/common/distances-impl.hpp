#include "./distances.h"

namespace xslam {

template <typename Scalar>
Distance::DistanceFunc Distance::createDistanceFunc(const DistanceType& type) {
  switch (type) {
    case DISTANCE_L2_NORM:
      //   return std::bind(
      //       &Distance::l2_norm, std::placeholders::_1,
      //       std::placeholders::_2);
      return Distance::l2_norm<Scalar>;
      break;
    default:
      throw std::runtime_error("distance not implemented");
      break;
  }
}

template <typename Scalar>
float Distance::l2_norm(const VectorXb& lhs, const VectorXb& rhs) {
  return (remap<Scalar>(lhs) - remap<Scalar>(rhs)).norm();
}

}  // namespace xslam
