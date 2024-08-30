

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_CERES_POSE_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_CERES_POSE_H_

#include <array>
#include <memory>

#include "Eigen/Core"
#include "cartographer/transform/rigid_transform.h"
#include "ceres/ceres.h"
#if CERES_VERSION_MAJOR > 2 ||                                                 \
    CERES_VERSION_MAJOR == 2 && CERES_VERSION_MINOR >= 1
  #include "ceres/manifold.h"
#endif

namespace cartographer {
namespace mapping {
namespace optimization {

class CeresPose {
public:
#if CERES_VERSION_MAJOR > 2 ||                                                 \
    CERES_VERSION_MAJOR == 2 && CERES_VERSION_MINOR >= 1
  CeresPose(
      const transform::Rigid3d        &pose,
      std::unique_ptr<ceres::Manifold> translation_manifold,
      std::unique_ptr<ceres::Manifold> rotation_manifold,
      ceres::Problem                  *problem);
#else
  CeresPose(
      const transform::Rigid3d                     &rigid,
      std::unique_ptr<ceres::LocalParameterization> translation_parametrization,
      std::unique_ptr<ceres::LocalParameterization> rotation_parametrization,
      ceres::Problem                               *problem);
#endif

  const transform::Rigid3d ToRigid() const;

  double       *translation() { return data_->translation.data(); }
  const double *translation() const { return data_->translation.data(); }

  double       *rotation() { return data_->rotation.data(); }
  const double *rotation() const { return data_->rotation.data(); }

  struct Data {
    std::array<double, 3> translation;
    // Rotation quaternion as (w, x, y, z).
    std::array<double, 4> rotation;
  };

  Data &data() { return *data_; }

private:
  std::shared_ptr<Data> data_;
};

CeresPose::Data FromPose(const transform::Rigid3d &pose);

} // namespace optimization
} // namespace mapping
} // namespace cartographer

#endif // CARTOGRAPHER_MAPPING_INTERNAL_OPTIMIZATION_CERES_POSE_H_
