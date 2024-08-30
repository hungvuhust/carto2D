

#include "cartographer/mapping/internal/optimization/ceres_pose.h"

namespace cartographer {
namespace mapping {
namespace optimization {

CeresPose::Data FromPose(const transform::Rigid3d &pose) {
  return CeresPose::Data{
      {{pose.translation().x(), pose.translation().y(),
        pose.translation().z()}},
      {{pose.rotation().w(), pose.rotation().x(), pose.rotation().y(),
        pose.rotation().z()}}};
}

#if CERES_VERSION_MAJOR > 2 ||                                                 \
    CERES_VERSION_MAJOR == 2 && CERES_VERSION_MINOR >= 1
CeresPose::CeresPose(
    const transform::Rigid3d        &pose,
    std::unique_ptr<ceres::Manifold> translation_manifold,
    std::unique_ptr<ceres::Manifold> rotation_manifold, ceres::Problem *problem)
    : data_(std::make_shared<CeresPose::Data>(FromPose(pose))) {
  problem->AddParameterBlock(
      data_->translation.data(), 3, translation_manifold.release());
  problem->AddParameterBlock(
      data_->rotation.data(), 4, rotation_manifold.release());
}
#else
CeresPose::CeresPose(
    const transform::Rigid3d                     &pose,
    std::unique_ptr<ceres::LocalParameterization> translation_parametrization,
    std::unique_ptr<ceres::LocalParameterization> rotation_parametrization,
    ceres::Problem                               *problem)
    : data_(std::make_shared<CeresPose::Data>(FromPose(pose))) {
  problem->AddParameterBlock(
      data_->translation.data(), 3, translation_parametrization.release());
  problem->AddParameterBlock(
      data_->rotation.data(), 4, rotation_parametrization.release());
}
#endif

const transform::Rigid3d CeresPose::ToRigid() const {
  return transform::Rigid3d::FromArrays(data_->rotation, data_->translation);
}

} // namespace optimization
} // namespace mapping
} // namespace cartographer
