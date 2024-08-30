

#ifndef CARTOGRAPHER_IO_DRAW_TRAJECTORIES_H_
#define CARTOGRAPHER_IO_DRAW_TRAJECTORIES_H_

#include "cairo/cairo.h"
#include "cartographer/io/color.h"
#include "cartographer/mapping/proto/trajectory.pb.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace io {

using PoseToPixelFunction=
    std::function<Eigen::Array2i(const transform::Rigid3d &pose)>;

// Draws the 'trajectory' with the given 'color' onto 'surface'. The
// 'pose_to_pixel' function must translate a trajectory node's position into the
// pixel on 'surface'.
void DrawTrajectory(
    const mapping::proto::Trajectory &trajectory, const FloatColor &color,
    const PoseToPixelFunction &pose_to_pixel, cairo_surface_t *surface);

} // namespace io
} // namespace cartographer

#endif // CARTOGRAPHER_IO_DRAW_TRAJECTORIES_H_
