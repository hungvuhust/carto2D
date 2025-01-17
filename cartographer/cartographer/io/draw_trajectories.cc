

#include "cartographer/io/draw_trajectories.h"

#include "cartographer/io/image.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace io {

void DrawTrajectory(
    const mapping::proto::Trajectory &trajectory, const FloatColor &color,
    const PoseToPixelFunction &pose_to_pixel, cairo_surface_t *surface) {
  if (trajectory.node_size() == 0) {
    return;
  }
  constexpr double kTrajectoryWidth     = 4.;
  constexpr double kTrajectoryEndMarkers= 6.;
  constexpr double kAlpha               = 0.7;

  auto cr= ::cartographer::io::MakeUniqueCairoPtr(cairo_create(surface));

  cairo_set_source_rgba(cr.get(), color[0], color[1], color[2], kAlpha);
  cairo_set_line_width(cr.get(), kTrajectoryWidth);

  for (const auto &node: trajectory.node()) {
    const Eigen::Array2i pixel= pose_to_pixel(transform::ToRigid3(node.pose()));
    cairo_line_to(cr.get(), pixel.x(), pixel.y());
  }
  cairo_stroke(cr.get());

  // Draw beginning and end markers.
  {
    const Eigen::Array2i pixel=
        pose_to_pixel(transform::ToRigid3(trajectory.node(0).pose()));
    cairo_set_source_rgba(cr.get(), 0., 1., 0., kAlpha);
    cairo_arc(
        cr.get(), pixel.x(), pixel.y(), kTrajectoryEndMarkers, 0, 2 * M_PI);
    cairo_fill(cr.get());
  }
  {
    const Eigen::Array2i pixel= pose_to_pixel(transform::ToRigid3(
        trajectory.node(trajectory.node_size() - 1).pose()));
    cairo_set_source_rgba(cr.get(), 1., 0., 0., kAlpha);
    cairo_arc(
        cr.get(), pixel.x(), pixel.y(), kTrajectoryEndMarkers, 0, 2 * M_PI);
    cairo_fill(cr.get());
  }
  cairo_surface_flush(surface);
}

} // namespace io
} // namespace cartographer
