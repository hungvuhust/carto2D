

#ifndef CARTOGRAPHER_MAPPING_2D_PROBABILITY_GRID_H_
#define CARTOGRAPHER_MAPPING_2D_PROBABILITY_GRID_H_

#include <vector>

#include "cartographer/common/port.h"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/mapping/2d/map_limits.h"
#include "cartographer/mapping/2d/xy_index.h"

namespace cartographer {
namespace mapping {

// Represents a 2D grid of probabilities.
class ProbabilityGrid : public Grid2D {
public:
  explicit ProbabilityGrid(
      const MapLimits &limits, ValueConversionTables *conversion_tables);
  explicit ProbabilityGrid(
      const proto::Grid2D &proto, ValueConversionTables *conversion_tables);

  // Sets the probability of the cell at 'cell_index' to the given
  // 'probability'. Only allowed if the cell was unknown before.
  void
      SetProbability(const Eigen::Array2i &cell_index, const float probability);

  // Applies the 'odds' specified when calling ComputeLookupTableToApplyOdds()
  // to the probability of the cell at 'cell_index' if the cell has not already
  // been updated. Multiple updates of the same cell will be ignored until
  // FinishUpdate() is called. Returns true if the cell was updated.
  //
  // If this is the first call to ApplyOdds() for the specified cell, its value
  // will be set to probability corresponding to 'odds'.
  bool ApplyLookupTable(
      const Eigen::Array2i &cell_index, const std::vector<uint16> &table);

  GridType GetGridType() const override;

  // Returns the probability of the cell with 'cell_index'.
  float GetProbability(const Eigen::Array2i &cell_index) const;

  proto::Grid2D           ToProto() const override;
  std::unique_ptr<Grid2D> ComputeCroppedGrid() const override;
  bool                    DrawToSubmapTexture(
                         proto::SubmapQuery::Response::SubmapTexture *const texture,
                         transform::Rigid3d local_pose) const override;

private:
  ValueConversionTables *conversion_tables_;
};

} // namespace mapping
} // namespace cartographer

#endif // CARTOGRAPHER_MAPPING_2D_PROBABILITY_GRID_H_
