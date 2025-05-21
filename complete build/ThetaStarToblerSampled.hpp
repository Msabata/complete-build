#ifndef THETA_STAR_TOBLER_SAMPLED_HPP
#define THETA_STAR_TOBLER_SAMPLED_HPP

#include "MapProcessingCommon.h" // For Grid_V3
#include "PathfindingUtils.hpp"  // For GridPoint
#include <vector>

namespace Pathfinding {

    // Declaration of the Theta* function
    std::vector<int> findThetaStarPath_Tobler_Sampled(
        const mapgeo::Grid_V3& logical_grid,
        const std::vector<float>& elevation_values,
        int elevation_width,
        int elevation_height,
        float log_cell_resolution,
        float elev_cell_resolution,
        float origin_offset_x,
        float origin_offset_y,
        const GridPoint& start,
        const GridPoint& end,
        int heuristic_type // Kept for API consistency, but primarily uses Euclidean internally
    );

} // namespace Pathfinding

#endif // THETA_STAR_TOBLER_SAMPLED_HPP