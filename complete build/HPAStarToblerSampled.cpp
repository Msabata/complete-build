#pragma once
#include "HPAStarToblerSampled.hpp"

#include <vector>
#include <queue>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_set>
#include <algorithm> // For std::reverse, std::min/max

// Assume PathfindingUtils provides dx[], dy[], NUM_DIRECTIONS, GridPoint, etc.
#include "PathfindingUtils.hpp"
// Assume MapProcessingCommon provides Grid_V3, GridFlags, numeric_traits, etc.
#include "MapProcessingCommon.h"


namespace Pathfinding {
    namespace HPA {

        using namespace mapgeo; // Use types from mapgeo namespace
        using namespace PathfindingUtils; // Use constants like dx, dy etc.


        // --- Static thread_local members initialization ---
        // Needs definition outside the class declaration
        thread_local std::vector<float> HierarchicalGraph::fine_g_scores_;
        thread_local std::vector<float> HierarchicalGraph::fine_f_scores_;
        thread_local std::vector<bool> HierarchicalGraph::fine_closed_;
        thread_local std::vector<int> HierarchicalGraph::fine_parents_;


        // --- Constructor ---
        HierarchicalGraph::HierarchicalGraph(int cluster_dimension) :
            cluster_dim_(std::max(1, cluster_dimension)) // Ensure cluster dim >= 1
        {}

        // --- Helper Functions ---
        inline bool HierarchicalGraph::isFineInBounds(int fx, int fy) const {
            return static_cast<unsigned>(fx) < static_cast<unsigned>(fine_width_) &&
                static_cast<unsigned>(fy) < static_cast<unsigned>(fine_height_);
        }
        inline bool HierarchicalGraph::isCoarseInBounds(int cx, int cy) const {
            return static_cast<unsigned>(cx) < static_cast<unsigned>(coarse_width_) &&
                static_cast<unsigned>(cy) < static_cast<unsigned>(coarse_height_);
        }
        inline GridPoint HierarchicalGraph::fineToCoarseCoords(int fx, int fy) const {
            return { fx / cluster_dim_, fy / cluster_dim_ };
        }
        inline int HierarchicalGraph::coarseIndex(int cx, int cy) const {
            // Assumes isCoarseInBounds check happened before if necessary
            return cy * coarse_width_ + cx;
        }
        inline int HierarchicalGraph::fineIndex(int fx, int fy) const {
            // Assumes isFineInBounds check happened before if necessary
            return fy * fine_width_ + fx;
        }
        inline void HierarchicalGraph::coarseCoords(int coarse_idx, int& cx, int& cy) const {
            cx = coarse_idx % coarse_width_;
            cy = coarse_idx / coarse_width_;
        }
        inline mapgeo::Point_float<float> HierarchicalGraph::getFineWorldCoords(int fx, int fy) const {
            return { (static_cast<float>(fx) + 0.5f) * fine_resolution_ + fine_origin_x_,
                    (static_cast<float>(fy) + 0.5f) * fine_resolution_ + fine_origin_y_ };
        }


        // --- Build Method (Preprocessing) ---
        bool HierarchicalGraph::build(
            const mapgeo::Grid_V3& logical_grid,
            const mapgeo::ElevationSampler& elevation_sampler,
            float log_cell_resolution,
            float origin_offset_x,
            float origin_offset_y)
        {
            if (!logical_grid.isValid() || cluster_dim_ <= 0 || log_cell_resolution <= EPSILON) {
                return false; // Invalid inputs
            }

            // Cache fine grid info
            fine_width_ = static_cast<int>(logical_grid.width());
            fine_height_ = static_cast<int>(logical_grid.height());
            fine_size_ = fine_width_ * fine_height_;
            fine_resolution_ = log_cell_resolution;
            fine_origin_x_ = origin_offset_x;
            fine_origin_y_ = origin_offset_y;

            // Calculate coarse grid dimensions
            coarse_width_ = (fine_width_ + cluster_dim_ - 1) / cluster_dim_;
            coarse_height_ = (fine_height_ + cluster_dim_ - 1) / cluster_dim_;
            coarse_size_ = coarse_width_ * coarse_height_;

            if (coarse_size_ == 0) return false;

            coarse_grid_data_.assign(coarse_size_, CoarseCell{}); // Initialize coarse grid
            coarse_adj_.assign(coarse_size_, std::vector<std::pair<int, float>>{}); // Initialize adj list


            // --- Step 1: Abstract Cells ---
            for (int cy = 0; cy < coarse_height_; ++cy) {
                for (int cx = 0; cx < coarse_width_; ++cx) {
                    int c_idx = coarseIndex(cx, cy);
                    CoarseCell& coarse_cell = coarse_grid_data_[c_idx];

                    int min_fx = cx * cluster_dim_;
                    int min_fy = cy * cluster_dim_;
                    int max_fx = std::min(min_fx + cluster_dim_, fine_width_);
                    int max_fy = std::min(min_fy + cluster_dim_, fine_height_);

                    double sum_elevation = 0.0;
                    double sum_terrain_cost = 0.0;
                    int traversable_count = 0;
                    bool coarse_impassable = false;
                    Point_float<double> world_coord_sum = { 0.0, 0.0 }; // Use double for sum

                    for (int fy = min_fy; fy < max_fy; ++fy) {
                        for (int fx = min_fx; fx < max_fx; ++fx) {
                            const auto& fine_cell = logical_grid.at(fx, fy);
                            if (fine_cell.hasFlag(GridFlags::FLAG_IMPASSABLE) || fine_cell.value <= numeric_traits<float>::epsilon) {
                                coarse_impassable = true;
                                goto next_coarse_cell; // Exit both loops for this coarse cell
                            }

                            // Fine cell is traversable
                            Point_float<float> fine_world = getFineWorldCoords(fx, fy);
                            sum_elevation += elevation_sampler.getElevationAt(fine_world.x, fine_world.y);
                            sum_terrain_cost += fine_cell.value;
                            world_coord_sum.x += fine_world.x;
                            world_coord_sum.y += fine_world.y;
                            traversable_count++;
                        }
                    }

                next_coarse_cell: // Label for goto
                    coarse_cell.is_traversable = !coarse_impassable && (traversable_count > 0);
                    if (coarse_cell.is_traversable) {
                        coarse_cell.average_elevation = static_cast<float>(sum_elevation / traversable_count);
                        coarse_cell.average_terrain_cost = static_cast<float>(sum_terrain_cost / traversable_count);
                        coarse_cell.world_center_coords.x = static_cast<float>(world_coord_sum.x / traversable_count);
                        coarse_cell.world_center_coords.y = static_cast<float>(world_coord_sum.y / traversable_count);
                    }
                } // end for cx
            } // end for cy


            // --- Step 2: Build Coarse Graph Edges ---
            for (int c1y = 0; c1y < coarse_height_; ++c1y) {
                for (int c1x = 0; c1x < coarse_width_; ++c1x) {
                    int c1_idx = coarseIndex(c1x, c1y);
                    const CoarseCell& cell1 = coarse_grid_data_[c1_idx];
                    if (!cell1.is_traversable) continue;

                    for (int dir = 0; dir < NUM_DIRECTIONS; ++dir) {
                        int c2x = c1x + dx[dir];
                        int c2y = c1y + dy[dir];

                        if (!isCoarseInBounds(c2x, c2y)) continue;

                        int c2_idx = coarseIndex(c2x, c2y);
                        const CoarseCell& cell2 = coarse_grid_data_[c2_idx];
                        if (!cell2.is_traversable) continue;

                        // Calculate approximate edge cost
                        float dist = worldDistance(cell1.world_center_coords, cell2.world_center_coords);
                        float cost = 0.0f;

                        if (dist > numeric_traits<float>::epsilon) {
                            float delta_h = cell2.average_elevation - cell1.average_elevation;
                            // Use average terrain cost of the *destination* for consistency? Or average of both? Let's average both.
                            float avg_terrain = (cell1.average_terrain_cost + cell2.average_terrain_cost) * 0.5f;
                            // Ensure avg_terrain is reasonable
                            avg_terrain = std::max(avg_terrain, numeric_traits<float>::epsilon * 10.0f); // Avoid issues if avg cost is near zero


                            float S = delta_h / dist;
                            float SlopeFactor = expf(-3.5f * fabsf(S + 0.05f));
                            float time_penalty = (SlopeFactor > numeric_traits<float>::epsilon) ? (1.0f / SlopeFactor) : infinite_penalty_;

                            cost = dist * time_penalty * avg_terrain;
                        }

                        if (cost < infinite_penalty_) {
                            // Add edge in both directions? Assumes symmetry in coarse cost calc.
                            coarse_adj_[c1_idx].push_back({ c2_idx, cost });
                        }
                    } // end direction loop
                } // end for c1x
            } // end for c1y

            return true; // Build successful
        }


        // --- Run A* on Coarse Graph ---
        std::vector<int> HierarchicalGraph::runCoarseAStar(int start_coarse_idx, int end_coarse_idx) {
            std::vector<int> path_indices;
            if (start_coarse_idx == end_coarse_idx && coarse_grid_data_[start_coarse_idx].is_traversable) {
                path_indices.push_back(start_coarse_idx);
                return path_indices;
            }

            std::vector<float> g_scores(coarse_size_, infinite_penalty_);
            std::vector<float> f_scores(coarse_size_, infinite_penalty_);
            std::vector<int> parents(coarse_size_, -1);
            std::vector<bool> closed(coarse_size_, false);

            // Priority Queue for coarse A*
            auto cmp = [&](int l, int r) {
                if (std::fabs(f_scores[l] - f_scores[r]) > numeric_traits<float>::epsilon) return f_scores[l] > f_scores[r];
                return g_scores[l] > g_scores[r]; // Tie-break with g-score
                };
            std::priority_queue<int, std::vector<int>, decltype(cmp)> openQueue(cmp);

            g_scores[start_coarse_idx] = 0.0f;
            const auto& start_cell = coarse_grid_data_[start_coarse_idx];
            const auto& end_cell = coarse_grid_data_[end_coarse_idx];
            float h_start = worldDistance(start_cell.world_center_coords, end_cell.world_center_coords) * min_heuristic_cost_factor_;
            f_scores[start_coarse_idx] = h_start;
            openQueue.push(start_coarse_idx);

            while (!openQueue.empty()) {
                int current_idx = openQueue.top();
                openQueue.pop();

                if (closed[current_idx]) continue;
                closed[current_idx] = true;

                if (current_idx == end_coarse_idx) break; // Goal reached

                const float current_g = g_scores[current_idx];
                const auto& current_cell = coarse_grid_data_[current_idx];

                // Explore neighbors using precomputed coarse graph
                for (const auto& edge : coarse_adj_[current_idx]) {
                    int neighbor_idx = edge.first;
                    float edge_cost = edge.second;

                    if (closed[neighbor_idx]) continue;

                    float tentative_g = current_g + edge_cost;

                    if (tentative_g < g_scores[neighbor_idx]) {
                        parents[neighbor_idx] = current_idx;
                        g_scores[neighbor_idx] = tentative_g;
                        const auto& neighbor_cell = coarse_grid_data_[neighbor_idx];
                        float h = worldDistance(neighbor_cell.world_center_coords, end_cell.world_center_coords) * min_heuristic_cost_factor_;
                        f_scores[neighbor_idx] = tentative_g + h;
                        openQueue.push(neighbor_idx);
                    }
                }
            }

            // Reconstruct coarse path
            if (parents[end_coarse_idx] != -1 || start_coarse_idx == end_coarse_idx) {
                int current = end_coarse_idx;
                while (current != -1) {
                    path_indices.push_back(current);
                    current = parents[current];
                }
                std::reverse(path_indices.begin(), path_indices.end());
            }
            return path_indices;
        }


        // --- Find Path (Query Method) ---
        std::vector<int> HierarchicalGraph::findPath(
            const GridPoint& start_fine,
            const GridPoint& end_fine,
            const mapgeo::Grid_V3& logical_grid,
            const mapgeo::ElevationSampler& elevation_sampler)
        {
            std::vector<int> resultPath; // Final fine path

            if (coarse_size_ == 0 || !isFineInBounds(start_fine.x, start_fine.y) || !isFineInBounds(end_fine.x, end_fine.y)) {
                return resultPath; // Not built or invalid start/end
            }

            // --- Step 1: Map to Coarse & Check Traversability ---
            GridPoint start_coarse_gp = fineToCoarseCoords(start_fine.x, start_fine.y);
            GridPoint end_coarse_gp = fineToCoarseCoords(end_fine.x, end_fine.y);

            if (!isCoarseInBounds(start_coarse_gp.x, start_coarse_gp.y) || !isCoarseInBounds(end_coarse_gp.x, end_coarse_gp.y)) {
                return resultPath; // Start/end map outside coarse bounds
            }

            int start_coarse_idx = coarseIndex(start_coarse_gp.x, start_coarse_gp.y);
            int end_coarse_idx = coarseIndex(end_coarse_gp.x, end_coarse_gp.y);

            if (!coarse_grid_data_[start_coarse_idx].is_traversable || !coarse_grid_data_[end_coarse_idx].is_traversable) {
                // Try finding nearest traversable coarse cells? For now, fail.
                return resultPath; // Start or end coarse cell is impassable
            }

            // --- Step 2: High-Level Search ---
            std::vector<int> coarse_path_indices = runCoarseAStar(start_coarse_idx, end_coarse_idx);

            if (coarse_path_indices.empty()) {
                return resultPath; // No path found on coarse graph
            }

            // --- Step 3: Low-Level Constrained Search ---
            std::unordered_set<int> coarse_corridor(coarse_path_indices.begin(), coarse_path_indices.end());

            resultPath = runConstrainedFineSearch(
                start_fine, end_fine, coarse_corridor, logical_grid, elevation_sampler);


            return resultPath;
        }

        // --- Fine Search Helper: Calculate Fine Segment Cost (Adapated from Lazy Theta*) ---
        float HierarchicalGraph::calculateFineSegmentCost(
            int x_start, int y_start, int x_end, int y_end,
            const mapgeo::Grid_V3& grid,
            const mapgeo::ElevationSampler& sampler)
        {
            // This uses the *fine* grid helpers and fine resolution stored in the class
            float total_cost = 0.0f;
            // Use a generic getLineSegmentCells - assumes it exists standalone or copied here
            std::vector<GridPoint> segment_cells = PathfindingUtils::getLineSegmentCells(x_start, y_start, x_end, y_end);
            if (segment_cells.size() < 2) { return 0.0f; }

            Point_float<float> prev_world = getFineWorldCoords(segment_cells[0].x, segment_cells[0].y);
            float prev_elev = sampler.getElevationAt(prev_world.x, prev_world.y);

            for (size_t i = 1; i < segment_cells.size(); ++i) {
                const GridPoint& current_gp = segment_cells[i];
                // Use fine grid check
                if (!isFineInBounds(current_gp.x, current_gp.y)) return infinite_penalty_;
                const auto& current_cell_data = grid.at(current_gp.x, current_gp.y);
                // Fine grid obstacle check
                if (current_cell_data.hasFlag(GridFlags::FLAG_IMPASSABLE) || current_cell_data.value <= numeric_traits<float>::epsilon) {
                    return infinite_penalty_;
                }
                Point_float<float> curr_world = getFineWorldCoords(current_gp.x, current_gp.y);
                float current_elev = sampler.getElevationAt(curr_world.x, curr_world.y);
                float delta_h = current_elev - prev_elev;
                float delta_dist_world = worldDistance(prev_world, curr_world);

                if (delta_dist_world > numeric_traits<float>::epsilon) {
                    float S = delta_h / delta_dist_world;
                    float SlopeFactor = expf(-3.5f * fabsf(S + 0.05f));
                    float time_penalty = (SlopeFactor > numeric_traits<float>::epsilon) ? (1.0f / SlopeFactor) : infinite_penalty_;
                    if (time_penalty >= infinite_penalty_) return infinite_penalty_;
                    float terrain_cost = current_cell_data.value;
                    total_cost += delta_dist_world * time_penalty * terrain_cost;
                }
                prev_world = curr_world;
                prev_elev = current_elev;
            }
            return total_cost;
        }

        // --- Fine Search Helper: Line of Sight (Adapated from Lazy Theta*) ---
        bool HierarchicalGraph::hasFineLineOfSight(
            int x0, int y0, int x1, int y1,
            const mapgeo::Grid_V3& grid)
        {
            // Uses fine grid bounds and checks
            if (!isFineInBounds(x0, y0) || !isFineInBounds(x1, y1)) return false;
            int dx = std::abs(x1 - x0); int dy = -std::abs(y1 - y0);
            int sx = (x0 < x1) ? 1 : -1; int sy = (y0 < y1) ? 1 : -1;
            int err = dx + dy; int cx = x0; int cy = y0;
            while (true) {
                if (cx != x0 || cy != y0) {
                    if (!isFineInBounds(cx, cy)) return false;
                    const auto& cell = grid.at(cx, cy);
                    if (cell.hasFlag(GridFlags::FLAG_IMPASSABLE) || cell.value <= numeric_traits<float>::epsilon) {
                        return false;
                    }
                }
                if (cx == x1 && cy == y1) break;
                int e2 = 2 * err;
                if (e2 >= dy) { if (cx == x1) break; err += dy; cx += sx; }
                if (e2 <= dx) { if (cy == y1) break; err += dx; cy += sy; }
            }
            return true;
        }

        // --- Fine Search Helper: Heuristic (Adapated from Lazy Theta*) ---
        float HierarchicalGraph::calculateFineHeuristic(int x1, int y1, int x2, int y2) {
            float dx = static_cast<float>(x2 - x1);
            float dy = static_cast<float>(y2 - y1);
            // Use fine resolution stored in class
            float dist = std::sqrt(dx * dx + dy * dy) * fine_resolution_;
            // Use min cost factor stored in class
            return dist * min_heuristic_cost_factor_;
        }


        // --- Run Constrained Fine Search (Lazy Theta* adapted) ---
        std::vector<int> HierarchicalGraph::runConstrainedFineSearch(
            const GridPoint& start_fine,
            const GridPoint& end_fine,
            const std::unordered_set<int>& coarse_corridor,
            const mapgeo::Grid_V3& logical_grid, // Pass fine grid directly
            const mapgeo::ElevationSampler& elevation_sampler) // Pass sampler directly
        {
            std::vector<int> resultPath;
            const int startIdx = fineIndex(start_fine.x, start_fine.y);
            const int endIdx = fineIndex(end_fine.x, end_fine.y);

            // Check start/end validity again on fine grid (redundant but safe)
            const GridCellData& startCell = logical_grid.at(start_fine.x, start_fine.y);
            if (startCell.value <= numeric_traits<float>::epsilon || startCell.hasFlag(GridFlags::FLAG_IMPASSABLE)) return resultPath;
            const GridCellData& endCell = logical_grid.at(end_fine.x, end_fine.y);
            if (endCell.value <= numeric_traits<float>::epsilon || endCell.hasFlag(GridFlags::FLAG_IMPASSABLE)) return resultPath;
            if (startIdx == endIdx) { resultPath.push_back(startIdx); return resultPath; }


            // --- Setup Fine Search Data Structures ---
            // Use the static thread_local members defined earlier
            try {
                fine_g_scores_.assign(fine_size_, infinite_penalty_);
                fine_f_scores_.assign(fine_size_, infinite_penalty_);
                fine_closed_.assign(fine_size_, false);
                fine_parents_.assign(fine_size_, -1);
            }
            catch (const std::bad_alloc&) { return resultPath; /* Memory allocation failed */ }

            // --- Priority Queue (Local to this function) ---
            auto cmp = [&](int l, int r) {
                if (std::fabs(fine_f_scores_[l] - fine_f_scores_[r]) > numeric_traits<float>::epsilon) return fine_f_scores_[l] > fine_f_scores_[r];
                return fine_g_scores_[l] > fine_g_scores_[r];
                };
            std::priority_queue<int, std::vector<int>, decltype(cmp)> openQueue(cmp);

            // --- Initialization ---
            fine_g_scores_[startIdx] = 0.0f;
            fine_f_scores_[startIdx] = calculateFineHeuristic(start_fine.x, start_fine.y, end_fine.x, end_fine.y);
            openQueue.push(startIdx);

            // --- Main Lazy Theta* Loop (with corridor constraint) ---
            while (!openQueue.empty()) {
                const int currentIdx = openQueue.top();
                openQueue.pop();

                if (fine_closed_[currentIdx]) continue;

                // --- Lazy Update Step ---
                int parentIdx = fine_parents_[currentIdx];
                if (parentIdx != -1) {
                    int grandParentIdx = fine_parents_[parentIdx];
                    if (grandParentIdx != -1) {
                        int x_curr, y_curr, x_gp, y_gp;
                        // Need fine toCoords
                        toCoords(currentIdx, fine_width_, x_curr, y_curr);
                        toCoords(grandParentIdx, fine_width_, x_gp, y_gp);

                        if (hasFineLineOfSight(x_gp, y_gp, x_curr, y_curr, logical_grid)) {
                            float segment_cost = calculateFineSegmentCost(x_gp, y_gp, x_curr, y_curr, logical_grid, elevation_sampler);
                            if (segment_cost < infinite_penalty_) {
                                float g_via_grandparent = fine_g_scores_[grandParentIdx] + segment_cost;
                                if (g_via_grandparent < fine_g_scores_[currentIdx]) {
                                    fine_g_scores_[currentIdx] = g_via_grandparent;
                                    fine_parents_[currentIdx] = grandParentIdx;
                                    fine_f_scores_[currentIdx] = fine_g_scores_[currentIdx] + calculateFineHeuristic(x_curr, y_curr, end_fine.x, end_fine.y);
                                }
                            }
                        }
                    }
                }
                // --- End Lazy Update ---

                fine_closed_[currentIdx] = true;
                if (currentIdx == endIdx) break; // Goal

                int x, y;
                toCoords(currentIdx, fine_width_, x, y);
                const float current_g = fine_g_scores_[currentIdx];

                // --- Explore Neighbors ---
                for (int dir = 0; dir < NUM_DIRECTIONS; ++dir) {
                    const int nx = x + dx[dir];
                    const int ny = y + dy[dir];

                    if (!isFineInBounds(nx, ny)) continue; // Fine bounds check

                    // <<< --- HPA* Constraint Check --- >>>
                    GridPoint neighbor_coarse_gp = fineToCoarseCoords(nx, ny);
                    // Check coarse bounds just in case fine mapping gives invalid coarse coords near edge
                    if (!isCoarseInBounds(neighbor_coarse_gp.x, neighbor_coarse_gp.y)) continue;
                    int neighbor_coarse_idx = coarseIndex(neighbor_coarse_gp.x, neighbor_coarse_gp.y);
                    if (coarse_corridor.find(neighbor_coarse_idx) == coarse_corridor.end()) {
                        continue; // Prune: Neighbor not in the high-level path corridor
                    }
                    // <<< ----------------------------- >>>


                    const int neighborIdx = fineIndex(nx, ny);
                    if (fine_closed_[neighborIdx]) continue;

                    const auto& neighborCell = logical_grid.at(nx, ny);
                    if (neighborCell.value <= numeric_traits<float>::epsilon || neighborCell.hasFlag(GridFlags::FLAG_IMPASSABLE)) {
                        continue;
                    }

                    float step_cost = calculateFineSegmentCost(x, y, nx, ny, logical_grid, elevation_sampler);
                    if (step_cost >= infinite_penalty_) continue;

                    float tentative_g = current_g + step_cost;
                    if (tentative_g < fine_g_scores_[neighborIdx]) {
                        fine_parents_[neighborIdx] = currentIdx;
                        fine_g_scores_[neighborIdx] = tentative_g;
                        fine_f_scores_[neighborIdx] = tentative_g + calculateFineHeuristic(nx, ny, end_fine.x, end_fine.y);
                        openQueue.push(neighborIdx);
                    }
                } // End neighbor loop
            } // End while

            // --- Path Reconstruction (Fine Path - same as Lazy Theta*) ---
            resultPath.clear();
            if (fine_parents_[endIdx] == -1 && startIdx != endIdx) { return resultPath; }

            std::vector<int> waypoints_reversed;
            int current_wp = endIdx;
            size_t safety_count = 0;
            const size_t max_path_len = static_cast<size_t>(fine_size_) * 2 + 2;
            while (current_wp != -1 && safety_count < max_path_len) {
                waypoints_reversed.push_back(current_wp);
                if (current_wp == startIdx) break;
                current_wp = fine_parents_[current_wp];
                safety_count++;
            }
            if (current_wp != startIdx && startIdx != endIdx) return std::vector<int>();
            if (safety_count >= max_path_len) return std::vector<int>();
            std::reverse(waypoints_reversed.begin(), waypoints_reversed.end());

            if (!waypoints_reversed.empty()) {
                resultPath.push_back(waypoints_reversed[0]);
                for (size_t i = 0; i < waypoints_reversed.size() - 1; ++i) {
                    int p1_idx = waypoints_reversed[i]; int p2_idx = waypoints_reversed[i + 1];
                    int x1, y1, x2, y2;
                    toCoords(p1_idx, fine_width_, x1, y1); toCoords(p2_idx, fine_width_, x2, y2);
                    // Use standalone getLineSegmentCells
                    std::vector<GridPoint> segment_cells = PathfindingUtils::getLineSegmentCells(x1, y1, x2, y2);
                    for (size_t j = 1; j < segment_cells.size(); ++j) {
                        int cell_idx = fineIndex(segment_cells[j].x, segment_cells[j].y);
                        if (resultPath.empty() || cell_idx != resultPath.back()) {
                            resultPath.push_back(cell_idx);
                        }
                    }
                }
            }
            else if (startIdx == endIdx) { resultPath.push_back(startIdx); }

            return resultPath;
        }


    } // namespace HPA
} // namespace Pathfinding