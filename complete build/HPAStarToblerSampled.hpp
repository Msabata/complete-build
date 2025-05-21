#ifndef HPA_STAR_TOBLER_SAMPLED_HPP
#define HPA_STAR_TOBLER_SAMPLED_HPP

#include "MapProcessingCommon.h"
#include "PathfindingUtils.hpp"
#include "ElevationSampler.hpp" // Needed for build and query
#include <vector>
#include <string>
#include <unordered_set> // For corridor lookup

namespace Pathfinding {
    namespace HPA { // Nested namespace for HPA* related code

        // Structure to hold coarse cell information
        struct CoarseCell {
            bool is_traversable = false;
            float average_elevation = 0.0f;
            float average_terrain_cost = 1.0f; // Default to 1 if no traversable cells
            mapgeo::Point_float<float> world_center_coords = { 0.0f, 0.0f };
            // Add other precomputed data if needed
        };

        // Class to manage the HPA* process
        class HierarchicalGraph {
        public:
            // Constructor allows setting cluster dimension
            explicit HierarchicalGraph(int cluster_dimension = 10);

            // Build the coarse graph (preprocessing step)
            // Returns true on success, false on failure (e.g., invalid inputs)
            bool build(
                const mapgeo::Grid_V3& logical_grid,
                const mapgeo::ElevationSampler& elevation_sampler, // Pass by const ref
                float log_cell_resolution,
                float origin_offset_x,
                float origin_offset_y);

            // Find a path using the HPA* strategy (query step)
            std::vector<int> findPath(
                const GridPoint& start_fine,
                const GridPoint& end_fine,
                const mapgeo::Grid_V3& logical_grid,           // Fine grid needed again
                const mapgeo::ElevationSampler& elevation_sampler // Needed again
                // heuristic_type is implicitly handled by fine/coarse search methods
            );

            // Optional: Allow getting coarse graph info for debugging/visualization
            const std::vector<CoarseCell>& getCoarseGrid() const { return coarse_grid_data_; }
            int getCoarseWidth() const { return coarse_width_; }
            int getCoarseHeight() const { return coarse_height_; }
            int getClusterDim() const { return cluster_dim_; }


        private:
            // --- Configuration ---
            int cluster_dim_;
            float min_heuristic_cost_factor_ = 0.5f; // For coarse A* heuristic scaling
            float infinite_penalty_ = std::numeric_limits<float>::max();

            // --- Fine Grid Info (cached during build) ---
            int fine_width_ = 0;
            int fine_height_ = 0;
            int fine_size_ = 0;
            float fine_resolution_ = 0.0f;
            float fine_origin_x_ = 0.0f;
            float fine_origin_y_ = 0.0f;

            // --- Coarse Grid Data ---
            std::vector<CoarseCell> coarse_grid_data_;
            int coarse_width_ = 0;
            int coarse_height_ = 0;
            int coarse_size_ = 0;

            // --- Coarse Graph Structure ---
            // Adjacency list: coarse_adj_[from_coarse_idx] = { {to_coarse_idx1, cost1}, {to_coarse_idx2, cost2}, ... }
            std::vector<std::vector<std::pair<int, float>>> coarse_adj_;

            // --- Helper Functions ---
            inline GridPoint fineToCoarseCoords(int fx, int fy) const;
            inline int coarseIndex(int cx, int cy) const;
            inline int fineIndex(int fx, int fy) const; // Just uses fine_width_
            inline void coarseCoords(int coarse_idx, int& cx, int& cy) const;
            inline bool isFineInBounds(int fx, int fy) const;
            inline bool isCoarseInBounds(int cx, int cy) const;
            inline mapgeo::Point_float<float> getFineWorldCoords(int fx, int fy) const;


            // --- Internal Search Functions ---
            // Runs A* on the precomputed coarse graph
            std::vector<int> runCoarseAStar(int start_coarse_idx, int end_coarse_idx);

            // Runs Lazy Theta* on the fine grid, constrained by the coarse corridor
            std::vector<int> runConstrainedFineSearch(
                const GridPoint& start_fine,
                const GridPoint& end_fine,
                const std::unordered_set<int>& coarse_corridor, // Set of allowed coarse cell indices
                const mapgeo::Grid_V3& logical_grid,
                const mapgeo::ElevationSampler& elevation_sampler);

            // --- Helper for constrained search (based on Lazy Theta*) ---
            // Copied/adapted helpers from previous Lazy Theta* implementation
            // Need access to fine grid info and sampler passed into runConstrainedFineSearch
            float calculateFineSegmentCost(
                int x_start, int y_start, int x_end, int y_end,
                const mapgeo::Grid_V3& grid,
                const mapgeo::ElevationSampler& sampler);

            bool hasFineLineOfSight(
                int x0, int y0, int x1, int y1,
                const mapgeo::Grid_V3& grid);

            float calculateFineHeuristic(int x1, int y1, int x2, int y2);


            // --- Data structures for the *fine* search (constrained) ---
            // Made static thread_local members for reuse across queries in same thread
            static thread_local std::vector<float> fine_g_scores_;
            static thread_local std::vector<float> fine_f_scores_;
            static thread_local std::vector<bool> fine_closed_;
            static thread_local std::vector<int> fine_parents_;
            // Note: Priority queue is typically local to the search function
        };

    } // namespace HPA
} // namespace Pathfinding

#endif // HPA_STAR_TOBLER_SAMPLED_HPP