// File: PathfindingUtils.hpp
#ifndef PATHFINDING_UTILS_HPP
#define PATHFINDING_UTILS_HPP

#include <vector>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <stdexcept>
#include <algorithm>

// Define GridPoint - This needs to be visible to both nvcc and C++ compiler
struct GridPoint {
    int x = 0;
    int y = 0;
    bool operator==(const GridPoint& other) const {
        return x == other.x && y == other.y;
    }
    struct Hash {
        std::size_t operator()(const GridPoint& p) const {
            return static_cast<std::size_t>(p.x) * 73856093 ^ static_cast<std::size_t>(p.y) * 19349663;
        }
    };
};


namespace PathfindingUtils {

    // --- Constants ---
    // Use #ifndef __CUDACC__ to hide these from nvcc compiler
#ifndef __CUDACC__
    // These definitions are ONLY for standard C++ compilers
    inline constexpr int NUM_DIRECTIONS = 8;
    inline constexpr int dx[NUM_DIRECTIONS] = { 1, 0, -1, 0, 1, -1, -1, 1 };
    inline constexpr int dy[NUM_DIRECTIONS] = { 0, 1, 0, -1, 1, 1, -1, -1 };
    inline constexpr float costs[NUM_DIRECTIONS] = { // Base geometric costs (Distance factor)
        1.0f, 1.0f, 1.0f, 1.0f,
        1.41421356f, 1.41421356f, 1.41421356f, 1.41421356f
    };
    inline constexpr float EPSILON = 1e-6f; // Small value for float checks
    // Heuristic type constants
    inline constexpr int HEURISTIC_EUCLIDEAN = 0;
    inline constexpr int HEURISTIC_DIAGONAL = 1;
    inline constexpr int HEURISTIC_MANHATTAN = 2;
    inline constexpr int HEURISTIC_MIN_COST = 3; // Scaled Diagonal by min combined cost factor
#endif // __CUDACC__

    // --- Functions ---
    // These function definitions can remain as they are. nvcc can handle inline functions.
    // If they cause issues later, they could also be wrapped in #ifndef __CUDACC__
    // and declared separately for device code if needed, but usually it's fine.
    namespace internal {
        inline float euclidean_distance(int x1, int y1, int x2, int y2) {
            float dx_h = static_cast<float>(x1 - x2);
            float dy_h = static_cast<float>(y1 - y2);
            return sqrtf(dx_h * dx_h + dy_h * dy_h);
        }

        inline float diagonal_distance(int x1, int y1, int x2, int y2) {
            float dx_h = std::fabs(static_cast<float>(x1 - x2));
            float dy_h = std::fabs(static_cast<float>(y1 - y2));
            // --- Small Fix: Use explicit indexing as 'costs' might be hidden from nvcc ---
#ifndef __CUDACC__
            // Use the C++ constant when compiling C++
            return costs[0] * (dx_h + dy_h) + (costs[4] - 2.0f * costs[0]) * std::min(dx_h, dy_h);
#else
            // Use explicit values when compiling with nvcc (as costs array might be hidden)
            // Assuming costs[0] is 1.0f and costs[4] is sqrt(2)
            constexpr float cost0 = 1.0f;
            constexpr float cost4 = 1.41421356f;
            return cost0 * (dx_h + dy_h) + (cost4 - 2.0f * cost0) * std::min(dx_h, dy_h); // Use min, not std::min for nvcc
#endif
        }


        inline float manhattan_distance(int x1, int y1, int x2, int y2) {
            // std::abs might require <cmath> which could pull in things nvcc dislikes
            // Let's use a simple ternary for host/device compatibility here if needed
            // Or just rely on <cstdlib> being included, which is usually okay.
            // Keep std::abs for now.
            return static_cast<float>(std::abs(x1 - x2) + std::abs(y1 - y2));
        }
    } // namespace internal


    /**
     * @brief Calculates the heuristic estimate between two points based on type.
     *        Note: Heuristic operates on GRID coordinates.
     */
    inline float calculate_heuristic(int x1, int y1, int x2, int y2, int heuristic_type) {
        // --- Small Fix: Explicitly use internal:: functions ---
        // And hide HEURISTIC constants from nvcc if they cause issues
#ifndef __CUDACC__
        switch (heuristic_type) {
        case HEURISTIC_DIAGONAL:
            return internal::diagonal_distance(x1, y1, x2, y2);
        case HEURISTIC_MANHATTAN:
            return internal::manhattan_distance(x1, y1, x2, y2);
        case HEURISTIC_MIN_COST:
            return internal::diagonal_distance(x1, y1, x2, y2) * 0.8f;
        case HEURISTIC_EUCLIDEAN:
        default:
            return internal::euclidean_distance(x1, y1, x2, y2);
        }
#else
        // Provide a default or error if nvcc tries to use this (it shouldn't)
        // Delta-stepping doesn't use heuristics anyway.
        return 0.0f; // Or trigger an error if called inappropriately
#endif
    }

    // --- Coordinate/Index Helpers ---
    inline int toIndex(int x, int y, int width) {
        return y * width + x;
    }

    inline void toCoords(int index, int width, int& x, int& y) {
        if (width <= 0) {
            x = -1; y = -1;
        }
        else {
            y = index / width;
            x = index % width;
        }
    }

} // namespace PathfindingUtils

#endif // PATHFINDING_UTILS_HPP