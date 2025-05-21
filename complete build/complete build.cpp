//////#include "MapProcessor.hpp"
//////#include "MapProcessingCommon.h"
//////#include "PathfindingUtils.hpp"
//////#include "AStarToblerSampled.hpp" // Include the A* function header
//////#include "DebugUtils.hpp" 
//////#include "GeoRefScanner.hpp" // Include the georeferencing scanner
//////// --- main.cpp (Snippets) ---
//////#include "ElevationFetcherPy.hpp"
/////////pøenést do elvation až zaène fungovat a pak paralizaci v void MapProcessor::applyFeatureSpecificRulesInternal a možná fix k vodì
//////// fixnout propojování konce cesty se zaèátkem cesty
//////#include <iostream>
//////#include <vector>
//////#include <string>
//////#include <optional>
//////#include <chrono>
//////#include <stdexcept>
//////#include <map>
//////#include <future> // For async
//////using namespace mapgeo;
//////using namespace Pathfinding;
//////using namespace PathfindingUtils;
//////using namespace debugutils;
//////using namespace mapscan;
//////
//////int main() {
//////    // --- Configuration ---
//////    //
//////    std::string inputFile = "E:\\Astarapps\\ConsoleApp1\\ConsoleApp1\\forest sample - kopie - kopie.xml";// ADJUST PATH
//////    //std::string inputFile = "D:\\OneDrive - VUT\\Plocha\\pavel\\liga pavel\\mapakpokus1.xml"; // ADJUST PATH
//////    //std::string inputFile = "D:\\OneDrive - VUT\\Plocha\\pavel\\lite_map - Copy.xml";
//////    int gridWidth = 100;  // Logical grid width
//////    int gridHeight = 100; // Logical grid height
//////    std::vector<std::string> layers = { "barrier" }; // XML layers to process
//////
//////
//////    mapscan::ScanResult scanResult = mapscan::scanXmlForGeoRefAndBounds(inputFile, layers);
//////
//////    // <<< NEW: Report results of the scan >>>
//////    if (scanResult.georeferencingFound) {
//////        std::cout << "Georeferencing Found:" << std::endl;
//////        if (scanResult.refUTM) {
//////            std::cout << "  UTM Ref Point: X=" << scanResult.refUTM.value().x << ", Y=" << scanResult.refUTM.value().y << std::endl;
//////        }
//////        else {
//////            std::cout << "  UTM Ref Point: Not found or invalid." << std::endl;
//////        }
//////        if (scanResult.refLatLon) {
//////            std::cout << "  Lat/Lon Ref Point: Lon=" << scanResult.refLatLon.value().x << ", Lat=" << scanResult.refLatLon.value().y << std::endl;
//////            std::cout << "  Map Scale: " << scanResult.mapScale.value_or(0.0) << std::endl;
//////        }
//////        else {
//////            std::cout << "  Lat/Lon Ref Point: Not found or invalid." << std::endl;
//////        }
//////    }
//////    else {
//////        std::cout << "Georeferencing: Not found in XML." << std::endl;
//////    }
//////
//////    // --- Pathfinding settings ---
//////    GridPoint start_point = { 0, 8 }; // Top-left corner
//////    GridPoint end_point = { gridWidth - 1, gridHeight - 55 }; // Bottom-right corner
//////    int heuristic_choice = HEURISTIC_MIN_COST; // Use the cost-adjusted heuristic
//////
//////    // Obstacle Config (terrain costs)
//////    ObstacleConfigMap obstacleValues = {
//////        {"201", -1.0f}, {"301", -1.0f}, {"307", -1.0f}, {"509", -1.0f},
//////        {"513", -1.0f}, {"514", -1.0f}, {"515", -1.0f}, {"516", -1.0f},
//////        {"520", -1.0f}, {"526", -1.0f}, {"528", -1.0f}, {"529", -1.0f},
//////        {"206", -1.0f}, {"417", -1.0f}, {"202", 10.0f}, {"210", 1.25f},
//////        {"211", 1.67f}, {"212", 5.0f},  {"213", 1.25f}, {"302", 5.0f},
//////        {"308", 2.0f},  {"309", 1.67f}, {"310", 1.43f}, {"403", 1.25f},
//////        {"404", 1.25f}, {"406", 1.25f}, {"407", 1.25f}, {"408", 1.67f},
//////        {"409", 1.67f}, {"410", 5.0f},  {"412", 1.11f}, {"413", 1.11f},
//////        {"414", 1.11f}, {"311", 1.01f}, {"401", 1.0f},  {"402", 1.0f},
//////        {"405", 1.0f},  {"501", 1.0f},  {"502", 1.0f},  {"503", 1.0f},
//////        {"504", 1.0f},  {"505", 1.0f},  {"506", 1.0f},  {"507", 1.0f},
//////        {"508", 1.0f},  {"519", 1.0f},  {"518", -1.0f},  {"527", 1.0f}
//////    };
//////
//////
//////    std::cout << "--- Map Processing ---" << std::endl;
//////
//////    try {
//////        // 1. Process map and get logical grid + resolution info
//////        MapProcessorConfig procConfig;
//////        procConfig.grid_width = gridWidth;
//////        procConfig.grid_height = gridHeight;
//////        procConfig.layers_to_process = layers;
//////        MapProcessor processor(procConfig);
//////
//////        if (!processor.loadMap(inputFile)) { std::cerr << "Failed map load\n"; return 1; }
//////
//////        auto start_grid_gen = std::chrono::high_resolution_clock::now();
//////        std::optional<Grid_V3> resultGridOpt = processor.generateGrid(obstacleValues);
//////        auto end_grid_gen = std::chrono::high_resolution_clock::now();
//////        std::chrono::duration<double, std::milli> grid_gen_duration = end_grid_gen - start_grid_gen;
//////
//////        if (!resultGridOpt) { std::cerr << "Failed grid gen\n"; return 1; }
//////        Grid_V3 logical_grid = std::move(resultGridOpt.value());
//////
//////        // Get calculated results from MapProcessor
//////        NormalizationResult normInfo = processor.getNormalizationResult();
//////        if (!normInfo.valid) {
//////            std::cerr << "Error: Normalization results from MapProcessor are invalid." << std::endl;
//////            return 1;
//////        }
//////        float logical_cell_resolution = static_cast<float>(processor.getAverageLogicalResolution()); // Using average
//////        float logical_origin_x = static_cast<float>(normInfo.min_x);
//////        float logical_origin_y = static_cast<float>(normInfo.min_y);
//////
//////
//////        if (logical_cell_resolution <= 0.0f) {
//////            std::cerr << "Error: Invalid logical cell resolution calculated (" << logical_cell_resolution << ").\n";
//////            return 1;
//////        }
//////
//////        std::cout << "Logical grid generated (" << logical_grid.width() << "x" << logical_grid.height()
//////            << ") in " << grid_gen_duration.count() << " ms." << std::endl;
//////        std::cout << "Logical Origin (Real World): X=" << logical_origin_x << ", Y=" << logical_origin_y << std::endl;
//////        std::cout << "Calculated Logical Cell Resolution: " << logical_cell_resolution << " units." << std::endl;
//////
//////
//////        // 2. Load/Simulate Elevation Data
//////        std::cout << "\n--- Elevation Data ---" << std::endl;
//////        // --- !! IMPORTANT !! ---
//////        // Replace simulation with loading real elevation data and its parameters
//////        int elevation_width = gridWidth + 10;   // Example: different width
//////        int elevation_height = gridHeight + 5; // Example: different height
//////        float elevation_resolution = 5.0f;     // Example: 5m resolution
//////        std::vector<float> elevation_values(elevation_width * elevation_height);
//////        // Example: Assume elevation origin is offset from logical origin
//////        // You MUST determine these from your elevation data source!
//////        float elevation_origin_x = logical_origin_x - 25.0f; // Example offset
//////        float elevation_origin_y = logical_origin_y + 10.0f; // Example offset
//////
//////        std::cout << "Simulating elevation data (" << elevation_width << "x" << elevation_height
//////            << ") with resolution " << elevation_resolution << "m..." << std::endl;
//////        // Simple slope simulation
//////        float base_elevation = 100.0f;
//////        float slope_val = 0.1f; // meters change per meter distance
//////        for (int y = 0; y < elevation_height; ++y) {
//////            for (int x = 0; x < elevation_width; ++x) {
//////                // Calc world pos relative to elevation origin
//////                float world_x_elev = static_cast<float>(x) * elevation_resolution;
//////                float world_y_elev = static_cast<float>(y) * elevation_resolution;
//////                elevation_values[static_cast<size_t>(y) * elevation_width + x] = base_elevation + world_x_elev * slope_val;
//////            }
//////        }
//////        std::cout << "Simulated elevation data created." << std::endl;
//////        std::cout << "Elevation Origin (Real World): X=" << elevation_origin_x << ", Y=" << elevation_origin_y << std::endl;
//////        // --- End Simulation ---
//////
//////
//////        // Calculate origin offsets needed by A*
//////        float origin_offset_x = elevation_origin_x - logical_origin_x;
//////        float origin_offset_y = elevation_origin_y - logical_origin_y;
//////        std::cout << "Calculated Origin Offset (Elev - Logical): X=" << origin_offset_x << ", Y=" << origin_offset_y << std::endl;
//////
//////
//////        // 3. Run A* Pathfinding
//////        std::cout << "\n--- A* Pathfinding ---" << std::endl;
//////        std::cout << "Finding path from (" << start_point.x << "," << start_point.y << ") to ("
//////            << end_point.x << "," << end_point.y << ")" << std::endl;
//////        std::cout << "Using Heuristic Type: " << heuristic_choice << std::endl;
//////
//////        if (!logical_grid.inBounds(start_point.x, start_point.y) || !logical_grid.inBounds(end_point.x, end_point.y)) {
//////            std::cerr << "Start/End became invalid?\n"; return 1;
//////        }
//////
//////        auto start_astar = std::chrono::high_resolution_clock::now();
//////        std::vector<int> path_indices = findAStarPath_Tobler_Sampled(
//////            logical_grid,
//////            elevation_values,
//////            elevation_width,
//////            elevation_height,
//////            logical_cell_resolution, // Use calculated value
//////            elevation_resolution,    // From elevation source
//////            origin_offset_x,         // Use calculated value
//////            origin_offset_y,         // Use calculated value
//////            start_point,
//////            end_point,
//////            heuristic_choice
//////        );
//////        auto end_astar = std::chrono::high_resolution_clock::now();
//////        std::chrono::duration<double, std::milli> astar_duration = end_astar - start_astar;
//////
//////        // 4. Display Results
//////        if (!path_indices.empty()) {
//////            std::cout << "\n--- Path Found ---" << std::endl;
//////            // ... (print path length, duration etc.) ...
//////
//////            // *** Conditionally Call the Debug Visualization ***
//////#if defined(_DEBUG) || defined(DEBUG)
//////            debugutils::debugVisualizeGridWithPath(logical_grid, path_indices, start_point, end_point);
//////            // You can also add other debug prints here
//////#endif
//////
//////        }
//////        else {
//////            std::cout << "\n--- Path Not Found ---" << std::endl;
//////
//////            // *** Optionally visualize grid even if no path found (only in debug) ***
//////#if defined(_DEBUG) || defined(DEBUG)
//////    // debugutils::debugVisualizeGridWithPath(logical_grid, path_indices, start_point, end_point);
//////#endif
//////        }
//////
//////
//////
//////    }
//////    catch (const std::exception& e) {
//////        std::cerr << "\n*** Critical Error: Standard Exception Caught: " << e.what() << " ***" << std::endl;
//////        return 1;
//////    }
//////    catch (...) {
//////        std::cerr << "\n*** Critical Error: Unknown exception occurred! ***" << std::endl;
//////        return 1;
//////    }
//////
//////}
//
//
////#include "MapProcessor.hpp"
////#include "MapProcessingCommon.h"
////#include "PathfindingUtils.hpp"
////#include "AStarToblerSampled.hpp"
////#include "ThetaStarToblerSampled.hpp"
////#include "DijkstraToblerSampled.hpp"
////#include "BFSToblerSampled.hpp"
////#include "GeoRefScanner.hpp"
////#include "ElevationFetcherPy.hpp"
////#include "DebugUtils.hpp"
////#include "LazyThetaStarToblerSampled.hpp"
////#include "WaypointExtractor.hpp" 
////#include "DeltaSteppingGPU.hpp"
////#include "HADS_GPU.hpp" // Include the HADS GPU header
////#include <iostream>
////#include <vector>
////#include <string>
////#include <optional>
////#include <chrono>
////#include <stdexcept>
////#include <map>
////// #include <future> // No longer needed for this version
////// #include <thread> // No longer needed for this version
////// #include <system_error> // No longer needed for this version
////#include <cmath>
////#include <iomanip>
////
////
////
////struct GpuTuningResult {
////    float best_delta = 0.0f;
////    float best_threshold = 0.0f;
////    double best_time_ms = std::numeric_limits<double>::max();
////    bool path_found = false; // Did the best config find a path?
////};
////
/////**
//// * @brief Performs simple parameter tuning for Delta-Stepping GPU.
//// *
//// * @param num_iterations Number of times to run pathfinding for each parameter pair to average time.
//// * @param logical_grid The logical cost grid.
//// * @param elevation_values The elevation data.
//// * @param elevation_width Width of elevation grid.
//// * @param elevation_height Height of elevation grid.
//// * @param log_cell_resolution Logical grid resolution.
//// * @param elev_cell_resolution Elevation grid resolution.
//// * @param origin_offset_x X offset.
//// * @param origin_offset_y Y offset.
//// * @param start Start point.
//// * @param end End point.
//// * @return GpuTuningResult Struct containing the best parameters found and the time.
//// */
////GpuTuningResult tuneDeltaSteppingGPU(
////    int num_iterations,
////    const mapgeo::Grid_V3& logical_grid,
////    const std::vector<float>& elevation_values,
////    int elevation_width, int elevation_height,
////    float log_cell_resolution, float elev_cell_resolution,
////    float origin_offset_x, float origin_offset_y,
////    const GridPoint& start, const GridPoint& end)
////{
////    GpuTuningResult best_result;
////
////    // --- Define Candidate Parameters ---
////    // TODO: Adjust these ranges based on expected costs from Tobler!
////    // Costs depend on base terrain, resolution, and slope.
////    std::vector<float> candidate_deltas = { 5.0f, 10.0f, 50.0f, 100.0f, 200.0f, 400.0f};
////    std::vector<float> candidate_thresholds = { 1.0f, 2.0f, 5.0f, 10.0f, 20.0f, 50.0f, 100.0f };
////
////    std::cout << "\n--- Starting Delta-Stepping GPU Tuning ---" << std::endl;
////    std::cout << "Iterations per parameter pair: " << num_iterations << std::endl;
////
////    for (float delta : candidate_deltas) {
////        for (float threshold : candidate_thresholds) {
////            std::cout << "  Testing Delta=" << delta << ", Threshold=" << threshold << "..." << std::flush;
////
////            double total_time_ms = 0.0;
////            bool path_found_this_config = false;
////            int runs_completed = 0;
////
////            for (int i = 0; i < num_iterations; ++i) {
////                auto start_time = std::chrono::high_resolution_clock::now();
////                std::vector<int> path = findPathGPU_DeltaStepping(
////                    logical_grid, elevation_values,
////                    elevation_width, elevation_height,
////                    log_cell_resolution, elev_cell_resolution,
////                    origin_offset_x, origin_offset_y,
////                    start, end,
////                    delta, threshold // Use current candidate parameters
////                );
////                auto end_time = std::chrono::high_resolution_clock::now();
////
////                // Only include time if a path was found (or if you want to average failures too)
////                if (!path.empty()) {
////                    std::chrono::duration<double, std::milli> duration = end_time - start_time;
////                    total_time_ms += duration.count();
////                    path_found_this_config = true; // Mark that at least one run found a path
////                    runs_completed++;
////                }
////                else {
////                    // Optionally break early if one run fails for this config?
////                    // Or just record that it failed at least once.
////                    path_found_this_config = false; // Mark as failed if any run fails
////                    std::cout << " [Run " << i + 1 << " FAILED] " << std::flush;
////                    // break; // Optional: Stop testing this config if one run fails
////                }
////            }
////
////            if (runs_completed > 0) { // Only consider configurations that succeeded at least once
////                double avg_time_ms = total_time_ms / runs_completed;
////                std::cout << " Avg Time: " << avg_time_ms << " ms." << std::endl;
////
////                // Update best result if this one is faster AND found a path
////                if (avg_time_ms < best_result.best_time_ms && path_found_this_config) {
////                    best_result.best_delta = delta;
////                    best_result.best_threshold = threshold;
////                    best_result.best_time_ms = avg_time_ms;
////                    best_result.path_found = true;
////                }
////            }
////            else {
////                std::cout << " All runs FAILED." << std::endl;
////            }
////        } // End threshold loop
////    } // End delta loop
////
////    std::cout << "--- Tuning Finished ---" << std::endl;
////    if (best_result.path_found) {
////        std::cout << "Best Parameters Found:" << std::endl;
////        std::cout << "  Delta:     " << best_result.best_delta << std::endl;
////        std::cout << "  Threshold: " << best_result.best_threshold << std::endl;
////        std::cout << "  Avg Time:  " << best_result.best_time_ms << " ms" << std::endl;
////    }
////    else {
////        std::cout << "No parameter configuration successfully found a path in all iterations." << std::endl;
////    }
////
////    return best_result;
////}
////
////// --- Using Namespaces ---
////using namespace mapgeo;
////using namespace Pathfinding;
////using namespace PathfindingUtils;
////using namespace mapscan;
////using namespace ElevationFetcher;
////using namespace debugutils;
////
////// --- RAII Helper for Python Initialization/Finalization ---
////struct PythonGuard { /* ... same as before ... */ };
////
////int main() {
////    // --- Initialize Python Interpreter ---
////    if (!ElevationFetcher::initializePython()) { /* ... error ... */ return 1; }
////    PythonGuard pythonGuard;
////
////    // --- Configuration ---
////    std::string inputFile = "E:\\Astarapps\\ConsoleApp1\\ConsoleApp1\\forest sample - kopie - kopie.xml";
////    //std::string inputFile = "D:\\OneDrive - VUT\\Plocha\\pavel\\liga pavel\\mapakpokus1.xml";
////    auto controlsFile = "D:\\OneDrive - VUT\\Plocha\\pavel\\pavel v3\\tester.xml"; // Path to the file containing 701, 702, 706
////    //std::string inputFile = "D:\\OneDrive - VUT\\Plocha\\pavel\\lite_map - Copy.xml"; // ADJUST
////    int gridWidth = 100;
////    int gridHeight = 100;
////    std::vector<std::string> layers = { "barrier" };
////    std::string pyModuleName = "elevation_logic";
////    std::string pyFetchFuncName = "get_elevation_grid";
////    std::string pyConvertFuncName = "convert_latlon_to_projected";
////    double desiredElevResolution = 90.0;
////
////    // --- Obstacle Config ---
////    ObstacleConfigMap obstacleValues = {
////                {"201", -1.0f}, {"301", -1.0f}, {"307", -1.0f}, {"509", -1.0f},
////                {"513", -1.0f}, {"514", -1.0f}, {"515", -1.0f}, {"516", -1.0f},
////                {"520", -1.0f}, {"526", -1.0f}, {"528", -1.0f}, {"529", -1.0f},
////                {"206", -1.0f}, {"417", -1.0f}, {"202", 10.0f}, {"210", 1.25f},
////                {"211", 1.67f}, {"212", 5.0f},  {"213", 1.25f}, {"302", 5.0f},
////                {"308", 2.0f},  {"309", 1.67f}, {"310", 1.43f}, {"403", 1.25f},
////                {"404", 1.25f}, {"406", 1.25f}, {"407", 1.25f}, {"408", 1.67f},
////                {"409", 1.67f}, {"410", 5.0f},  {"412", 1.11f}, {"413", 1.11f},
////                {"414", 1.11f}, {"311", 1.01f}, {"401", 1.0f},  {"402", 1.0f},
////                {"405", 1.0f},  {"501", 1.0f},  {"502", 1.0f},  {"503", 1.0f},
////                {"504", 1.0f},  {"505", 1.0f},  {"506", 1.0f},  {"507", 1.0f},
////                {"508", 1.0f},  {"519", 1.0f},  {"518", -1.0f},  {"527", 1.0f}
////            };
////
////    // --- Pre-Scan ---
////    std::cout << "--- Pre-Scan for GeoRef & Bounds ---" << std::endl;
////    mapscan::ScanResult scanResult = mapscan::scanXmlForGeoRefAndBounds(inputFile, layers);
////    bool canFetchElevation = false;
////    double mapScaleFromXml = 10000.0;
////    if (scanResult.georeferencingFound && scanResult.refLatLon &&
////        scanResult.rawBoundsUM && scanResult.rawBoundsUM->initialized && scanResult.mapScale)
////    {
////        canFetchElevation = true;
////        mapScaleFromXml = scanResult.mapScale.value();
////        // ... (Print scan results) ...
////    }
////    else { /* ... Log warnings ... */ }
////
////    // --- Pathfinding Settings ---
////   /* GridPoint start_point = { gridWidth / 10, gridHeight/2 };
////    GridPoint end_point = { gridWidth /5*4, gridHeight*2/3 };*/
////    GridPoint start_point = { 0, 8 }; // Top-left corner
////    GridPoint end_point = { gridWidth - 1, gridHeight - 55 }; // Bottom-right corner
////
////    int heuristic_choice = HEURISTIC_MIN_COST;
////
////    // --- Map Processing (Run first, completely) ---
////    std::cout << "\n--- Map Processing ---" << std::endl;
////    std::optional<Grid_V3> logical_grid_opt;
////    std::optional<mapgeo::NormalizationResult> normInfo_opt;
////    double logical_res_internal_units_um = 0.0;
////    double logical_origin_internal_x_um = 0.0;
////    double logical_origin_internal_y_um = 0.0;
////    
////    canFetchElevation = false; // Reset to false for elevation fetch
////    auto bounds = scanResult.rawBoundsUM.value();
////    std::optional<std::vector<GridPoint>> waypointsOpt = waypoint::extractWaypointsFromFile(
////        controlsFile, // Pass the file containing 701, 702, 706
////        bounds.min_x, bounds.max_x,  // Use bounds determined from main map scan
////        bounds.min_y, bounds.max_y,
////        gridWidth, gridHeight        // Target grid dimensions
////    );
////
////    if (!waypointsOpt || waypointsOpt.value().size() < 2) {
////        std::cerr << "Error: Failed to extract valid Start/Control/End sequence from '" << controlsFile << "'." << std::endl;
////        return 1;
////    }
////    const std::vector<GridPoint>& waypoints = waypointsOpt.value();
////    std::cout << "  Successfully extracted " << waypoints.size() << " waypoints." << std::endl;
////
////    try {
////        MapProcessorConfig procConfig; // Create the config struct
////
////        // --- ADD THESE LINES ---
////        procConfig.grid_width = gridWidth;     // Set the desired width (1000)
////        procConfig.grid_height = gridHeight;   // Set the desired height (1000)
////        procConfig.layers_to_process = layers; // Set the layers (good practice)
////        // -----------------------
////
////        MapProcessor processor(procConfig); // Pass the CORRECTLY configured struct
////        if (!processor.loadMap(inputFile)) { throw std::runtime_error("Map load failed"); }
////        logical_grid_opt = processor.generateGrid(obstacleValues);
////        if (!logical_grid_opt) { throw std::runtime_error("Grid generation failed"); }
////        normInfo_opt = processor.getNormalizationResult();
////        if (!normInfo_opt || !normInfo_opt->valid) { throw std::runtime_error("Normalization results invalid."); }
////        logical_res_internal_units_um = processor.getAverageLogicalResolution();
////        logical_origin_internal_x_um = normInfo_opt->min_x;
////        logical_origin_internal_y_um = normInfo_opt->min_y;
////        // ... (Print logical grid results) ...
////    }
////    catch (const std::exception& e) {
////        std::cerr << "Critical Error during map processing: " << e.what() << std::endl;
////        return 1;
////    }
////
////    // --- *** Call Python Fetch Synchronously (After Map Processing) *** ---
////    std::cout << "\n--- Preparing Elevation Data for A* (Synchronous Fetch) ---" << std::endl;
////    ElevationData elevationResult; // Will hold results from Python
////    auto useRealElevation = true;
////
////    if (canFetchElevation) {
////        const auto& rawBounds = scanResult.rawBoundsUM.value();
////        const auto& anchorLatLon = scanResult.refLatLon.value();
////        double anchorInternalX = 0.0;
////        double anchorInternalY = 0.0;
////
////        std::cout << "Calling Python elevation fetch function directly..." << std::endl;
////        auto start_fetch = std::chrono::high_resolution_clock::now();
////        // Call fetchElevationDataEmbedded directly in the main thread
////        elevationResult = fetchElevationDataEmbedded(
////            pyModuleName, pyFetchFuncName,
////            anchorLatLon.y, anchorLatLon.x, // Lat, Lon
////            anchorInternalX, anchorInternalY,
////            rawBounds.min_x, rawBounds.min_y, // µm
////            rawBounds.max_x, rawBounds.max_y, // µm
////            mapScaleFromXml,
////            desiredElevResolution
////        );
////        auto end_fetch = std::chrono::high_resolution_clock::now();
////        std::chrono::duration<double, std::milli> fetch_duration = end_fetch - start_fetch;
////        std::cout << "Python fetch function returned in " << fetch_duration.count() << " ms." << std::endl;
////
////        if (elevationResult.success && elevationResult.hasData()) {
////            std::cout << "Elevation data received successfully." << std::endl;
////            useRealElevation = true;
////            // ... (Print details) ...
////        }
////        else {
////            std::cerr << "Python elevation fetch failed. Reason: " << elevationResult.errorMessage << std::endl;
////        }
////    } // else: canFetchElevation was false, skip fetch
////
////    /*if (!useRealElevation) {
////        std::cout << "Using dummy elevation data for pathfinding." << std::endl;
////    }*/
////
////    // --- Calculate Final Parameters for A* ---
////    // ... (This section remains exactly the same as the previous main.cpp) ...
////    // ... (Calculate meters_per_internal_unit, log_cell_resolution_meters) ...
////    // ... (If useRealElevation, calculate logicalOriginProj and origin_offset_x/y) ...
////    // ... (If !useRealElevation, setup dummy parameters) ...
////    // --- Final parameter calculation logic copied from previous answer ---
////    std::vector<float> elevation_values_final;
////    int elevation_width_final = gridWidth;
////    int elevation_height_final = gridHeight;
////    double elevation_resolution_final = 1.0;
////    double elevation_origin_final_x = 0.0;
////    double elevation_origin_final_y = 0.0;
////    float origin_offset_x = 0.0f;
////    float origin_offset_y = 0.0f;
////    float log_cell_resolution_meters = 1.0f;
////
////    double meters_per_internal_unit = mapScaleFromXml / 1000000.0;
////    log_cell_resolution_meters = static_cast<float>(logical_res_internal_units_um * meters_per_internal_unit);
////
////    if (log_cell_resolution_meters <= 1e-6f) {
////        std::cerr << "Warning: Calculated logical cell resolution fallback. Using 1.0m." << std::endl;
////        log_cell_resolution_meters = 1.0f;
////    }
////
////    if (useRealElevation && canFetchElevation) {
////        elevation_values_final = std::move(elevationResult.values);
////        elevation_width_final = elevationResult.width;
////        elevation_height_final = elevationResult.height;
////        elevation_resolution_final = elevationResult.resolution_meters;
////        elevation_origin_final_x = elevationResult.origin_proj_x;
////        elevation_origin_final_y = elevationResult.origin_proj_y;
////
////        const auto& anchorLatLon = scanResult.refLatLon.value();
////        double anchorInternalX = 0.0; double anchorInternalY = 0.0;
////
////        std::cout << "Info: Converting anchor Lat/Lon to Projected CRS via Python..." << std::endl;
////        ProjectedPointResult anchorProj = convertLatLonToProjectedViaPython(
////            pyModuleName, pyConvertFuncName, anchorLatLon.x, anchorLatLon.y
////        );
////
////        if (anchorProj.success) {
////            double known_proj_x = anchorProj.x; double known_proj_y = anchorProj.y;
////            std::cout << "  Anchor Projected Coords: X=" << known_proj_x << " Y=" << known_proj_y << std::endl;
////
////            double origin_offset_x_m = (logical_origin_internal_x_um - anchorInternalX) * meters_per_internal_unit;
////            double origin_offset_y_m = (logical_origin_internal_y_um - anchorInternalY) * meters_per_internal_unit * -1.0; // Y-inversion
////
////            double logicalOriginProjX = known_proj_x + origin_offset_x_m;
////            double logicalOriginProjY = known_proj_y + origin_offset_y_m;
////            std::cout << "  Calculated Logical Origin (Projected): X=" << logicalOriginProjX << " Y=" << logicalOriginProjY << std::endl;
////
////            origin_offset_x = static_cast<float>(elevation_origin_final_x - logicalOriginProjX);
////            origin_offset_y = static_cast<float>(elevation_origin_final_y - logicalOriginProjY);
////
////        }
////        else { /* ... handle error, set useRealElevation=false ... */
////            std::cerr << "Error: Could not convert anchor Lat/Lon: " << anchorProj.error << ". Using zero offset." << std::endl;
////            useRealElevation = false;
////        }
////    }
////    useRealElevation = false; // For testing, force dummy elevation data
////    if (!useRealElevation) { /* ... Setup dummy data ... */
////        elevation_values_final.assign(static_cast<size_t>(gridWidth) * gridHeight, 100.0f);
////        elevation_width_final = gridWidth; elevation_height_final = gridHeight;
////        elevation_resolution_final = log_cell_resolution_meters > 1e-6f ? log_cell_resolution_meters : 1.0;
////        elevation_origin_final_x = 0.0; elevation_origin_final_y = 0.0;
////        origin_offset_x = 0.0f; origin_offset_y = 0.0f;
////    }
////    // --- End parameter calculation ---
////
////
////    // --- Run A* Pathfinding ---
////    std::cout << "\n---  Delta-Stepping GPU ( Pathfinding ---" << std::endl;
////    std::cout << "Finding path from (" << start_point.x << "," << start_point.y << ") to ("
////        << end_point.x << "," << end_point.y << ")" << std::endl;
////    std::cout << "Using Heuristic Type: " << heuristic_choice << std::endl;
////    
////    if (!logical_grid_opt->inBounds(start_point.x, start_point.y) || !logical_grid_opt->inBounds(end_point.x, end_point.y)) {
////        std::cerr << "Error: Start/End points out of logical grid bounds.\n";
////        return 1;
////    }
////
////    auto start_astar = std::chrono::high_resolution_clock::now();
////
////    
////    float delta_tuning_param = 100.0f; // Example - **NEEDS TUNING**
////    float light_edge_threshold_tuning_param = 2.0f; // Example - **NEEDS TUNING**
////    int   hads_heuristic_radius = 100;              // Example: Precompute heuristic within 100 grid cells of goal
////    float hads_prune_factor = 1.2f;                 // Example: Prune if neighbor h is > 20% worse than current h
////    float hads_heuristic_weight = 1.0f;             // Example: Use unweighted Euclidean distance initially
////
////
////    printf("DEBUG: Calling findPathGPU_DeltaStepping with:\n");
////    printf("  elevation_values_final.empty() = %s\n", elevation_values_final.empty() ? "true" : "false");
////    printf("  elevation_width_final = %d\n", elevation_width_final);
////    printf("  elevation_height_final = %d\n", elevation_height_final);
////    printf("  elevation_resolution_final = %f\n", static_cast<float>(elevation_resolution_final)); // Print the value being passed
////    fflush(stdout);
////
////    
////    /*std::vector<int> path_indices = findPathGPU_DeltaStepping(
////        logical_grid_opt.value(),           // The logical cost grid
////        elevation_values_final,             // Real or dummy elevation values
////        elevation_width_final,              // Width of elevation grid
////        elevation_height_final,             // Height of elevation grid
////        log_cell_resolution_meters,         // Logical grid resolution in meters
////        static_cast<float>(elevation_resolution_final), // Elevation grid resolution in meters
////        origin_offset_x,                    // Offset X in meters (Projected CRS)
////        origin_offset_y,                    // Offset Y in meters (Projected CRS)
////        start_point,                // Use segment start
////        end_point,                  // Use segment end
////        delta_tuning_param,                  // Delta tuning parameter
////        light_edge_threshold_tuning_param     // Light edge threshold tuning parameter
////    );*/
////
////    std::vector<int> path_indices = findPathGPU_HADS(
////        logical_grid_opt.value(),           // The logical cost grid
////        elevation_values_final,             // Real or dummy elevation values
////        elevation_width_final,              // Width of elevation grid
////        elevation_height_final,             // Height of elevation grid
////        log_cell_resolution_meters,         // Logical grid resolution in meters
////        static_cast<float>(elevation_resolution_final), // Elevation grid resolution in meters
////        origin_offset_x,                    // Offset X in meters (Projected CRS)
////        origin_offset_y,                    // Offset Y in meters (Projected CRS)
////        start_point,                // Use segment start
////        end_point,                 // Use segment end
////        // HADS Tuning Parameters:
////        delta_tuning_param,
////        light_edge_threshold_tuning_param,
////        hads_heuristic_radius,
////        hads_prune_factor,
////        hads_heuristic_weight
////    );
////
////
////    auto end_segment = std::chrono::high_resolution_clock::now();
////    std::chrono::duration<double, std::milli> astar_duration = end_segment - start_astar;
////
////
////    // --- Display Results ---
////    if (!path_indices.empty()) {
////        std::cout << "\n--- Path Found ---" << std::endl;
////        std::cout << "Path length: " << path_indices.size() << " cells." << std::endl;
////        std::cout << "A* search duration: " << astar_duration.count() << " ms." << std::endl;
////        // Add debug visualization if defined/needed
////#if defined(_DEBUG) || defined(DEBUG)
////        if (debugutils::debugVisualizeGridWithPath) { // Check if function pointer/symbol exists
////            debugutils::debugVisualizeGridWithPath(logical_grid_opt.value(), path_indices, start_point, end_point);
////        }
////#endif
////    }
////    else {
////        std::cout << "\n--- Path Not Found ---" << std::endl;
////        std::cout << "A* search duration: " << astar_duration.count() << " ms." << std::endl;
////    }
////
////
////    //int tuning_iterations = 1; // Number of runs per param pair for averaging
////    //GpuTuningResult tuning_result = tuneDeltaSteppingGPU(
////    //    tuning_iterations,
////    //    logical_grid_opt.value(),           // The logical cost grid
////    //    elevation_values_final,             // Real or dummy elevation values
////    //    elevation_width_final,              // Width of elevation grid
////    //    elevation_height_final,             // Height of elevation grid
////    //    log_cell_resolution_meters,         // Logical grid resolution in meters
////    //    static_cast<float>(elevation_resolution_final), // Elevation grid resolution in meters
////    //    origin_offset_x,                    // Offset X in meters (Projected CRS)
////    //    origin_offset_y,                    // Offset Y in meters (Projected CRS)
////    //    start_point,                // Use segment start
////    //    end_point
////    //);
////
////
////    //// You can now use the best parameters for final runs or further analysis
////    //if (tuning_result.path_found) {
////    //    std::cout << "\nRunning final pathfinding with best parameters..." << std::endl;
////    //    std::vector<int> final_path  = findPathGPU_DeltaStepping(
////    //        logical_grid_opt.value(),           // The logical cost grid
////    //        elevation_values_final,             // Real or dummy elevation values
////    //        elevation_width_final,              // Width of elevation grid
////    //        elevation_height_final,             // Height of elevation grid
////    //        log_cell_resolution_meters,         // Logical grid resolution in meters
////    //        static_cast<float>(elevation_resolution_final), // Elevation grid resolution in meters
////    //        origin_offset_x,                    // Offset X in meters (Projected CRS)
////    //        origin_offset_y,                    // Offset Y in meters (Projected CRS)
////    //        start_point,                // Use segment start
////    //        end_point,                  // Use segment end
////    //        tuning_result.best_delta, tuning_result.best_threshold 
////    //    );
////    //}
////    //else {
////    //    std::cout << "\nCould not find optimal parameters or a valid path during tuning." << std::endl;
////    //}
////
////    std::cout << "\nExecution finished." << std::endl;
////    //// PythonGuard calls finalizePython automatically when main exits
////    return 0;
////}
//
#include "MapProcessor.hpp"
#include "MapProcessingCommon.h"
#include "PathfindingUtils.hpp"
#include "AStarToblerSampled.hpp"
#include "ThetaStarToblerSampled.hpp"
#include "DijkstraToblerSampled.hpp"
#include "BFSToblerSampled.hpp"
#include "GeoRefScanner.hpp"
#include "ElevationFetcherPy.hpp"
#include "DebugUtils.hpp"
#include "LazyThetaStarToblerSampled.hpp"
#include "WaypointExtractor.hpp" // Included
#include "PathSaver.hpp"
#include "DeltaSteppingGPU.hpp"
#include "HADS_GPU.hpp" // Include the HADS GPU header
#include "AStarGPU.hpp"
#include <iostream>
#include <vector>
#include <string>
#include <optional>
#include <chrono>
#include <stdexcept>
#include <map>
#include <cmath>
#include <iomanip>
#include <numeric> // Needed for std::accumulate if calculating total path cost
#include <fstream> // For file output



struct GpuTuningResult {
    float best_delta = 0.0f;
    float best_threshold = 0.0f;
    double best_time_ms = std::numeric_limits<double>::max();
    bool path_found = false; // Did the best config find a path?
};

/**
 * @brief Performs simple parameter tuning for Delta-Stepping GPU.
 *
 * @param num_iterations Number of times to run pathfinding for each parameter pair to average time.
 * @param logical_grid The logical cost grid.
 * @param elevation_values The elevation data.
 * @param elevation_width Width of elevation grid.
 * @param elevation_height Height of elevation grid.
 * @param log_cell_resolution Logical grid resolution.
 * @param elev_cell_resolution Elevation grid resolution.
 * @param origin_offset_x X offset.
 * @param origin_offset_y Y offset.
 * @param start Start point.
 * @param end End point.
 * @return GpuTuningResult Struct containing the best parameters found and the time.
 */
GpuTuningResult tuneDeltaSteppingGPU(
    int num_iterations,
    const mapgeo::Grid_V3& logical_grid,
    const std::vector<float>& elevation_values,
    int elevation_width, int elevation_height,
    float log_cell_resolution, float elev_cell_resolution,
    float origin_offset_x, float origin_offset_y,
    const GridPoint& start, const GridPoint& end)
{
    GpuTuningResult best_result;

    // --- Define Candidate Parameters ---
    // TODO: Adjust these ranges based on expected costs from Tobler!
    // Costs depend on base terrain, resolution, and slope.
    std::vector<float> candidate_deltas = { 5.0f, 10.0f, 50.0f, 100.0f, 200.0f, 400.0f };
    std::vector<float> candidate_thresholds = { 1.0f, 2.0f, 5.0f, 10.0f, 20.0f, 50.0f, 100.0f };

    std::cout << "\n--- Starting Delta-Stepping GPU Tuning ---" << std::endl;
    std::cout << "Iterations per parameter pair: " << num_iterations << std::endl;

    for (float delta : candidate_deltas) {
        for (float threshold : candidate_thresholds) {
            std::cout << "  Testing Delta=" << delta << ", Threshold=" << threshold << "..." << std::flush;

            double total_time_ms = 0.0;
            bool path_found_this_config = false;
            int runs_completed = 0;

            for (int i = 0; i < num_iterations; ++i) {
                auto start_time = std::chrono::high_resolution_clock::now();
                std::vector<int> path = findPathGPU_DeltaStepping(
                    logical_grid, elevation_values,
                    elevation_width, elevation_height,
                    log_cell_resolution, elev_cell_resolution,
                    origin_offset_x, origin_offset_y,
                    start, end,
                    delta, threshold // Use current candidate parameters
                );
                auto end_time = std::chrono::high_resolution_clock::now();

                // Only include time if a path was found (or if you want to average failures too)
                if (!path.empty()) {
                    std::chrono::duration<double, std::milli> duration = end_time - start_time;
                    total_time_ms += duration.count();
                    path_found_this_config = true; // Mark that at least one run found a path
                    runs_completed++;
                }
                else {
                    // Optionally break early if one run fails for this config?
                    // Or just record that it failed at least once.
                    path_found_this_config = false; // Mark as failed if any run fails
                    std::cout << " [Run " << i + 1 << " FAILED] " << std::flush;
                    // break; // Optional: Stop testing this config if one run fails
                }
            }

            if (runs_completed > 0) { // Only consider configurations that succeeded at least once
                double avg_time_ms = total_time_ms / runs_completed;
                std::cout << " Avg Time: " << avg_time_ms << " ms." << std::endl;

                // Update best result if this one is faster AND found a path
                if (avg_time_ms < best_result.best_time_ms && path_found_this_config) {
                    best_result.best_delta = delta;
                    best_result.best_threshold = threshold;
                    best_result.best_time_ms = avg_time_ms;
                    best_result.path_found = true;
                }
            }
            else {
                std::cout << " All runs FAILED." << std::endl;
            }
        } // End threshold loop
    } // End delta loop

    std::cout << "--- Tuning Finished ---" << std::endl;
    if (best_result.path_found) {
        std::cout << "Best Parameters Found:" << std::endl;
        std::cout << "  Delta:     " << best_result.best_delta << std::endl;
        std::cout << "  Threshold: " << best_result.best_threshold << std::endl;
        std::cout << "  Avg Time:  " << best_result.best_time_ms << " ms" << std::endl;
    }
    else {
        std::cout << "No parameter configuration successfully found a path in all iterations." << std::endl;
    }

    return best_result;
}

// --- Using Namespaces ---
using namespace mapgeo;
using namespace Pathfinding;
using namespace PathfindingUtils;
using namespace mapscan;
using namespace ElevationFetcher;
//using namespace debugutils;
using namespace waypoint; // Added waypoint namespace

// --- RAII Helper for Python Initialization/Finalization ---
struct PythonGuard {
    PythonGuard() = default;
    ~PythonGuard() {
        ElevationFetcher::finalizePython();
        std::cout << "Python finalized." << std::endl;
    }
    // Prevent copying/moving
    PythonGuard(const PythonGuard&) = delete;
    PythonGuard& operator=(const PythonGuard&) = delete;
    PythonGuard(PythonGuard&&) = delete;
    PythonGuard& operator=(PythonGuard&&) = delete;
};

int main() {
    // --- Initialize Python Interpreter ---
    if (!ElevationFetcher::initializePython()) {
        std::cerr << "FATAL: Failed to initialize Python interpreter." << std::endl;
        return 1;
    }
    PythonGuard pythonGuard; // RAII guard

    // --- Configuration ---
    //std::string inputFile = "E:\\Astarapps\\ConsoleApp1\\ConsoleApp1\\forest sample - kopie - kopie.xml";
    //std::string inputFile = "D:\\OneDrive - VUT\\Plocha\\pavel\\liga pavel\\mapakpokus1.xml";
    //auto controlsFile = "D:\\OneDrive - VUT\\Plocha\\pavel\\pavel v3\\try2.omap";
    //std::string inputFile = "D:\\OneDrive - VUT\\Plocha\\pavel\\mapa.omap";
    //auto controlsFile = "D:\\OneDrive - VUT\\Plocha\\pavel\\tester.omap";
    std::string inputFile = "D:\\OneDrive - VUT\\Plocha\\pavel\\bigmap\\test_nuber-of_lines.omap";
    auto controlsFile = "D:\\OneDrive - VUT\\Plocha\\pavel\\bigmap\\test.omap";
    // Path to the file containing 701, 702, 706
    //std::string inputFile = "D:\\OneDrive - VUT\\Plocha\\pavel\\lite_map - Copy.xml"; // ADJUST
    int gridWidth = 3000; // Increased grid size
    int gridHeight = 3000; // Increased grid size
    std::vector<std::string> layers = { "barrier", "course" }; // Added "course" layer for waypoints
    std::string pyModuleName = "elevation_logic";
    std::string pyFetchFuncName = "get_elevation_grid";
    std::string pyConvertFuncName = "convert_latlon_to_projected";
    double desiredElevResolution = 90.0;

    // --- Obstacle Config --- (Keep your existing values)
    ObstacleConfigMap obstacleValues{
                        {"201", 30.0f}, {"301", -1.0f}, {"307", -1.0f}, {"509", 0.7f},
                        {"513", -1.0f}, {"514", -1.0f}, {"515", -1.0f}, {"516", -1.0f},
                        {"520", -1.0f}, {"526", -1.0f}, {"528", -1.0f}, {"529", -1.0f},
                        {"206", -1.0f}, {"417", -1.0f}, {"202", 10.0f}, {"210", 1.25f},
                        {"211", 1.67f}, {"212", 5.0f},  {"213", 1.25f}, {"302", 5.0f},
                        {"308", 2.0f},  {"309", 1.67f}, {"310", 1.43f}, {"403", 1.25f},
                        {"404", 1.25f}, {"406", 1.50f}, {"407", 1.50f}, {"408", 1.67f},
                        {"409", 1.67f}, {"410", 5.0f},  {"412", 1.11f}, {"413", 1.11f},
                        {"414", 1.11f}, {"311", 1.01f}, {"401", 1.0f},  {"402", 1.0f},
                        {"405", 1.0f},  {"501", 0.50f},  {"502", 0.5f},  {"503", 0.6f},
                        {"504", 0.6f},  {"505", 0.6f},  {"506", 0.6f},  {"507", 0.6f},
                        {"508", 0.65f},  {"519", 0.7f},  {"518", -1.0f},  {"527", 1.0f}
    };

    // --- Pre-Scan ---
    std::cout << "--- Pre-Scan for GeoRef & Bounds ---" << std::endl;
    mapscan::ScanResult scanResult = mapscan::scanXmlForGeoRefAndBounds(inputFile, layers);
    bool canFetchElevation = true;

    double mapScaleFromXml = 10000.0; // Default scale

    if (scanResult.georeferencingFound && scanResult.refLatLon &&
        scanResult.rawBoundsUM && scanResult.rawBoundsUM->initialized && scanResult.mapScale)
    {
        canFetchElevation = true;
        mapScaleFromXml = scanResult.mapScale.value();
        std::cout << "  GeoRef Found: Yes" << std::endl;
        std::cout << "  Anchor Lat/Lon: " << scanResult.refLatLon->y << ", " << scanResult.refLatLon->x << std::endl;
        std::cout << "  Raw Bounds (um): X[" << scanResult.rawBoundsUM->min_x << ", " << scanResult.rawBoundsUM->max_x
            << "], Y[" << scanResult.rawBoundsUM->min_y << ", " << scanResult.rawBoundsUM->max_y << "]" << std::endl;
        std::cout << "  Map Scale: 1:" << mapScaleFromXml << std::endl;
    }
    else {
        std::cerr << "Warning: Could not find complete georeferencing info in XML." << std::endl;
        if (!scanResult.rawBoundsUM || !scanResult.rawBoundsUM->initialized) {
            std::cerr << "Error: Could not determine coordinate bounds from XML. Cannot proceed." << std::endl;
            return 1; // Cannot proceed without bounds
        }
        if (!scanResult.mapScale) {
            std::cerr << "Warning: Map scale not found, assuming 1:" << mapScaleFromXml << std::endl;
        }
    }

    // --- Pathfinding Settings ---
    // Start/End points are now determined by waypoint extraction below
    int heuristic_choice = HEURISTIC_MIN_COST;

    // --- Map Processing (Run first, completely) ---
    std::cout << "\n--- Map Processing ---" << std::endl;
    std::optional<Grid_V3> logical_grid_opt;
    std::optional<mapgeo::NormalizationResult> normInfo_opt;
    double logical_res_internal_units_um = 0.0;
    double logical_origin_internal_x_um = 0.0;
    double logical_origin_internal_y_um = 0.0;

    try {
        MapProcessorConfig procConfig;
        procConfig.grid_width = gridWidth;
        procConfig.grid_height = gridHeight;
        procConfig.layers_to_process = layers; // Ensure waypoint layer included if needed

        MapProcessor processor(procConfig);
        if (!processor.loadMap(inputFile)) { throw std::runtime_error("Map load failed"); }

        auto start_grid_gen = std::chrono::high_resolution_clock::now();
        logical_grid_opt = processor.generateGrid(obstacleValues);
        auto end_grid_gen = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> grid_gen_duration = end_grid_gen - start_grid_gen;

        if (!logical_grid_opt) { throw std::runtime_error("Grid generation failed"); }

        normInfo_opt = processor.getNormalizationResult();
        if (!normInfo_opt || !normInfo_opt->valid) { throw std::runtime_error("Normalization results invalid."); }

        logical_res_internal_units_um = processor.getAverageLogicalResolution();
        logical_origin_internal_x_um = normInfo_opt->min_x;
        logical_origin_internal_y_um = normInfo_opt->min_y;

        std::cout << "  Logical grid generated (" << logical_grid_opt->width() << "x" << logical_grid_opt->height()
            << ") in " << grid_gen_duration.count() << " ms." << std::endl;
        std::cout << "  Logical Origin (Internal um): X=" << logical_origin_internal_x_um << ", Y=" << logical_origin_internal_y_um << std::endl;
        std::cout << "  Calculated Logical Cell Resolution (Internal um): " << logical_res_internal_units_um << std::endl;

    }
    catch (const std::exception& e) {
        std::cerr << "Critical Error during map processing: " << e.what() << std::endl;
        return 1;
    }

    // --- Waypoint Extraction (Using bounds from main map scan) ---
    std::cout << "\n--- Waypoint Extraction ---" << std::endl;
    if (!scanResult.rawBoundsUM || !scanResult.rawBoundsUM->initialized) {
        std::cerr << "Error: Cannot extract waypoints without valid bounds from main map scan." << std::endl;
        return 1;
    }
    const auto& bounds = scanResult.rawBoundsUM.value();
    std::optional<std::vector<GridPoint>> waypointsOpt = waypoint::extractWaypointsFromFile(
        controlsFile, // Pass the file containing 701, 702, 706
        bounds.min_x, bounds.max_x,  // Use bounds determined from main map scan
        bounds.min_y, bounds.max_y,
        gridWidth, gridHeight        // Target grid dimensions
    );

    if (!waypointsOpt || waypointsOpt.value().size() < 2) {
        std::cerr << "Error: Failed to extract valid Start/Control/End sequence from '" << controlsFile << "'." << std::endl;
        std::cerr << "  Ensure the file exists and contains exactly one '701' (Start) and one '706' (Finish) symbol object," << std::endl;
        std::cerr << "  and that all waypoint objects (701, 702, 706) are single points." << std::endl;
        return 1;
    }

    const std::vector<GridPoint>& waypoints = waypointsOpt.value();
    std::cout << "  Successfully extracted " << waypoints.size() << " waypoints (Start, Controls, End)." << std::endl;
    std::cout << "  Waypoint Sequence (Grid Coords): ";
    for (const auto& wp : waypoints) { std::cout << "(" << wp.x << "," << wp.y << ") "; }
    std::cout << std::endl;

    // --- Call Python Fetch Synchronously (After Map Processing) ---
    // (Keep your existing Python fetching logic exactly as it was)
    std::cout << "\n--- Preparing Elevation Data for Pathfinding (Synchronous Fetch) ---" << std::endl;
    ElevationData elevationResult;
    bool useRealElevation = true;
    canFetchElevation = true;
    //noreal data getting
    if (canFetchElevation && false) {
        const auto& rawBounds = scanResult.rawBoundsUM.value();
        const auto& anchorLatLon = scanResult.refLatLon.value();
        // Use 0,0 for anchor internal coords as they are relative to the anchor itself
        double anchorInternalX = 0.0;
        double anchorInternalY = 0.0;

        std::cout << "  Calling Python elevation fetch function directly..." << std::endl;
        auto start_fetch = std::chrono::high_resolution_clock::now();
        elevationResult = fetchElevationDataEmbedded(
            pyModuleName, pyFetchFuncName,
            anchorLatLon.y, anchorLatLon.x,
            anchorInternalX, anchorInternalY,
            rawBounds.min_x, rawBounds.min_y,
            rawBounds.max_x, rawBounds.max_y,
            mapScaleFromXml,
            desiredElevResolution
        );
        auto end_fetch = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> fetch_duration = end_fetch - start_fetch;
        std::cout << "  Python fetch function returned in " << fetch_duration.count() << " ms." << std::endl;

        if (elevationResult.success && elevationResult.hasData()) {
            std::cout << "  Elevation data received successfully (" << elevationResult.width << "x" << elevationResult.height
                << ", Res: " << elevationResult.resolution_meters << "m, Origin Proj: "
                << elevationResult.origin_proj_x << "," << elevationResult.origin_proj_y << ")." << std::endl;
            useRealElevation = true;
        }
        else {
            std::cerr << "Warning: Python elevation fetch failed. Reason: " << elevationResult.errorMessage << std::endl;
        }
    }
    else {
        std::cout << "  Skipping Python elevation fetch (GeoRef info incomplete)." << std::endl;
    }

    if (!useRealElevation) {
        std::cout << "  Using dummy elevation data for pathfinding." << std::endl;
    }

    // --- Calculate Final Parameters for Pathfinding ---
    // (Keep your existing parameter calculation logic exactly as it was)
    std::vector<float> elevation_values_final;
    int elevation_width_final = gridWidth;
    int elevation_height_final = gridHeight;
    double elevation_resolution_final = 1.0;
    double elevation_origin_final_x = 0.0;
    double elevation_origin_final_y = 0.0;
    float origin_offset_x = 0.0f;
    float origin_offset_y = 0.0f;
    float log_cell_resolution_meters = 1.0f;

    double meters_per_internal_unit = mapScaleFromXml / 1000000.0; // um to m conversion based on scale
    log_cell_resolution_meters = static_cast<float>(logical_res_internal_units_um * meters_per_internal_unit);

    if (log_cell_resolution_meters <= 1e-6f) {
        std::cerr << "Warning: Calculated logical cell resolution is near zero (" << log_cell_resolution_meters << "m). Check map scale and units. Using fallback 1.0m." << std::endl;
        log_cell_resolution_meters = 1.0f;
    }
    std::cout << "  Final Logical Cell Resolution (meters): " << log_cell_resolution_meters << std::endl;
    
    if (useRealElevation && canFetchElevation) {
        elevation_values_final = std::move(elevationResult.values);
        elevation_width_final = elevationResult.width;
        elevation_height_final = elevationResult.height;
        elevation_resolution_final = elevationResult.resolution_meters;
        elevation_origin_final_x = elevationResult.origin_proj_x;
        elevation_origin_final_y = elevationResult.origin_proj_y;

        const auto& anchorLatLon = scanResult.refLatLon.value();
        double anchorInternalX = 0.0; double anchorInternalY = 0.0; // Relative internal coords

        // Convert anchor Lat/Lon to Projected CRS to establish reference
        std::cout << "  Converting anchor Lat/Lon to Projected CRS via Python..." << std::endl;
        ProjectedPointResult anchorProj = convertLatLonToProjectedViaPython(
            pyModuleName, pyConvertFuncName, anchorLatLon.x, anchorLatLon.y
        );

        if (anchorProj.success) {
            double known_proj_x = anchorProj.x;
            double known_proj_y = anchorProj.y;
            std::cout << "    Anchor Projected Coords: X=" << known_proj_x << " Y=" << known_proj_y << std::endl;

            // Calculate projected coordinates of the logical grid's origin (0,0 internal)
            // Offset from anchor in internal units (um)
            double origin_offset_internal_x_um = logical_origin_internal_x_um - anchorInternalX;
            double origin_offset_internal_y_um = logical_origin_internal_y_um - anchorInternalY;
            // Convert offset to meters, applying Y-inversion for screen-like coords
            double origin_offset_x_m = origin_offset_internal_x_um * meters_per_internal_unit;
            double origin_offset_y_m = origin_offset_internal_y_um * meters_per_internal_unit * -1.0;

            double logicalOriginProjX = known_proj_x + origin_offset_x_m;
            double logicalOriginProjY = known_proj_y + origin_offset_y_m;
            std::cout << "    Calculated Logical Origin (Projected): X=" << logicalOriginProjX << " Y=" << logicalOriginProjY << std::endl;

            // Calculate the final offset needed by A*: Elevation Origin - Logical Origin (in projected CRS)
            origin_offset_x = static_cast<float>(elevation_origin_final_x - logicalOriginProjX);
            origin_offset_y = static_cast<float>(elevation_origin_final_y - logicalOriginProjY);
            std::cout << "  Final Origin Offset (Elev - Logical, meters): X=" << origin_offset_x << ", Y=" << origin_offset_y << std::endl;

        }
        else {
            std::cerr << "Warning: Could not convert anchor Lat/Lon: " << anchorProj.error << ". Using zero offset for pathfinding." << std::endl;
            useRealElevation = false; // Fallback to dummy if conversion fails
            origin_offset_x = 0.0f;
            origin_offset_y = 0.0f;
        }
    }
    
    // Setup dummy data if real data failed or wasn't available
    useRealElevation = false;
    if (!useRealElevation) {
        elevation_values_final.assign(static_cast<size_t>(gridWidth) * gridHeight, 100.0f); // Flat 100m elevation
        elevation_width_final = gridWidth;
        elevation_height_final = gridHeight;
        elevation_resolution_final = log_cell_resolution_meters > 1e-6 ? log_cell_resolution_meters : 1.0; // Match logical if possible
        elevation_origin_final_x = 0.0; // Dummy origin
        elevation_origin_final_y = 0.0;
        origin_offset_x = 0.0f; // No offset for dummy data
        origin_offset_y = 0.0f;
        std::cout << "  Using Dummy Elevation Grid: " << elevation_width_final << "x" << elevation_height_final
            << ", Res: " << elevation_resolution_final << "m, Flat @ 100m" << std::endl;
    }


    // --- Run Pathfinding Sequentially Between Waypoints ---
    std::cout << "\n--- Pathfinding (Sequential Waypoints) ---" << std::endl;
    std::cout << "Using Heuristic Type: " << heuristic_choice << std::endl;

    std::vector<int> full_path_indices;
    bool path_found_for_all_segments = true;
    double total_astar_duration_ms = 0.0;
    auto start_astar_full = std::chrono::high_resolution_clock::now();

    for (size_t i = 0; i < waypoints.size() - 1; ++i) {
        GridPoint segment_start_point = waypoints[i];
        GridPoint segment_end_point = waypoints[i + 1];

        std::cout << "\nCalculating segment " << i + 1 << "/" << waypoints.size() - 1 << ": From ("
            << segment_start_point.x << "," << segment_start_point.y << ") to ("
            << segment_end_point.x << "," << segment_end_point.y << ")" << std::endl;

        // --- Bounds Check ---
        if (!logical_grid_opt->inBounds(segment_start_point.x, segment_start_point.y)) {
            std::cerr << "Error: Segment Start Point (" << segment_start_point.x << "," << segment_start_point.y << ") is out of logical grid bounds." << std::endl;
            path_found_for_all_segments = false;
            break;
        }
        if (!logical_grid_opt->inBounds(segment_end_point.x, segment_end_point.y)) {
            std::cerr << "Error: Segment End Point (" << segment_end_point.x << "," << segment_end_point.y << ") is out of logical grid bounds." << std::endl;
            path_found_for_all_segments = false;
            break;
        }

        // --- Identical Start/End Check ---
        if (segment_start_point == segment_end_point) {
            std::cout << "  Segment start and end points are identical. Adding single point." << std::endl;
            int pointIndex = toIndex(segment_start_point.x, segment_start_point.y, gridWidth);
            if (full_path_indices.empty() || full_path_indices.back() != pointIndex) {
                full_path_indices.push_back(pointIndex);
            }
            continue; // Move to the next segment
        }

        // --- Call Pathfinding Function ---
        auto start_segment = std::chrono::high_resolution_clock::now();
        // *** Ensure you are calling the intended pathfinding algorithm ***

        float delta_tuning_param = 50.0f; // Example 
        float light_edge_threshold_tuning_param = 50.0f; // Example 5.0f 100.0f
        /*
        std::vector<int> segment_path_indices = findPathGPU_DeltaStepping(
            logical_grid_opt.value(),           // The logical cost grid
            elevation_values_final,             // Real or dummy elevation values
            elevation_width_final,              // Width of elevation grid
            elevation_height_final,             // Height of elevation grid
            log_cell_resolution_meters,         // Logical grid resolution in meters
            static_cast<float>(elevation_resolution_final), // Elevation grid resolution in meters
            origin_offset_x,                    // Offset X in meters (Projected CRS)
            origin_offset_y,                    // Offset Y in meters (Projected CRS)
            segment_start_point,                // Use segment start
            segment_end_point,                  // Use segment end
            delta_tuning_param,                  // Delta tuning parameter
            light_edge_threshold_tuning_param     // Light edge threshold tuning parameter
        );
        */
        /*std::vector<int> segment_path_indices = findAStarPath_Tobler_Sampled(
			logical_grid_opt.value(),           // The logical cost grid
			elevation_values_final,             // Real or dummy elevation values
			elevation_width_final,              // Width of elevation grid
			elevation_height_final,             // Height of elevation grid
			log_cell_resolution_meters,         // Logical grid resolution in meters
			static_cast<float>(elevation_resolution_final), // Elevation grid resolution in meters
			origin_offset_x,                    // Offset X in meters (Projected CRS)
			origin_offset_y,                    // Offset Y in meters (Projected CRS)
			segment_start_point,                // Use segment start
			segment_end_point,                   // Use segment end
            heuristic_choice // Heuristic choice
		);*/
       
        int   hads_heuristic_radius = 100;              // Example: Precompute heuristic within 100 grid cells of goal
        float hads_prune_factor = 1.00f;                 // Example: Prune if neighbor h is > 20% worse than current h
        float hads_heuristic_weight = 0.95f; 
        delta_tuning_param = 227.097f;
        light_edge_threshold_tuning_param = 3.0f;
        hads_heuristic_radius = 75;
        hads_prune_factor = 1.000f;
        hads_heuristic_weight = 2.050f;
        
        
        std::vector<int> segment_path_indices = findPathGPU_HADS(
            logical_grid_opt.value(),           // The logical cost grid
            elevation_values_final,             // Real or dummy elevation values
            elevation_width_final,              // Width of elevation grid
            elevation_height_final,             // Height of elevation grid
            log_cell_resolution_meters,         // Logical grid resolution in meters
            static_cast<float>(elevation_resolution_final), // Elevation grid resolution in meters
            origin_offset_x,                    // Offset X in meters (Projected CRS)
            origin_offset_y,                    // Offset Y in meters (Projected CRS)
            segment_start_point,                // Use segment start
            segment_end_point,                 // Use segment end
            // HADS Tuning Parameters:
            delta_tuning_param,
            light_edge_threshold_tuning_param,
            hads_heuristic_radius,
            hads_prune_factor,
            hads_heuristic_weight
        );

        /*
        std::vector<int> segment_path_indices = findPathGPU_AStar(
			logical_grid_opt.value(),           // The logical cost grid
			elevation_values_final,             // Real or dummy elevation values
			elevation_width_final,              // Width of elevation grid
			elevation_height_final,             // Height of elevation grid
			log_cell_resolution_meters,         // Logical grid resolution in meters
			static_cast<float>(elevation_resolution_final), // Elevation grid resolution in meters
			origin_offset_x,                    // Offset X in meters (Projected CRS)
			origin_offset_y,                    // Offset Y in meters (Projected CRS)
			segment_start_point,                // Use segment start
			segment_end_point,                   // Use segment end
            heuristic_choice                  // Heuristic choice
		);*/
        /*std::vector<int> segment_path_indices = findLazyThetaStarPath_Tobler_Sampled(
			logical_grid_opt.value(),           // The logical cost grid
			elevation_values_final,             // Real or dummy elevation values
			elevation_width_final,              // Width of elevation grid
			elevation_height_final,             // Height of elevation grid
			log_cell_resolution_meters,         // Logical grid resolution in meters
			static_cast<float>(elevation_resolution_final), // Elevation grid resolution in meters
			origin_offset_x,                    // Offset X in meters (Projected CRS)
			origin_offset_y,                    // Offset Y in meters (Projected CRS)
			segment_start_point,                // Use segment start
			segment_end_point,                  // Use segment end
			heuristic_choice                   // Heuristic choice
		);*/

        auto end_segment = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> segment_duration = end_segment - start_segment;
        total_astar_duration_ms += segment_duration.count();

        // --- Check Segment Result & Concatenate ---
        if (segment_path_indices.empty()) {
            std::cerr << "  Error: Path not found for segment " << i + 1 << " (" << segment_duration.count() << " ms)." << std::endl;
            path_found_for_all_segments = false;
            break; // Stop processing further segments if one fails
        }

        std::cout << "  Segment " << i + 1 << " path found (" << segment_path_indices.size() << " cells, " << segment_duration.count() << " ms)." << std::endl;

        if (full_path_indices.empty()) {
            // First segment: add the whole path
            full_path_indices = std::move(segment_path_indices); // More efficient
        }
        else {
            // Subsequent segments: skip the first point (duplicate) if path has > 1 point
            if (segment_path_indices.size() > 1) {
                // Make sure the first point really is a duplicate before skipping
                if (full_path_indices.back() == segment_path_indices.front()) {
                    full_path_indices.insert(full_path_indices.end(),
                        std::make_move_iterator(segment_path_indices.begin() + 1),
                        std::make_move_iterator(segment_path_indices.end()));
                }
                else {
                    // This shouldn't normally happen if waypoints are distinct grid cells, but handle defensively
                    std::cerr << "Warning: Start of segment " << i + 1 << " doesn't match end of previous. Appending full segment." << std::endl;
                    full_path_indices.insert(full_path_indices.end(),
                        std::make_move_iterator(segment_path_indices.begin()),
                        std::make_move_iterator(segment_path_indices.end()));
                }
            }
            else if (segment_path_indices.size() == 1) {
                // Handle case where segment path is just one point
                if (full_path_indices.back() != segment_path_indices[0]) {
                    full_path_indices.push_back(segment_path_indices[0]);
                }
            }
            // If segment_path_indices was empty, we already broke the loop
        }
    } // End loop through waypoints

    auto end_astar_full = std::chrono::high_resolution_clock::now(); // Mark overall end time

    // --- Display Final Combined Results ---
    std::cout << "\n--- Final Path Result ---" << std::endl;
    if (path_found_for_all_segments) {
        std::cout << "Status: Full path successfully generated through all " << waypoints.size() << " waypoints." << std::endl;
        std::cout << "Total Path Length: " << full_path_indices.size() << " cells." << std::endl;
        std::cout << "Total Pathfinding Duration (Sum of segments): " << total_astar_duration_ms << " ms." << std::endl;
        std::cout << "Full Path indices (first/last 10): ";
        size_t n_print = 10;
        if (full_path_indices.size() <= 2 * n_print) {
            for (int idx : full_path_indices) { std::cout << idx << " "; }
        }
        else {
            for (size_t i = 0; i < n_print; ++i) { std::cout << full_path_indices[i] << " "; }
            std::cout << "... ";
            for (size_t i = full_path_indices.size() - n_print; i < full_path_indices.size(); ++i) { std::cout << full_path_indices[i] << " "; }
        }
        std::cout << std::endl;

        // Add debug visualization if defined/needed
#if defined(_DEBUG) || defined(DEBUG)
        if (debugutils::debugVisualizeGridWithPath && !full_path_indices.empty()) {
            // Visualize the full path using the first and last waypoint as nominal start/end
            debugutils::debugVisualizeGridWithPath(logical_grid_opt.value(), full_path_indices, waypoints.front(), waypoints.back());
        }
#endif

    }
    else {
        std::cout << "Status: Path generation failed. Could not connect all waypoints." << std::endl;
        std::cout << "Partial Path Length (up to failure): " << full_path_indices.size() << " cells." << std::endl;
        std::cout << "Total Pathfinding Duration (until failure): " << total_astar_duration_ms << " ms." << std::endl;
    }
    // --- *** NEW SECTION: Save Path to OMAP File *** ---
    if (path_found_for_all_segments && !full_path_indices.empty()) { // Check path isn't empty
        std::cout << "\n--- Saving Path to OMAP File ---" << std::endl;

        // *** CONVERT controlsFile (const char*) to std::string ***
        std::string controlsFileStr(controlsFile);
        // *********************************************************

        // Construct output filename using the std::string version
        std::string outputOmapPath;
        try {
            std::string dirPath = "";
            std::string baseName = controlsFileStr; // Start with the string version

            // Find the position of the last path separator
            size_t slashPos = controlsFileStr.find_last_of("/\\"); // Use string version
            if (slashPos != std::string::npos) {
                // Extract directory path including the separator
                dirPath = controlsFileStr.substr(0, slashPos + 1); // Use string version
                // Extract base filename
                baseName = controlsFileStr.substr(slashPos + 1);    // Use string version
            }

            // Find the position of the last dot in the base filename
            size_t dotPosInBase = baseName.find_last_of(".");
            if (dotPosInBase != std::string::npos) {
                // Remove the extension from the base filename
                baseName = baseName.substr(0, dotPosInBase);
            }

            // Construct the final path
            outputOmapPath = dirPath + baseName + "_path.omap";

        }
        catch (const std::out_of_range& oor) {
            std::cerr << "  Error constructing output filename (out of range): " << oor.what() << std::endl;
            outputOmapPath = controlsFileStr + "_path.omap"; // Fallback using string
        }
        catch (const std::exception& e) {
            std::cerr << "  Error constructing output filename: " << e.what() << std::endl;
            outputOmapPath = controlsFileStr + "_path.omap"; // Fallback using string
        }

        if (outputOmapPath.empty()) {
            std::cerr << "  Error: Could not determine a valid output filename. Skipping save." << std::endl;
        }
        else {
            std::cout << "  Output file will be: " << outputOmapPath << std::endl;

            // 1. Copy the original controls file to the output path
            try {
                std::cout << "  Copying '" << controlsFileStr << "' to '" << outputOmapPath << "'..." << std::endl; // Use string version
                // Use the original const char* for ifstream constructor, as it's often more direct
                std::ifstream src(controlsFile, std::ios::binary);
                if (!src) {
                    // Use string version for error message concatenation
                    std::string errorMsg = "Could not open source file for copying: " + controlsFileStr;
                    throw std::runtime_error(errorMsg);
                }
                // Use string version for ofstream constructor if needed (though const char* usually works)
                std::ofstream dst(outputOmapPath, std::ios::binary | std::ios::trunc);
                if (!dst) {
                    // Use string version for error message concatenation
                    std::string errorMsg = "Could not open destination file for copying: " + outputOmapPath;
                    throw std::runtime_error(errorMsg);
                }
                dst << src.rdbuf();
                src.close();
                dst.close();
                if (!dst.good()) {
                    // Use string version for error message concatenation
                    std::string errorMsg = "Error occurred during file copy operation to: " + outputOmapPath;
                    throw std::runtime_error(errorMsg);
                }
                std::cout << "  File copied successfully." << std::endl;

                if (logical_grid_opt && normInfo_opt && normInfo_opt->valid) {
                    // The targetLayer argument here is now effectively ignored by PathSaver,
                    // but passing it doesn't hurt.
                    std::string targetLayer = "course"; // This is now ignored internally by savePathToOmap

                    if (pathsaver::savePathToOmap(outputOmapPath,
                        full_path_indices,
                        logical_grid_opt.value(),
                        normInfo_opt.value(),
                        "704", // Correct symbol code
                        targetLayer)) // Pass ignored argument
                    {
                        std::cout << "  Simplified path saving process completed successfully." << std::endl;
                    }
                    else {
                        std::cerr << "  Error: Path saving process failed (check PathSaver logs)." << std::endl;
                    }
                }
                else {
                    std::cerr << "  Error: Skipping path saving because logical grid or normalization info is invalid." << std::endl;
                }

            }
            catch (const std::exception& e) {
                std::cerr << "  Error during file copy or save process: " << e.what() << std::endl;
            }
        }
    }
    else if (!path_found_for_all_segments) {
        std::cout << "\n--- Skipping Path Saving (Path Not Found) ---" << std::endl;
    }
    else {
        std::cout << "\n--- Skipping Path Saving (Path is Empty) ---" << std::endl;
    }
    // --- End Save Path Section ---

    GridPoint start_point = { gridWidth / 10, gridHeight/2 };
    GridPoint end_point = { gridWidth /5*4, gridHeight*2/3 };
    auto tuning_iterations = 5;

    //GpuTuningResult tuning_result = tuneDeltaSteppingGPU(
    //            tuning_iterations,
    //            logical_grid_opt.value(),           // The logical cost grid
    //            elevation_values_final,             // Real or dummy elevation values
    //            elevation_width_final,              // Width of elevation grid
    //            elevation_height_final,             // Height of elevation grid
    //            log_cell_resolution_meters,         // Logical grid resolution in meters
    //            static_cast<float>(elevation_resolution_final), // Elevation grid resolution in meters
    //            origin_offset_x,                    // Offset X in meters (Projected CRS)
    //            origin_offset_y,                    // Offset Y in meters (Projected CRS)
    //            start_point,                // Use segment start
    //            end_point
    //        );
    //    
    //    
    //        // You can now use the best parameters for final runs or further analysis
    //        if (tuning_result.path_found) {
    //            std::cout << "\nRunning final pathfinding with best parameters..." << std::endl;
    //            std::vector<int> final_path  = findPathGPU_DeltaStepping(
    //                logical_grid_opt.value(),           // The logical cost grid
    //                elevation_values_final,             // Real or dummy elevation values
    //                elevation_width_final,              // Width of elevation grid
    //                elevation_height_final,             // Height of elevation grid
    //                log_cell_resolution_meters,         // Logical grid resolution in meters
    //                static_cast<float>(elevation_resolution_final), // Elevation grid resolution in meters
    //                origin_offset_x,                    // Offset X in meters (Projected CRS)
    //                origin_offset_y,                    // Offset Y in meters (Projected CRS)
    //                start_point,                // Use segment start
    //                end_point,                  // Use segment end
    //                tuning_result.best_delta, tuning_result.best_threshold 
    //            );
    //        }


    // ... (Rest of main, including Python finalization and return) ...
    std::cout << "\nExecution finished." << std::endl;
    return path_found_for_all_segments ? 0 : 1;
}
   /*       best_delta_params.best_delta = 113.549f;
            best_delta_params.best_threshold = 10.0f; 
            best_delta_params.path_found = true; // Assume tuning found a path
            best_delta_params.best_time_ms = 37.166; // Optional: store the known best time for reference
            best_hads_params.best_delta = 227.097f;
            best_hads_params.best_threshold = 10.0f;
            best_hads_params.best_hads_heuristic_radius = 100;
            best_hads_params.best_hads_prune_factor = 1.000f;
            best_hads_params.best_hads_heuristic_weight = 0.950f;
            best_hads_params.best_time_ms = 27.780; // Optional: store the known best time for reference
            best_hads_params.path_found = true; */
        

////#include "MapProcessor.hpp"
////#include "MapProcessingCommon.h"
////#include "PathfindingUtils.hpp"
////#include "AStarToblerSampled.hpp"
////#include "ThetaStarToblerSampled.hpp" // Assuming you have this header
////#include "LazyThetaStarToblerSampled.hpp" // Assuming you have this header
////// #include "DijkstraToblerSampled.hpp" // Add if available
////// #include "BFSToblerSampled.hpp"    // Add if available
////#include "GeoRefScanner.hpp"
////#include "ElevationFetcherPy.hpp"
////#include "DebugUtils.hpp" // Assuming you might have this
////#include "WaypointExtractor.hpp"
////#include "PathSaver.hpp"
////#include "DeltaSteppingGPU.hpp" // Assuming header exists
////#include "HADS_GPU.hpp"         // Assuming header exists
////#include "AStarGPU.hpp"         // Assuming header exists
////
////#include <iostream>
////#include <vector>
////#include <string>
////#include <optional>
////#include <chrono>
////#include <stdexcept>
////#include <map>
////#include <cmath>
////#include <iomanip>
////#include <numeric> // Needed for std::accumulate, std::inner_product
////#include <fstream> // For file output
////#include <limits>
////#include <algorithm> // For std::sort
////
////// --- Using Namespaces ---
////using namespace mapgeo;
////using namespace Pathfinding; // Assuming CPU algorithms are here
////using namespace PathfindingUtils;
////using namespace mapscan;
////using namespace ElevationFetcher;
//////using namespace debugutils;
////using namespace waypoint;
////// Assuming GPU functions are in global/other namespaces or adjust as needed
////
////// --- RAII Helper for Python Initialization/Finalization ---
////struct PythonGuard {
////    PythonGuard() = default;
////    ~PythonGuard() {
////        ElevationFetcher::finalizePython();
////        std::cout << "Python finalized." << std::endl;
////    }
////    PythonGuard(const PythonGuard&) = delete;
////    PythonGuard& operator=(const PythonGuard&) = delete;
////    PythonGuard(PythonGuard&&) = delete;
////    PythonGuard& operator=(PythonGuard&&) = delete;
////};
////
////// --- Experiment Configuration ---
////const int NUM_RUNS_PER_ALGORITHM = 5;
////const bool TUNE_GPU_PARAMS_FIRST = true; // Set to true to run tuning before comparisons
////const bool USE_DUMMY_ELEVATION_FOR_EXPERIMENTS = true; // <<< SET TO true TO SKIP PYTHON FETCH AND USE FLAT DATA >>>
////
////// --- Data Structures for Experiments ---
////
////// Algorithm identifiers
////enum class AlgorithmType {
////    CPU_ASTAR,
////    CPU_THETA_STAR,
////    CPU_LAZY_THETA_STAR,
////    GPU_DELTA_STEPPING,
////    GPU_HADS,
////    GPU_ASTAR
////};
////
////std::string algoToString(AlgorithmType type) {
////    switch (type) {
////    case AlgorithmType::CPU_ASTAR:             return "CPU A*";
////    case AlgorithmType::CPU_THETA_STAR:        return "CPU Theta*";
////    case AlgorithmType::CPU_LAZY_THETA_STAR:   return "CPU Lazy Theta*";
////    case AlgorithmType::GPU_DELTA_STEPPING:    return "GPU Delta-Stepping";
////    case AlgorithmType::GPU_HADS:              return "GPU HADS";
////    case AlgorithmType::GPU_ASTAR:             return "GPU A*";
////    default:                                   return "Unknown";
////    }
////}
////
////// Structure to hold results for one run
////struct ExperimentResult {
////    AlgorithmType algorithm;
////    int run_number;
////    double total_time_ms = 0.0;
////    size_t path_cell_count = 0;
////    double geometric_length_m = 0.0;
////    // double modelled_cost = 0.0; // Add later if needed
////    bool success = false;
////};
////
////// Structure for tuning results
////struct GpuTuningResult {
////    float best_delta = 0.0f;
////    float best_threshold = 0.0f;
////    // HADS specific best params
////    int   best_hads_heuristic_radius = 0;
////    float best_hads_prune_factor = 0.0f;
////    float best_hads_heuristic_weight = 0.0f;
////    // Common results
////    double best_time_ms = std::numeric_limits<double>::max();
////    bool path_found = false;
////};
////
////// --- Forward Declarations for Tuning Functions ---
////GpuTuningResult tuneDeltaSteppingGPU(
////    int num_iterations, const Grid_V3& logical_grid, const std::vector<float>& elevation_values,
////    int elev_width, int elev_height, float log_res_m, float elev_res_m,
////    float offset_x, float offset_y, const GridPoint& start, const GridPoint& end);
////
////GpuTuningResult tuneHADS_GPU(
////    int num_iterations, const Grid_V3& logical_grid, const std::vector<float>& elevation_values,
////    int elev_width, int elev_height, float log_res_m, float elev_res_m,
////    float offset_x, float offset_y, const GridPoint& start, const GridPoint& end);
////
////// --- Helper Function Implementations ---
////
/////**
//// * @brief Calculates the geometric length of a path in meters.
//// */
////double calculateGeometricPathLength(const std::vector<int>& path_indices, int grid_width, float log_cell_resolution_meters) {
////    if (path_indices.size() < 2) {
////        return 0.0;
////    }
////
////    double total_length = 0.0;
////    int prev_x, prev_y;
////    PathfindingUtils::toCoords(path_indices[0], grid_width, prev_x, prev_y);
////
////    for (size_t i = 1; i < path_indices.size(); ++i) {
////        int curr_x, curr_y;
////        PathfindingUtils::toCoords(path_indices[i], grid_width, curr_x, curr_y);
////
////        int dx_grid = std::abs(curr_x - prev_x);
////        int dy_grid = std::abs(curr_y - prev_y);
////
////        double segment_dist_grid = 0.0;
////        if (dx_grid == 1 && dy_grid == 1) { // Diagonal move
////            segment_dist_grid = PathfindingUtils::costs[4]; // Approx 1.414
////        }
////        else if (dx_grid <= 1 && dy_grid <= 1) { // Orthogonal move (or same cell)
////            segment_dist_grid = static_cast<double>(dx_grid + dy_grid); // Should be 1.0 or 0.0
////        }
////        else {
////            // This shouldn't happen with standard A* neighbors
////            std::cerr << "Warning: Unexpected grid step > 1 (" << dx_grid << "," << dy_grid << ") in path length calculation." << std::endl;
////            // Estimate using Euclidean distance in grid cells
////            segment_dist_grid = std::sqrt(static_cast<double>(dx_grid * dx_grid + dy_grid * dy_grid));
////        }
////
////        total_length += segment_dist_grid * log_cell_resolution_meters;
////
////        prev_x = curr_x;
////        prev_y = curr_y;
////    }
////    return total_length;
////}
////
////// --- Placeholder for Modelled Cost Calculation (Complex) ---
/////*
////double calculateModelledPathCost(
////    const std::vector<int>& path_indices,
////    const Grid_V3& logical_grid,
////    const std::vector<float>& elevation_values,
////    int elevation_width, int elevation_height,
////    float log_cell_resolution, float elev_cell_resolution,
////    float origin_offset_x, float origin_offset_y)
////{
////    if (path_indices.size() < 2) return 0.0;
////
////    double total_cost = 0.0;
////    ElevationSampler elevation_sampler(elevation_values, elevation_width, elevation_height,
////                                       elev_cell_resolution, origin_offset_x, origin_offset_y);
////    int log_width = logical_grid.width();
////    const float infinite_penalty = std::numeric_limits<float>::max();
////    const float log_diag_dist = log_cell_resolution * PathfindingUtils::costs[4]; // ~1.414 * res
////
////    int prev_idx = path_indices[0];
////    int prev_x, prev_y;
////    PathfindingUtils::toCoords(prev_idx, log_width, prev_x, prev_y);
////
////    for (size_t i = 1; i < path_indices.size(); ++i) {
////        int curr_idx = path_indices[i];
////        int curr_x, curr_y;
////        PathfindingUtils::toCoords(curr_idx, log_width, curr_x, curr_y);
////
////        // Determine direction index (crude way, assumes valid A* path)
////        int dx = curr_x - prev_x;
////        int dy = curr_y - prev_y;
////        int dir = -1;
////        for(int d=0; d < PathfindingUtils::NUM_DIRECTIONS; ++d) {
////            if (PathfindingUtils::dx[d] == dx && PathfindingUtils::dy[d] == dy) {
////                dir = d;
////                break;
////            }
////        }
////        if (dir == -1) {
////             std::cerr << "Warning: Could not determine direction for cost calculation step." << std::endl;
////             continue; // Skip cost for this segment
////        }
////
////
////        // --- Replicate Cost Calculation Logic from A* ---
////        float world_x_prev = (static_cast<float>(prev_x) + 0.5f) * log_cell_resolution;
////        float world_y_prev = (static_cast<float>(prev_y) + 0.5f) * log_cell_resolution;
////        float prev_elevation = elevation_sampler.getElevationAt(world_x_prev, world_y_prev);
////
////        float world_x_curr = (static_cast<float>(curr_x) + 0.5f) * log_cell_resolution;
////        float world_y_curr = (static_cast<float>(curr_y) + 0.5f) * log_cell_resolution;
////        float current_elevation = elevation_sampler.getElevationAt(world_x_curr, world_y_curr);
////
////        float delta_h = current_elevation - prev_elevation;
////        float delta_dist_world = (dir < 4) ? log_cell_resolution : log_diag_dist;
////        float S = (delta_dist_world > PathfindingUtils::EPSILON) ? (delta_h / delta_dist_world) : 0.0f;
////
////        float SlopeFactor = expf(-3.5f * fabsf(S + 0.05f));
////        float time_penalty = (SlopeFactor > PathfindingUtils::EPSILON) ? (1.0f / SlopeFactor) : infinite_penalty;
////
////        if (time_penalty >= infinite_penalty) {
////            std::cerr << "Warning: Infinite penalty encountered during path cost recalc." << std::endl;
////            // Decide how to handle - add a large cost? Skip?
////             total_cost += 1e10; // Add a large finite cost
////             continue;
////        }
////
////        const GridCellData& neighborCell = logical_grid.at(curr_x, curr_y);
////        float base_terrain_cost = neighborCell.value;
////        if (base_terrain_cost <= 0.0f || neighborCell.hasFlag(GridFlags::FLAG_IMPASSABLE)) {
////             std::cerr << "Warning: Path goes through cell marked impassable during cost recalc." << std::endl;
////             // Add large cost or handle as error
////             total_cost += 1e10;
////             continue;
////        }
////        float base_geometric_cost = PathfindingUtils::costs[dir];
////        float final_move_cost = base_geometric_cost * base_terrain_cost * time_penalty;
////
////        total_cost += static_cast<double>(final_move_cost);
////
////        // Update previous point
////        prev_idx = curr_idx;
////        prev_x = curr_x;
////        prev_y = curr_y;
////    }
////    return total_cost;
////}
////*/
////
/////**
//// * @brief Prints the collected experiment results in a formatted table.
//// */
////void printResultsTable(const std::vector<ExperimentResult>& results) {
////    if (results.empty()) {
////        std::cout << "No experiment results to display." << std::endl;
////        return;
////    }
////
////    std::cout << "\n\n--- Experiment Results Summary ---" << std::endl;
////    std::cout << std::left << std::setw(20) << "Algorithm"
////        << std::right << std::setw(15) << "Avg Time (ms)"
////        << std::setw(10) << "Std Dev"
////        << std::setw(18) << "Avg Length (m)"
////        << std::setw(15) << "Avg Cells"
////        << std::setw(10) << "Success" << std::endl;
////    std::cout << std::string(88, '-') << std::endl;
////
////    // Group results by algorithm
////    std::map<AlgorithmType, std::vector<ExperimentResult>> grouped_results;
////    for (const auto& res : results) {
////        grouped_results[res.algorithm].push_back(res);
////    }
////
////    // Sort algorithms for consistent output order (optional)
////    std::vector<AlgorithmType> sorted_algos;
////    for (auto const& [algo, res_list] : grouped_results) {
////        sorted_algos.push_back(algo);
////    }
////    std::sort(sorted_algos.begin(), sorted_algos.end());
////
////
////    // Calculate and print stats for each algorithm
////    for (const auto& algo_type : sorted_algos) {
////        const auto& algo_results = grouped_results[algo_type];
////        if (algo_results.empty()) continue;
////
////        double sum_time = 0.0;
////        double sum_sq_time = 0.0;
////        double sum_length = 0.0;
////        double sum_cells = 0.0;
////        int success_count = 0;
////        int valid_runs = 0; // Count runs that succeeded for averaging
////
////        for (const auto& res : algo_results) {
////            if (res.success) {
////                sum_time += res.total_time_ms;
////                sum_sq_time += res.total_time_ms * res.total_time_ms;
////                sum_length += res.geometric_length_m;
////                sum_cells += static_cast<double>(res.path_cell_count);
////                success_count++;
////                valid_runs++;
////            }
////        }
////
////        double avg_time = (valid_runs > 0) ? (sum_time / valid_runs) : 0.0;
////        double avg_length = (valid_runs > 0) ? (sum_length / valid_runs) : 0.0;
////        double avg_cells = (valid_runs > 0) ? (sum_cells / valid_runs) : 0.0;
////
////        // Calculate standard deviation for time
////        double std_dev_time = 0.0;
////        if (valid_runs > 1) {
////            double variance = (sum_sq_time / valid_runs) - (avg_time * avg_time);
////            if (variance < 0.0) variance = 0.0; // Avoid negative due to precision
////            std_dev_time = std::sqrt(variance);
////        }
////
////        std::cout << std::fixed << std::setprecision(2);
////        std::cout << std::left << std::setw(20) << algoToString(algo_type)
////            << std::right << std::setw(15) << avg_time
////            << std::setw(10) << std_dev_time
////            << std::setw(18) << avg_length
////            << std::fixed << std::setprecision(0) << std::setw(15) << avg_cells
////            << std::fixed << std::setprecision(2) // Reset for next lines
////            << std::setw(9) << success_count << "/" << algo_results.size()
////            << std::endl;
////    }
////    std::cout << std::string(88, '-') << std::endl;
////}
////
////
////// --- Main Function ---
////int main() {
////    // --- Initialize Python Interpreter ---
////    if (!ElevationFetcher::initializePython()) {
////        std::cerr << "FATAL: Failed to initialize Python interpreter." << std::endl;
////        return 1;
////    }
////    PythonGuard pythonGuard; // RAII guard
////
////    // --- Configuration ---
////    //std::string inputFile = "E:\\Astarapps\\ConsoleApp1\\ConsoleApp1\\forest sample - kopie - kopie.xml";
////    //std::string inputFile = "D:\\OneDrive - VUT\\Plocha\\pavel\\liga pavel\\mapakpokus1.xml";
////    //auto controlsFile = "D:\\OneDrive - VUT\\Plocha\\pavel\\pavel v3\\try2.omap";
////    std::string inputFile = "D:\\OneDrive - VUT\\Plocha\\pavel\\bigmap\\test_nuber-of_lines.omap";
////    auto controlsFile = "D:\\OneDrive - VUT\\Plocha\\pavel\\bigmap\\test.omap";
////    //std::string inputFile = "D:\\OneDrive - VUT\\Plocha\\pavel\\mapa.omap";
////    //auto controlsFile = "D:\\OneDrive - VUT\\Plocha\\pavel\\tester.omap";
////    // Path to the file containing 701, 702, 706
////    //std::string inputFile = "D:\\OneDrive - VUT\\Plocha\\pavel\\lite_map - Copy.xml"; // ADJUST
////    int gridWidth = 3000; // Increased grid size
////    int gridHeight = 3000; // Increased grid size
////    std::vector<std::string> layers = { "barrier", "course" }; // Added "course" layer for waypoints
////    std::string pyModuleName = "elevation_logic";
////    std::string pyFetchFuncName = "get_elevation_grid";
////    std::string pyConvertFuncName = "convert_latlon_to_projected";
////    double desiredElevResolution = 90.0; // Desired resolution for Python fetch
////    int default_heuristic_choice = HEURISTIC_MIN_COST; // Default heuristic
////
////
////    // --- Obstacle Config --- (Keep your existing values)
////    ObstacleConfigMap obstacleValues{
////                    {"201", 30.0f}, {"301", -1.0f}, {"307", -1.0f}, {"509",  0.7f},
////                    {"513", -1.0f}, {"514", -1.0f}, {"515", -1.0f}, {"516", -1.0f},
////                    {"520", -1.0f}, {"526", -1.0f}, {"528", -1.0f}, {"529", -1.0f},
////                    {"206", -1.0f}, {"417", -1.0f}, {"202", 10.0f}, {"210", 1.25f},
////                    {"211", 1.67f}, {"212", 5.0f},  {"213", 1.25f}, {"302", 5.0f},
////                    {"308", 2.0f},  {"309", 1.67f}, {"310", 1.43f}, {"403", 1.25f},
////                    {"404", 1.25f}, {"406", 1.50f}, {"407", 1.50f}, {"408", 1.67f},
////                    {"409", 1.67f}, {"410", 5.0f},  {"412", 1.11f}, {"413", 1.11f},
////                    {"414", 1.11f}, {"311", 1.01f}, {"401", 1.0f},  {"402", 1.0f},
////                    {"405", 1.0f},  {"501", 0.60f},  {"502", 0.6f},  {"503", 0.6f},
////                    {"504", 0.6f},  {"505", 0.6f},  {"506", 0.65f},  {"507", 0.75f},
////                    {"508", 0.8f},  {"519", 0.9f},  {"518", -1.0f},  {"527", 1.0f}
////    };
////
////
////
////
////    // --- Pre-Scan ---
////    std::cout << "--- Pre-Scan for GeoRef & Bounds ---" << std::endl;
////    mapscan::ScanResult scanResult = mapscan::scanXmlForGeoRefAndBounds(inputFile, layers);
////    double mapScaleFromXml = 10000.0; // Default scale
////
////    if (!scanResult.georeferencingFound || !scanResult.refLatLon ||
////        !scanResult.rawBoundsUM || !scanResult.rawBoundsUM->initialized || !scanResult.mapScale)
////    {
////        std::cerr << "Warning: Could not find complete georeferencing info. Elevation fetch might fail or use defaults." << std::endl;
////        // Don't exit yet, allow fallback to dummy data or processing without elevation
////    }
////    else {
////        mapScaleFromXml = scanResult.mapScale.value();
////        std::cout << "  GeoRef Found. Scale: 1:" << mapScaleFromXml << std::endl;
////    }
////    if (!scanResult.rawBoundsUM || !scanResult.rawBoundsUM->initialized) {
////        std::cerr << "Error: Cannot determine coordinate bounds from XML. Cannot proceed." << std::endl;
////        return 1;
////    }
////
////
////    // --- Map Processing (Run once) ---
////    std::cout << "\n--- Map Processing ---" << std::endl;
////    std::optional<Grid_V3> logical_grid_opt;
////    std::optional<mapgeo::NormalizationResult> normInfo_opt;
////    double logical_res_internal_units_um = 0.0;
////    double logical_origin_internal_x_um = 0.0;
////    double logical_origin_internal_y_um = 0.0;
////    float log_cell_resolution_meters = 1.0f; // Initialize with a fallback
////
////    try {
////        // ... (MapProcessor setup and execution as before) ...
////        MapProcessorConfig procConfig;
////        procConfig.grid_width = gridWidth;
////        procConfig.grid_height = gridHeight;
////        procConfig.layers_to_process = layers;
////
////        MapProcessor processor(procConfig);
////        if (!processor.loadMap(inputFile)) { throw std::runtime_error("Map load failed"); }
////
////        auto start_grid_gen = std::chrono::high_resolution_clock::now();
////        logical_grid_opt = processor.generateGrid(obstacleValues);
////        auto end_grid_gen = std::chrono::high_resolution_clock::now();
////        std::chrono::duration<double, std::milli> grid_gen_duration = end_grid_gen - start_grid_gen;
////
////        if (!logical_grid_opt) { throw std::runtime_error("Grid generation failed"); }
////
////        normInfo_opt = processor.getNormalizationResult();
////        if (!normInfo_opt || !normInfo_opt->valid) { throw std::runtime_error("Normalization results invalid."); }
////
////        logical_res_internal_units_um = processor.getAverageLogicalResolution();
////        logical_origin_internal_x_um = normInfo_opt->min_x;
////        logical_origin_internal_y_um = normInfo_opt->min_y;
////
////        // Calculate resolution in meters *after* successful processing
////        double meters_per_internal_unit = mapScaleFromXml / 1000000.0; // um to m
////        log_cell_resolution_meters = static_cast<float>(logical_res_internal_units_um * meters_per_internal_unit);
////        if (log_cell_resolution_meters <= 1e-6f) {
////            std::cerr << "Warning: Calculated logical cell resolution is near zero (" << log_cell_resolution_meters << "m). Using fallback 1.0m." << std::endl;
////            log_cell_resolution_meters = 1.0f;
////        }
////
////        std::cout << "  Logical grid generated (" << logical_grid_opt->width() << "x" << logical_grid_opt->height()
////            << ") in " << grid_gen_duration.count() << " ms." << std::endl;
////        std::cout << "  Logical Cell Resolution (meters): " << log_cell_resolution_meters << std::endl;
////
////    }
////    catch (const std::exception& e) {
////        std::cerr << "Critical Error during map processing: " << e.what() << std::endl;
////        return 1;
////    }
////
////
////    // --- Waypoint Extraction (Run once) ---
////    std::cout << "\n--- Waypoint Extraction ---" << std::endl;
////    const auto& bounds = scanResult.rawBoundsUM.value();
////    std::optional<std::vector<GridPoint>> waypointsOpt = waypoint::extractWaypointsFromFile(
////        controlsFile,
////        bounds.min_x, bounds.max_x, bounds.min_y, bounds.max_y,
////        gridWidth, gridHeight
////    );
////
////    if (!waypointsOpt || waypointsOpt.value().size() < 2) {
////        std::cerr << "Error: Failed to extract valid Start/Control/End sequence." << std::endl;
////        return 1;
////    }
////    const std::vector<GridPoint>& waypoints = waypointsOpt.value();
////    std::cout << "  Successfully extracted " << waypoints.size() << " waypoints." << std::endl;
////
////
////    // --- Elevation Data Preparation ---
////    std::cout << "\n--- Preparing Elevation Data ---" << std::endl;
////    std::vector<float> elevation_values_final;
////    int elevation_width_final = gridWidth;
////    int elevation_height_final = gridHeight;
////    double elevation_resolution_final = static_cast<double>(log_cell_resolution_meters); // Default to match logical
////    double elevation_origin_final_x = 0.0;
////    double elevation_origin_final_y = 0.0;
////    float origin_offset_x = 0.0f;
////    float origin_offset_y = 0.0f;
////    bool actualElevationDataUsed = false;
////
////    if (USE_DUMMY_ELEVATION_FOR_EXPERIMENTS) {
////        std::cout << "  Using DUMMY elevation data as requested." << std::endl;
////        elevation_values_final.assign(static_cast<size_t>(gridWidth) * gridHeight, 100.0f); // Flat 100m
////        elevation_width_final = gridWidth;
////        elevation_height_final = gridHeight;
////        elevation_resolution_final = static_cast<double>(log_cell_resolution_meters); // Match logical
////        elevation_origin_final_x = 0.0; // Dummy origin doesn't matter if offset is 0
////        elevation_origin_final_y = 0.0;
////        origin_offset_x = 0.0f; // No offset needed
////        origin_offset_y = 0.0f;
////    }
////    else if (scanResult.georeferencingFound && scanResult.refLatLon && scanResult.mapScale) {
////        // Attempt to fetch real data
////        std::cout << "  Attempting to fetch REAL elevation data via Python..." << std::endl;
////        const auto& rawBoundsUM = scanResult.rawBoundsUM.value();
////        const auto& anchorLatLon = scanResult.refLatLon.value();
////        double anchorInternalX = 0.0, anchorInternalY = 0.0; // Anchor's internal coords relative to itself
////
////        auto start_fetch = std::chrono::high_resolution_clock::now();
////        ElevationData elevationResult = fetchElevationDataEmbedded(
////            pyModuleName, pyFetchFuncName,
////            anchorLatLon.y, anchorLatLon.x,
////            anchorInternalX, anchorInternalY, // Anchor internal coords (0,0)
////            rawBoundsUM.min_x, rawBoundsUM.min_y, // Bounds in internal units (um)
////            rawBoundsUM.max_x, rawBoundsUM.max_y,
////            mapScaleFromXml, desiredElevResolution);
////        auto end_fetch = std::chrono::high_resolution_clock::now();
////        std::chrono::duration<double, std::milli> fetch_duration = end_fetch - start_fetch;
////        std::cout << "  Python fetch attempt completed in " << fetch_duration.count() << " ms." << std::endl;
////
////        if (elevationResult.success && elevationResult.hasData()) {
////            std::cout << "    Real elevation data received successfully." << std::endl;
////            elevation_values_final = std::move(elevationResult.values);
////            elevation_width_final = elevationResult.width;
////            elevation_height_final = elevationResult.height;
////            elevation_resolution_final = elevationResult.resolution_meters;
////            elevation_origin_final_x = elevationResult.origin_proj_x;
////            elevation_origin_final_y = elevationResult.origin_proj_y;
////
////            // Calculate offset (same logic as before)
////            ProjectedPointResult anchorProj = convertLatLonToProjectedViaPython(
////                pyModuleName, pyConvertFuncName, anchorLatLon.x, anchorLatLon.y);
////
////            if (anchorProj.success) {
////                double meters_per_internal_unit = mapScaleFromXml / 1000000.0;
////                double origin_offset_internal_x_um = logical_origin_internal_x_um - anchorInternalX;
////                double origin_offset_internal_y_um = logical_origin_internal_y_um - anchorInternalY;
////                double origin_offset_x_m = origin_offset_internal_x_um * meters_per_internal_unit;
////                double origin_offset_y_m = origin_offset_internal_y_um * meters_per_internal_unit * -1.0; // Invert Y for screen -> projected
////                double logicalOriginProjX = anchorProj.x + origin_offset_x_m;
////                double logicalOriginProjY = anchorProj.y + origin_offset_y_m;
////                origin_offset_x = static_cast<float>(elevation_origin_final_x - logicalOriginProjX);
////                origin_offset_y = static_cast<float>(elevation_origin_final_y - logicalOriginProjY);
////                std::cout << "    Calculated Origin Offset (Elev - Logical): X=" << origin_offset_x << ", Y=" << origin_offset_y << std::endl;
////                actualElevationDataUsed = true;
////            }
////            else {
////                std::cerr << "    Warning: Could not convert anchor Lat/Lon for offset calculation: " << anchorProj.error << std::endl;
////            }
////        }
////        else {
////            std::cerr << "    Warning: Python elevation fetch failed: " << elevationResult.errorMessage << std::endl;
////        }
////    }
////    else {
////        std::cout << "  Skipping REAL elevation fetch (incomplete GeoRef)." << std::endl;
////    }
////
////    // Fallback to dummy if real fetch failed or wasn't attempted and dummy wasn't forced
////    if (!actualElevationDataUsed && !USE_DUMMY_ELEVATION_FOR_EXPERIMENTS) {
////        std::cout << "  FALLBACK: Using dummy elevation data." << std::endl;
////        elevation_values_final.assign(static_cast<size_t>(gridWidth) * gridHeight, 100.0f);
////        elevation_width_final = gridWidth;
////        elevation_height_final = gridHeight;
////        elevation_resolution_final = static_cast<double>(log_cell_resolution_meters);
////        elevation_origin_final_x = 0.0;
////        elevation_origin_final_y = 0.0;
////        origin_offset_x = 0.0f;
////        origin_offset_y = 0.0f;
////    }
////    std::cout << "  Final Elevation Setup: " << elevation_width_final << "x" << elevation_height_final
////        << " Grid, Resolution: " << elevation_resolution_final << "m." << std::endl;
////
////
////    // --- Tuning Phase (Optional) ---
////    GpuTuningResult best_delta_params; // Store best Delta-Stepping params
////    GpuTuningResult best_hads_params;  // Store best HADS params
////
////    if (TUNE_GPU_PARAMS_FIRST) {
////        std::cout << "\n--- GPU Parameter Tuning Phase ---" << std::endl;
////        // Use the first segment for tuning (simpler, assumes somewhat representative)
////        if (waypoints.size() >= 2) {
////            GridPoint tune_start = waypoints[0];
////            GridPoint tune_end = waypoints[3]; // Use first segment
////
////            std::cout << "Tuning using segment: (" << tune_start.x << "," << tune_start.y << ") -> ("
////                << tune_end.x << "," << tune_end.y << ")" << std::endl;
////
////            /*best_delta_params = tuneDeltaSteppingGPU(
////                2, // Fewer iterations for tuning maybe?
////                logical_grid_opt.value(), elevation_values_final,
////                elevation_width_final, elevation_height_final,
////                log_cell_resolution_meters, static_cast<float>(elevation_resolution_final),
////                origin_offset_x, origin_offset_y, tune_start, tune_end);*/
////
////            best_delta_params.best_delta = 113.549f; 
////            best_delta_params.best_threshold = 10.0f; 
////            best_delta_params.path_found = true; // Assume tuning found a path
////            best_delta_params.best_time_ms = 37.166; // Optional: store the known best time for reference
////            /*best_hads_params = tuneHADS_GPU(
////                3,
////                logical_grid_opt.value(), elevation_values_final,
////                elevation_width_final, elevation_height_final,
////                log_cell_resolution_meters, static_cast<float>(elevation_resolution_final),
////                origin_offset_x, origin_offset_y, tune_start, tune_end);*/
////            best_hads_params.best_delta = 80.000f;
////            best_hads_params.best_threshold = 2.0f;
////            best_hads_params.best_hads_heuristic_radius = 500;
////            best_hads_params.best_hads_prune_factor = 1.000f;
////            best_hads_params.best_hads_heuristic_weight = 0.90f;
////            // 
////            best_hads_params.best_time_ms = 27.780; // Optional: store the known best time for reference
////            best_hads_params.path_found = true;
////        }
////        
////        else {
////            std::cerr << "Warning: Not enough waypoints for tuning. Skipping tuning phase." << std::endl;
////        }
////        std::cout << "--- Tuning Phase Complete ---" << std::endl;
////    }
////    else {
////        std::cout << "\n--- Skipping GPU Parameter Tuning ---" << std::endl;
////        // Set default parameters if not tuning
////        best_delta_params.best_delta = 50.0f; // Default
////        best_delta_params.best_threshold = 10.0f; // Default
////        best_hads_params.best_delta = best_delta_params.best_delta; // Use same delta for HADS default
////        best_hads_params.best_threshold = best_delta_params.best_threshold; // Use same threshold
////        best_hads_params.best_hads_heuristic_radius = 100; // Default
////        best_hads_params.best_hads_prune_factor = 1.05f; // Default
////        best_hads_params.best_hads_heuristic_weight = 1.0f; // Default
////        best_hads_params.path_found = true; // Assume default params might work
////        best_delta_params.path_found = true; // Assume default params might work
////    }
////
////    // --- Experiment Execution Phase ---
////    std::cout << "\n\n--- Starting Algorithm Comparison Runs ---" << std::endl;
////    std::cout << "Number of runs per algorithm: " << NUM_RUNS_PER_ALGORITHM << std::endl;
////
////    std::vector<AlgorithmType> algorithms_to_test = {
////        //AlgorithmType::CPU_ASTAR,
////        //AlgorithmType::CPU_THETA_STAR,
////        //AlgorithmType::CPU_LAZY_THETA_STAR,
////        AlgorithmType::GPU_DELTA_STEPPING,
////        AlgorithmType::GPU_HADS,
////        //AlgorithmType::GPU_ASTAR
////    };
////
////    std::vector<ExperimentResult> all_results;
////
////    for (const auto& algo_type : algorithms_to_test) {
////        std::string algo_name = algoToString(algo_type);
////        std::cout << "\n--- Testing Algorithm: " << algo_name << " ---" << std::endl;
////
////        // Skip GPU algos if tuning failed to find *any* working parameters
////        if ((algo_type == AlgorithmType::GPU_DELTA_STEPPING && !best_delta_params.path_found && TUNE_GPU_PARAMS_FIRST) ||
////            (algo_type == AlgorithmType::GPU_HADS && !best_hads_params.path_found && TUNE_GPU_PARAMS_FIRST)) {
////            std::cout << "  Skipping " << algo_name << " because tuning failed to find working parameters." << std::endl;
////            continue;
////        }
////
////
////        for (int run = 1; run <= NUM_RUNS_PER_ALGORITHM; ++run) {
////            std::cout << "  Starting Run " << run << "/" << NUM_RUNS_PER_ALGORITHM << "..." << std::flush;
////
////            std::vector<int> full_path_indices;
////            bool path_found_for_all_segments = true;
////            double total_run_duration_ms = 0.0;
////            auto start_full_run = std::chrono::high_resolution_clock::now();
////
////            for (size_t i = 0; i < waypoints.size() - 1; ++i) {
////                GridPoint segment_start_point = waypoints[i];
////                GridPoint segment_end_point = waypoints[i + 1];
////
////                // Basic validation (redundant check, but safe)
////                if (!logical_grid_opt->inBounds(segment_start_point.x, segment_start_point.y) ||
////                    !logical_grid_opt->inBounds(segment_end_point.x, segment_end_point.y)) {
////                    std::cerr << "\nError: Invalid waypoint in Run " << run << " Segment " << i + 1 << std::endl;
////                    path_found_for_all_segments = false;
////                    break;
////                }
////                if (segment_start_point == segment_end_point) {
////                    int pointIndex = toIndex(segment_start_point.x, segment_start_point.y, gridWidth);
////                    if (full_path_indices.empty() || full_path_indices.back() != pointIndex) {
////                        full_path_indices.push_back(pointIndex);
////                    }
////                    continue;
////                }
////
////
////                auto start_segment = std::chrono::high_resolution_clock::now();
////                std::vector<int> segment_path_indices;
////
////                // --- Select and Run Algorithm ---
////                try {
////                    switch (algo_type) {
////                    case AlgorithmType::CPU_ASTAR:
////                        segment_path_indices = findAStarPath_Tobler_Sampled(
////                            logical_grid_opt.value(), elevation_values_final, elevation_width_final, elevation_height_final,
////                            log_cell_resolution_meters, static_cast<float>(elevation_resolution_final),
////                            origin_offset_x, origin_offset_y, segment_start_point, segment_end_point, default_heuristic_choice);
////                        break;
////                    case AlgorithmType::CPU_THETA_STAR:
////                        // Ensure Theta* function exists and signature matches
////                        segment_path_indices = findThetaStarPath_Tobler_Sampled( // Adjust function name if different
////                            logical_grid_opt.value(), elevation_values_final, elevation_width_final, elevation_height_final,
////                            log_cell_resolution_meters, static_cast<float>(elevation_resolution_final),
////                            origin_offset_x, origin_offset_y, segment_start_point, segment_end_point, default_heuristic_choice);
////                        break;
////                    case AlgorithmType::CPU_LAZY_THETA_STAR:
////                        //Ensure Lazy Theta* function exists and signature matches
////                        segment_path_indices = findLazyThetaStarPath_Tobler_Sampled( // Adjust function name if different
////                            logical_grid_opt.value(), elevation_values_final, elevation_width_final, elevation_height_final,
////                            log_cell_resolution_meters, static_cast<float>(elevation_resolution_final),
////                            origin_offset_x, origin_offset_y, segment_start_point, segment_end_point, default_heuristic_choice);
////                        break;
////                    case AlgorithmType::GPU_DELTA_STEPPING:
////                        segment_path_indices = findPathGPU_DeltaStepping(
////                            logical_grid_opt.value(), elevation_values_final, elevation_width_final, elevation_height_final,
////                            log_cell_resolution_meters, static_cast<float>(elevation_resolution_final),
////                            origin_offset_x, origin_offset_y, segment_start_point, segment_end_point,
////                            best_delta_params.best_delta, best_delta_params.best_threshold); // Use tuned params
////                        break;
////                    case AlgorithmType::GPU_HADS:
////                        segment_path_indices = findPathGPU_HADS(
////                            logical_grid_opt.value(), elevation_values_final, elevation_width_final, elevation_height_final,
////                            log_cell_resolution_meters, static_cast<float>(elevation_resolution_final),
////                            origin_offset_x, origin_offset_y, segment_start_point, segment_end_point,
////                            best_hads_params.best_delta, best_hads_params.best_threshold, // Use tuned base params
////                            best_hads_params.best_hads_heuristic_radius, // Use tuned HADS params
////                            best_hads_params.best_hads_prune_factor,
////                            best_hads_params.best_hads_heuristic_weight);
////                        break;
////                    case AlgorithmType::GPU_ASTAR:
////                        segment_path_indices = findPathGPU_AStar( // Adjust function name if different
////                            logical_grid_opt.value(), elevation_values_final, elevation_width_final, elevation_height_final,
////                            log_cell_resolution_meters, static_cast<float>(elevation_resolution_final),
////                            origin_offset_x, origin_offset_y, segment_start_point, segment_end_point, default_heuristic_choice);
////                        break;
////                        // Add cases for Dijkstra, BFS if implemented
////                    }
////                }
////                catch (const std::exception& path_ex) {
////                    std::cerr << "\nException during pathfinding call for " << algo_name << " Run " << run << " Segment " << i + 1 << ": " << path_ex.what() << std::endl;
////                    path_found_for_all_segments = false;
////                    segment_path_indices.clear(); // Ensure path is marked as empty/failed
////                }
////
////
////                auto end_segment = std::chrono::high_resolution_clock::now();
////                std::chrono::duration<double, std::milli> segment_duration = end_segment - start_segment;
////                // Don't add segment time to total run time here - do it once at the end
////
////                if (segment_path_indices.empty()) {
////                    std::cout << " [Seg " << i + 1 << " FAILED] " << std::flush;
////                    path_found_for_all_segments = false;
////                    break; // Stop processing segments for this run if one fails
////                }
////
////                // Concatenate paths (same logic as before)
////                if (full_path_indices.empty()) {
////                    full_path_indices = std::move(segment_path_indices);
////                }
////                else if (segment_path_indices.size() > 1 && full_path_indices.back() == segment_path_indices.front()) {
////                    full_path_indices.insert(full_path_indices.end(),
////                        std::make_move_iterator(segment_path_indices.begin() + 1),
////                        std::make_move_iterator(segment_path_indices.end()));
////                }
////                else if (segment_path_indices.size() >= 1 && (full_path_indices.empty() || full_path_indices.back() != segment_path_indices.front())) {
////                    // Handle single point path or non-matching start
////                    if (!full_path_indices.empty() && !segment_path_indices.empty() && full_path_indices.back() != segment_path_indices.front()) {
////                        std::cerr << "\nWarning: Path segments do not connect cleanly for " << algo_name << " Run " << run << ". Appending full segment." << std::endl;
////                    }
////                    full_path_indices.insert(full_path_indices.end(),
////                        std::make_move_iterator(segment_path_indices.begin()),
////                        std::make_move_iterator(segment_path_indices.end()));
////                }
////
////            } // End loop through waypoints/segments
////
////            auto end_full_run = std::chrono::high_resolution_clock::now();
////            total_run_duration_ms = std::chrono::duration<double, std::milli>(end_full_run - start_full_run).count();
////
////            // --- Record Results for this Run ---
////            ExperimentResult result;
////            result.algorithm = algo_type;
////            result.run_number = run;
////            result.success = path_found_for_all_segments;
////
////            if (result.success) {
////                result.total_time_ms = total_run_duration_ms;
////                result.path_cell_count = full_path_indices.size();
////                result.geometric_length_m = calculateGeometricPathLength(full_path_indices, gridWidth, log_cell_resolution_meters);
////                // result.modelled_cost = calculateModelledPathCost(...); // Add if implemented
////                std::cout << " Done. Time: " << result.total_time_ms << " ms, Length: " << result.geometric_length_m << " m, Cells: " << result.path_cell_count << std::endl;
////            }
////            else {
////                result.total_time_ms = total_run_duration_ms; // Record time even if failed
////                result.path_cell_count = 0;
////                result.geometric_length_m = 0.0;
////                std::cout << " FAILED. Time: " << result.total_time_ms << " ms." << std::endl;
////            }
////            all_results.push_back(result);
////
////            // Optionally save the path from the *first* successful run of each algorithm
////            if (run == 1 && result.success && !full_path_indices.empty()) {
////                std::cout << "    Saving path from first successful run..." << std::flush;
////                std::string outputOmapPath = controlsFile; // Start with input
////                size_t dotPos = outputOmapPath.find_last_of(".");
////                if (dotPos != std::string::npos) {
////                    outputOmapPath = outputOmapPath.substr(0, dotPos);
////                }
////                outputOmapPath += "_" + algo_name + "_path.omap";
////                // Replace spaces/asterisks in filename if necessary
////                std::replace(outputOmapPath.begin(), outputOmapPath.end(), ' ', '_');
////                std::replace(outputOmapPath.begin(), outputOmapPath.end(), '*', 'S');
////
////                // Reuse existing copy/save logic
////                try {
////                    std::ifstream src(controlsFile, std::ios::binary);
////                    std::ofstream dst(outputOmapPath, std::ios::binary | std::ios::trunc);
////                    if (src && dst) {
////                        dst << src.rdbuf();
////                        src.close(); dst.close();
////                        if (dst.good()) {
////                            if (pathsaver::savePathToOmap(outputOmapPath, full_path_indices, logical_grid_opt.value(), normInfo_opt.value(), "704.0", "course")) {
////                                std::cout << " Saved." << std::endl;
////                            }
////                            else { std::cout << " Save FAILED (PathSaver)." << std::endl; }
////                        }
////                        else { std::cout << " Save FAILED (Copy error)." << std::endl; }
////                    }
////                    else { std::cout << " Save FAILED (File open error)." << std::endl; }
////                }
////                catch (const std::exception& save_ex) {
////                    std::cerr << "\nException during path save for " << algo_name << ": " << save_ex.what() << std::endl;
////                }
////            }
////
////
////        } // End loop for multiple runs
////    } // End loop through algorithms
////
////
////    // --- Display Results Table ---
////    printResultsTable(all_results);
////
////    std::cout << "\nExperiment execution finished." << std::endl;
////    return 0; // Indicate success, even if some algorithms failed
////}
////
////
////// --- Tuning Function Implementations ---
////
////GpuTuningResult tuneDeltaSteppingGPU(
////    int num_iterations, const Grid_V3& logical_grid, const std::vector<float>& elevation_values,
////    int elev_width, int elev_height, float log_res_m, float elev_res_m,
////    float offset_x, float offset_y, const GridPoint& start, const GridPoint& end)
////{
////    GpuTuningResult best_result;
////    best_result.best_time_ms = std::numeric_limits<double>::max(); // Initialize to max
////
////    // --- Define Candidate Parameters ---
////    // These might need significant adjustment based on your expected edge costs!
////    std::vector<float> candidate_deltas = { 50.0f * log_res_m, 100.0f * log_res_m};//{ 1.0f * log_res_m, 5.0f * log_res_m, 10.0f * log_res_m, 20.0f * log_res_m, 50.0f * log_res_m, 100.0f * log_res_m }; // Scale delta roughly with resolution/cost?
////    std::vector<float> candidate_thresholds = { 1.0f, 2.0f, 5.0f, 10.0f, 20.0f }; // Threshold might be less dependent on scale
////
////    std::cout << "\n--- Starting Delta-Stepping GPU Tuning (Max " << num_iterations << " iter/pair) ---" << std::endl;
////
////    for (float delta : candidate_deltas) {
////        for (float threshold : candidate_thresholds) {
////            std::cout << "  Testing Delta=" << delta << ", Threshold=" << threshold << "..." << std::flush;
////            double total_time_ms = 0.0;
////            bool path_found_this_config = false;
////            int runs_completed_ok = 0;
////
////            for (int i = 0; i < num_iterations; ++i) {
////                auto start_time = std::chrono::high_resolution_clock::now();
////                std::vector<int> path;
////                try {
////                    path = findPathGPU_DeltaStepping( // Use the actual function name
////                        logical_grid, elevation_values, elev_width, elev_height,
////                        log_res_m, elev_res_m, offset_x, offset_y,
////                        start, end, delta, threshold);
////                }
////                catch (const std::exception& e) {
////                    std::cerr << "\nException during Delta-Stepping tuning run: " << e.what() << std::endl;
////                    path.clear(); // Mark as failed
////                }
////                auto end_time = std::chrono::high_resolution_clock::now();
////
////                if (!path.empty()) {
////                    total_time_ms += std::chrono::duration<double, std::milli>(end_time - start_time).count();
////                    path_found_this_config = true;
////                    runs_completed_ok++;
////                }
////                else {
////                    // Optional: Stop testing this config if one run fails?
////                    // path_found_this_config = false; // Mark failed only if *all* runs fail? Or if *any* run fails? Let's mark failed if *any* run fails.
////                    path_found_this_config = false;
////                    std::cout << " [Run " << i + 1 << " FAILED] " << std::flush;
////                    break; // Stop iterations for this pair if one fails
////                }
////            }
////
////            if (runs_completed_ok > 0 && path_found_this_config) { // Only consider if all runs succeeded
////                double avg_time_ms = total_time_ms / runs_completed_ok;
////                std::cout << " Avg Time: " << std::fixed << std::setprecision(3) << avg_time_ms << " ms." << std::endl;
////                if (avg_time_ms < best_result.best_time_ms) {
////                    best_result.best_delta = delta;
////                    best_result.best_threshold = threshold;
////                    best_result.best_time_ms = avg_time_ms;
////                    best_result.path_found = true; // Mark that we found at least one working config
////                }
////            }
////            else {
////                std::cout << " Failed or Inconsistent." << std::endl;
////            }
////        } // End threshold loop
////    } // End delta loop
////
////    std::cout << "--- Delta-Stepping Tuning Finished ---" << std::endl;
////    if (best_result.path_found) {
////        std::cout << "  Best Params: Delta=" << best_result.best_delta
////            << ", Threshold=" << best_result.best_threshold
////            << " (Avg Time: " << best_result.best_time_ms << " ms)" << std::endl;
////    }
////    else {
////        std::cout << "  No parameter configuration successfully found a path in all iterations." << std::endl;
////        // Use some default if tuning fails?
////        best_result.best_delta = 50.0f;
////        best_result.best_threshold = 10.0f;
////        best_result.path_found = false; // Still mark as not found via tuning
////    }
////    return best_result;
////}
////
////
////GpuTuningResult tuneHADS_GPU(
////    int num_iterations, const Grid_V3& logical_grid, const std::vector<float>& elevation_values,
////    int elev_width, int elev_height, float log_res_m, float elev_res_m,
////    float offset_x, float offset_y, const GridPoint& start, const GridPoint& end)
////{
////    GpuTuningResult best_result;
////    best_result.best_time_ms = std::numeric_limits<double>::max(); // Initialize to max
////
////    // --- Define Candidate Parameters ---
////    std::vector<float> candidate_deltas = { 50.0f * log_res_m, 100.0f * log_res_m }; //std::vector<float> candidate_deltas = { 1.0f * log_res_m, 5.0f * log_res_m, 10.0f * log_res_m, 20.0f * log_res_m, 50.0f * log_res_m, 100.0f * log_res_m };
////    std::vector<float> candidate_thresholds = { 1.0f, 2.0f, 5.0f, 10.0f, 20.0f };
////    std::vector<int> candidate_heuristic_radii = { 50, 100, 200 }; // Grid cells
////    std::vector<float> candidate_prune_factors = { 1.0f, 1.05f, 1.1f, 1.2f }; // Multiplier for heuristic pruning
////    std::vector<float> candidate_heuristic_weights = { 0.9f, 0.95f, 1.0f }; // Weight for heuristic in HADS cost
////
////    std::cout << "\n--- Starting HADS GPU Tuning (Max " << num_iterations << " iter/combo) ---" << std::endl;
////
////    // This is a large parameter space! Consider reducing ranges or doing staged tuning.
////    for (float delta : candidate_deltas) {
////        for (float threshold : candidate_thresholds) {
////            for (int radius : candidate_heuristic_radii) {
////                for (float prune : candidate_prune_factors) {
////                    for (float weight : candidate_heuristic_weights) {
////                        std::cout << "  Testing D=" << delta << ", T=" << threshold << ", R=" << radius << ", P=" << prune << ", W=" << weight << "..." << std::flush;
////                        double total_time_ms = 0.0;
////                        bool path_found_this_config = false;
////                        int runs_completed_ok = 0;
////
////                        for (int i = 0; i < num_iterations; ++i) {
////                            auto start_time = std::chrono::high_resolution_clock::now();
////                            std::vector<int> path;
////                            try {
////                                path = findPathGPU_HADS( // Use the actual function name
////                                    logical_grid, elevation_values, elev_width, elev_height,
////                                    log_res_m, elev_res_m, offset_x, offset_y,
////                                    start, end,
////                                    delta, threshold, radius, prune, weight); // Pass all HADS params
////                            }
////                            catch (const std::exception& e) {
////                                std::cerr << "\nException during HADS tuning run: " << e.what() << std::endl;
////                                path.clear(); // Mark as failed
////                            }
////                            auto end_time = std::chrono::high_resolution_clock::now();
////
////                            if (!path.empty()) {
////                                total_time_ms += std::chrono::duration<double, std::milli>(end_time - start_time).count();
////                                path_found_this_config = true;
////                                runs_completed_ok++;
////                            }
////                            else {
////                                path_found_this_config = false;
////                                std::cout << " [Run " << i + 1 << " FAILED] " << std::flush;
////                                break; // Stop iterations for this combo if one fails
////                            }
////                        } // End iterations loop
////
////                        if (runs_completed_ok > 0 && path_found_this_config) {
////                            double avg_time_ms = total_time_ms / runs_completed_ok;
////                            std::cout << " Avg Time: " << std::fixed << std::setprecision(3) << avg_time_ms << " ms." << std::endl;
////                            if (avg_time_ms < best_result.best_time_ms) {
////                                best_result.best_delta = delta;
////                                best_result.best_threshold = threshold;
////                                best_result.best_hads_heuristic_radius = radius;
////                                best_result.best_hads_prune_factor = prune;
////                                best_result.best_hads_heuristic_weight = weight;
////                                best_result.best_time_ms = avg_time_ms;
////                                best_result.path_found = true;
////                            }
////                        }
////                        else {
////                            std::cout << " Failed or Inconsistent." << std::endl;
////                        }
////                    } // End weight loop
////                } // End prune loop
////            } // End radius loop
////        } // End threshold loop
////    } // End delta loop
////
////    std::cout << "--- HADS Tuning Finished ---" << std::endl;
////    if (best_result.path_found) {
////        std::cout << "  Best Params: D=" << best_result.best_delta << ", T=" << best_result.best_threshold
////            << ", R=" << best_result.best_hads_heuristic_radius << ", P=" << best_result.best_hads_prune_factor
////            << ", W=" << best_result.best_hads_heuristic_weight
////            << " (Avg Time: " << best_result.best_time_ms << " ms)" << std::endl;
////    }
////    else {
////        std::cout << "  No HADS parameter configuration successfully found a path in all iterations." << std::endl;
////        // Use some default if tuning fails?
////        best_result.best_delta = 50.0f;
////        best_result.best_threshold = 10.0f;
////        best_result.best_hads_heuristic_radius = 100;
////        best_result.best_hads_prune_factor = 1.05f;
////        best_result.best_hads_heuristic_weight = 1.0f;
////        best_result.path_found = false; // Still mark as not found via tuning
////    }
////    return best_result;
////}