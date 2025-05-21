// File: Fetcher.cpp

#include "Fetcher.hpp" // <<< INCLUDE HEADER FIRST >>>
                      // Brings in Common.hpp and <future>

// Include headers needed for IMPLEMENTATION
#include "ApiClient.hpp"
#include "CoordConverter.hpp"
#include <vector>
#include <cmath>
#include <thread>    // For std::async launch policy / potential thread creation
#include <stdexcept>
#include <iostream>

// <<< NO direct include of ElevationFetchingCommon.hpp needed here >>>

namespace ElevationFetching {

    // --- Forward declaration ---
    ElevationFetching::ElevationDataResult fetchElevationTask(
        MapGeoRefCalculated geo_ref_data,
        ApiConfig api_config,
        double desired_resolution_m
    );

    // --- Public async function definition ---
    std::future<ElevationFetching::ElevationDataResult> fetchElevationDataAsync(
        const MapGeoRefCalculated& geo_ref_data,
        const ApiConfig& api_config,
        double desired_resolution_m
    ) {
        return std::async(
            std::launch::async,
            fetchElevationTask,
            geo_ref_data,
            api_config,
            desired_resolution_m
        );
    }

    // --- Task function DEFINITION ---
    ElevationFetching::ElevationDataResult fetchElevationTask(
        MapGeoRefCalculated geo_ref_data,
        ApiConfig api_config,
        double desired_resolution_m
    ) {
        ElevationFetching::ElevationDataResult result;
        // ... implementation uses ApiClient, CoordConverter etc. ...
        try {
            // Example usage inside:
            ApiClient client(api_config);
            // ... generate utm points ...
            // std::vector<PointLatLon> latlon_points;
            // for(const auto& utm_pt : utm_query_points) {
            //    latlon_points.push_back(utmToLatLonApprox(utm_pt, geo_ref_data.refUTM, geo_ref_data.refLatLon));
            // }
            // result.values = client.getElevations(latlon_points);
            // ... set other result fields ...
            result.success = true;
        }
        catch (...) {
            // ... set error message ...
            result.success = false;
        }

        return result;
    }

} // namespace ElevationFetching