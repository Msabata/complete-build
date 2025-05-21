// File: Fetcher.hpp
#ifndef FETCHER_HPP
#define FETCHER_HPP

#include "ElevationFetchingCommon.hpp" // <<< INCLUDE COMMON FIRST >>>
#include <future>                      // Include future header

// Forward declare only structs used in the function signature
// These are defined in ElevationFetchingCommon.hpp which is included above
namespace ElevationFetching {
    struct MapGeoRefCalculated;
    struct ApiConfig;
    struct ElevationDataResult;
}

namespace ElevationFetching {

    std::future<ElevationFetching::ElevationDataResult> fetchElevationDataAsync(
        const MapGeoRefCalculated& geo_ref_data,
        const ApiConfig& api_config,
        double desired_resolution_m
    );

} // namespace ElevationFetching
#endif // FETCHER_HPP