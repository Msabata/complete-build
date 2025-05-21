// File: ApiClient.hpp
#ifndef API_CLIENT_HPP
#define API_CLIENT_HPP

#include "ElevationFetchingCommon.hpp" // <<< ONLY THIS NEEDED FOR DEFINITIONS HERE >>>

#include <vector>       // Needed for std::vector in method signatures
#include <string>       // Needed for std::string
#include <optional>     // Needed if checkCache returns optional
#include <mutex>        // Needed for std::mutex member

// NO httplib forward declarations or includes here

namespace ElevationFetching {

    class ApiClient {
    public:
        explicit ApiClient(const ApiConfig& config); // Uses ApiConfig
        ~ApiClient();

        // Uses PointLatLon, returns vector<float>
        std::vector<float> getElevations(const std::vector<PointLatLon>& points);

    private:
        ApiConfig config_; // Uses ApiConfig
        // std::map<std::string, std::vector<float>> cache_; // Example cache type
        std::mutex cache_mutex_; // Uses std::mutex

        // Uses PointLatLon, returns string
        std::string generateCacheKey(const std::vector<PointLatLon>& points) const;
        // Returns optional<vector<float>>
        std::optional<std::vector<float>> checkCache(const std::string& key) const;
        // Takes vector<float>
        void saveCache(const std::string& key, const std::vector<float>& elevations) const;

        // Internal helpers - Use PointLatLon, return vector<float>
        std::vector<float> fetchOpenMeteo(const std::vector<PointLatLon>& batch);
        // std::vector<float> fetchOpenElevation(const std::vector<PointLatLon>& batch);
        // std::vector<float> fetchGoogle(const std::vector<PointLatLon>& batch);
    };

} // namespace ElevationFetching
#endif // API_CLIENT_HPP