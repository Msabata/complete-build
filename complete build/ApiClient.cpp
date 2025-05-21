// File: ApiClient.cpp
// #define CPPHTTPLIB_OPENSSL_SUPPORT // Keep commented out

#include "ApiClient.hpp" // <<< INCLUDE HEADER FIRST >>>
                         // This brings in ElevationFetchingCommon.hpp

// <<< REMOVE direct include of Common.hpp if it exists here >>>
// #include "ElevationFetchingCommon.hpp" // <<< REMOVE THIS LINE >>>

#include "httplib.h"
#include "json.hpp"

// Include necessary standard library headers
#include <iostream>
#include <sstream>
#include <vector>        // Already included via ApiClient.hpp? Safe redundancy
#include <string>        // Already included via ApiClient.hpp? Safe redundancy
#include <stdexcept>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <functional>
#include <filesystem>    // For C++17 directory creation
#include <optional>      // For checkCache return


// --- Placeholder definitions if not provided elsewhere ---
// Make sure these match your actual definitions
#ifndef API_CLIENT_PLACEHOLDERS
#define API_CLIENT_PLACEHOLDERS
namespace ElevationFetching {
    enum class ApiType {
        OPEN_METEO,
        OPEN_ELEVATION,
        GOOGLE
        // Add other types if needed
    };

    struct PointLatLon {
        double lat = 0.0;
        double lon = 0.0;
    };

    struct ApiConfig {
        ApiType type = ApiType::OPEN_METEO;
        std::string cacheDirectory = "./elevation_cache/";
        int batchSize = 100; // Example batch size
        // Add other config options like API keys, specific URLs if needed
    };
} // namespace ElevationFetching
#endif
// --- End Placeholder definitions ---


namespace ElevationFetching {

    // Constructor
    ApiClient::ApiClient(ApiConfig config) : config_(std::move(config)) {
        // Ensure cache directory exists (C++17)
        try {
            if (!config_.cacheDirectory.empty()) {
                std::filesystem::path cachePath = config_.cacheDirectory;
                // Add trailing separator if missing for consistent path joining later
                if (cachePath.filename() == cachePath.string()) { // Check if it's just a name without trailing slash
                    cachePath += std::filesystem::path::preferred_separator;
                    config_.cacheDirectory = cachePath.string(); // Update config
                }

                if (!std::filesystem::exists(cachePath)) {
                    std::filesystem::create_directories(cachePath);
                    std::cout << "Info: Created cache directory: " << config_.cacheDirectory << std::endl;
                }
            }
        }
        catch (const std::filesystem::filesystem_error& e) {
            std::cerr << "Warning: Failed to create/access cache directory '" << config_.cacheDirectory << "': " << e.what() << std::endl;
            config_.cacheDirectory = ""; // Disable caching if directory fails
        }
    }

    ApiClient::~ApiClient() = default; // Default destructor likely okay for httplib

    // --- Public Get Elevations Method ---
    std::vector<float> ApiClient::getElevations(const std::vector<PointLatLon>& points) {
        if (points.empty()) {
            return {};
        }

        // --- Caching ---
        std::string cacheKey = generateCacheKey(points);
        auto cachedData = checkCache(cacheKey);
        if (cachedData) {
            std::cout << "Info: Using cached elevation data for key: " << cacheKey << std::endl;
            return *cachedData;
        }

        std::cout << "Info: Fetching elevation data from API for " << points.size() << " points..." << std::endl;

        std::vector<float> all_elevations;
        all_elevations.reserve(points.size());
        size_t total_points = points.size();
        int batch_size = std::max(1, config_.batchSize); // Ensure batch size is at least 1

        size_t num_batches = static_cast<size_t>(std::ceil(static_cast<double>(total_points) / batch_size));

        for (size_t i = 0; i < total_points; i += batch_size) {
            size_t end_index = std::min(i + batch_size, total_points);
            std::vector<PointLatLon> batch(points.begin() + i, points.begin() + end_index);

            std::cout << "Info: Processing batch " << (i / batch_size) + 1 << "/"
                << num_batches << " (" << batch.size() << " points)" << std::endl;

            try {
                std::vector<float> batch_results;
                switch (config_.type) {
                case ApiType::OPEN_METEO:
                    batch_results = fetchOpenMeteo(batch);
                    break;
                case ApiType::OPEN_ELEVATION:
                    // batch_results = fetchOpenElevation(batch); // Placeholder
                    throw std::runtime_error("Open Elevation API not yet implemented.");
                    // break; // unreachable
                case ApiType::GOOGLE:
                    // batch_results = fetchGoogle(batch); // Placeholder
                    throw std::runtime_error("Google Elevation API not yet implemented.");
                    // break; // unreachable
                default:
                    throw std::runtime_error("Unsupported API type selected.");
                }

                if (batch_results.size() != batch.size()) {
                    // Handle partial success/failure within a batch if necessary
                    std::cerr << "Warning: API returned unexpected number of results for batch "
                        << (i / batch_size) + 1 << ". Expected "
                        << batch.size() << ", got " << batch_results.size() << ". Padding with NaN." << std::endl;
                    // Pad with NaN or throw error? For now, pad.
                    batch_results.resize(batch.size(), std::numeric_limits<float>::quiet_NaN());
                }
                all_elevations.insert(all_elevations.end(), batch_results.begin(), batch_results.end());

            }
            catch (const std::exception& e) {
                std::cerr << "Error fetching batch " << (i / batch_size) + 1 << ": " << e.what() << std::endl;
                // Strategy: Fail fast or continue with NaN? For now, fail fast.
                throw; // Rethrow the exception
                // Or: Pad this batch with NaN and continue?
                // std::vector<float> error_batch(batch.size(), std::numeric_limits<float>::quiet_NaN());
                // all_elevations.insert(all_elevations.end(), error_batch.begin(), error_batch.end());
            }
            // Optional: Add delay between batches if needed for rate limiting
            // #include <thread>
            // #include <chrono>
            // std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // --- Caching ---
        saveCache(cacheKey, all_elevations);

        std::cout << "Info: Finished fetching elevation data." << std::endl;
        return all_elevations;
    }


    // --- Private Helper Implementations ---

    // Example implementation for Open-Meteo using httplib v0.20.0 and SimpleJSON
    std::vector<float> ApiClient::fetchOpenMeteo(const std::vector<PointLatLon>& batch) {
        if (batch.empty()) return {};

        // Construct URL
        std::stringstream ss_lat, ss_lon;
        // Use sufficient precision for typical Lat/Lon requirements
        ss_lat << std::fixed << std::setprecision(6);
        ss_lon << std::fixed << std::setprecision(6);

        for (size_t i = 0; i < batch.size(); ++i) {
            ss_lat << batch[i].lat << (i == batch.size() - 1 ? "" : ",");
            ss_lon << batch[i].lon << (i == batch.size() - 1 ? "" : ",");
        }

        // Use HTTPS endpoint directly
        std::string host = "api.open-meteo.com";
        std::string scheme_host_port = "https://" + host;
        std::string path = "/v1/elevation?latitude=" + ss_lat.str() + "&longitude=" + ss_lon.str();


        // Make HTTP request using cpp-httplib v0.20.0
        httplib::Client cli(scheme_host_port);
        cli.set_follow_location(true); // Allow redirects (like http -> https if we used http host)

        // Set timeouts (optional but recommended)
        cli.set_connection_timeout(10); // 10 seconds
        cli.set_read_timeout(30);       // 30 seconds
        cli.set_write_timeout(10);      // 10 seconds

        // Use Get, which returns httplib::Result
        httplib::Result res = cli.Get(path.c_str());

        // Check for connection/request errors first
        if (!res) {
            auto err = res.error(); // Get the error code enum
            throw std::runtime_error("HTTP request failed (Open-Meteo): " + httplib::to_string(err));
        }

        // Check the HTTP status code from the response
        // Note: Access response via res->status, res->body etc.
        if (res->status != 200) {
            throw std::runtime_error("API Error (Open-Meteo): Status " + std::to_string(res->status)
                + ", Body: " + (res->body.empty() ? "[empty]" : res->body));
        }

        // Parse JSON response using SimpleJSON
        std::vector<float> elevations;
        try {
            // Use the json namespace
            json::JSON json_response = json::JSON::Load(res->body);

            // Check for API-level errors reported in the JSON body
            if (json_response.hasKey("error") && json_response["error"].ToBool(true)) { // Pass 'true' to ToBool to check if key exists and is true
                std::string reason = json_response.hasKey("reason") ? json_response["reason"].ToString() : "Unknown reason";
                throw std::runtime_error("API Logic Error (Open-Meteo): " + reason);
            }

            // Check if the 'elevation' key exists and is an array
            if (!json_response.hasKey("elevation") || json_response["elevation"].JSONType() != json::JSON::Class::Array) {
                throw std::runtime_error("API Response Error (Open-Meteo): Missing or invalid 'elevation' array.");
            }

            // Iterate through the array using ArrayRange()
            auto elevation_array_wrapper = json_response["elevation"].ArrayRange();
            // Can't reserve easily as ArrayRange wrapper doesn't expose size directly
            // elevations.reserve(elevation_array_wrapper.size()); // Not possible with SimpleJSON wrapper

            for (const json::JSON& val : elevation_array_wrapper) {
                if (val.JSONType() == json::JSON::Class::Floating) {
                    elevations.push_back(static_cast<float>(val.ToFloat()));
                }
                else if (val.JSONType() == json::JSON::Class::Integral) {
                    // Handle integers if the API might return them
                    elevations.push_back(static_cast<float>(val.ToInt()));
                }
                else if (val.IsNull()) {
                    elevations.push_back(std::numeric_limits<float>::quiet_NaN()); // Handle nulls as NaN
                }
                else {
                    // Log the problematic value type?
                    std::cerr << "Warning: Unexpected type found in elevation array: " << static_cast<int>(val.JSONType()) << std::endl;
                    elevations.push_back(std::numeric_limits<float>::quiet_NaN()); // Treat unexpected as NaN
                    // Or throw:
                    // throw std::runtime_error("API Response Error (Open-Meteo): Non-numeric/non-null value found in elevation array.");
                }
            }
        }
        catch (const std::exception& e) { // Catch JSON parsing or logic errors
            throw std::runtime_error("JSON Processing Error (Open-Meteo): " + std::string(e.what()) + ". Response body: " + res->body);
        }
        catch (...) { // Catch potential errors from SimpleJSON if it throws non-std exceptions
            throw std::runtime_error("Unknown JSON Processing Error (Open-Meteo). Response body: " + res->body);
        }


        return elevations;
    }

    // --- Cache Implementation using SimpleJSON ---

    // generateCacheKey remains largely the same conceptually, but ensure filename safety.
    std::string ApiClient::generateCacheKey(const std::vector<PointLatLon>& points) const {
        // Simple key based on first/last point and count - NOT robust for all cases.
        // A proper implementation would hash all points or use bounds+resolution+count.
        // Consider using a cryptographic hash (like SHA256) of a canonical representation
        // of all points for better collision resistance and key validity.
        if (points.empty()) return "empty.json";

        std::stringstream ss;
        ss << std::fixed << std::setprecision(6) // Precision matters for cache keys
            << static_cast<int>(config_.type) << "_" // Use int representation of enum
            << points.front().lat << "_" << points.front().lon << "_"
            << points.back().lat << "_" << points.back().lon << "_"
            << points.size();

        // Optional: Hash the stringstream result for a more compact/uniform key
        // #include <functional> // Needed for std::hash
        // std::size_t hash_val = std::hash<std::string>{}(ss.str());
        // return std::to_string(hash_val) + ".json";

        // Simple filename for now, ensure it's filesystem-safe if needed.
        // Replace potentially problematic characters if the stringstream could generate them.
        std::string base_key = ss.str();
        // Basic sanitization (replace common problematic chars) - may need more robust solution
        std::replace(base_key.begin(), base_key.end(), '/', '_');
        std::replace(base_key.begin(), base_key.end(), '\\', '_');
        std::replace(base_key.begin(), base_key.end(), ':', '_');
        std::replace(base_key.begin(), base_key.end(), '*', '_');
        std::replace(base_key.begin(), base_key.end(), '?', '_');
        std::replace(base_key.begin(), base_key.end(), '"', '_');
        std::replace(base_key.begin(), base_key.end(), '<', '_');
        std::replace(base_key.begin(), base_key.end(), '>', '_');
        std::replace(base_key.begin(), base_key.end(), '|', '_');

        return base_key + ".json";
    }

    std::optional<std::vector<float>> ApiClient::checkCache(const std::string& key) const {
        if (config_.cacheDirectory.empty()) {
            // std::cout << "Debug: Caching disabled (no directory)." << std::endl;
            return std::nullopt;
        }

        std::filesystem::path cacheFilePath = std::filesystem::path(config_.cacheDirectory) / key;
        // std::cout << "Debug: Checking cache file: " << cacheFilePath << std::endl;

        if (!std::filesystem::exists(cacheFilePath)) {
            // std::cout << "Debug: Cache file not found." << std::endl;
            return std::nullopt;
        }

        std::ifstream cacheFile(cacheFilePath);
        if (cacheFile.is_open()) {
            try {
                // Read the entire file content
                std::string content((std::istreambuf_iterator<char>(cacheFile)), std::istreambuf_iterator<char>());
                cacheFile.close(); // Close the file after reading

                if (content.empty()) {
                    std::cerr << "Warning: Cache file is empty: '" << cacheFilePath << "'\n";
                    return std::nullopt; // Treat empty file as cache miss
                }

                // Parse using SimpleJSON
                json::JSON cached_json = json::JSON::Load(content);

                if (cached_json.hasKey("elevations") && cached_json["elevations"].JSONType() == json::JSON::Class::Array) {
                    std::vector<float> elevations;
                    auto arr_wrapper = cached_json["elevations"].ArrayRange();

                    // Iterate and parse floats/nulls
                    for (const auto& val : arr_wrapper) {
                        if (val.JSONType() == json::JSON::Class::Floating) {
                            elevations.push_back(static_cast<float>(val.ToFloat()));
                        }
                        else if (val.JSONType() == json::JSON::Class::Integral) {
                            elevations.push_back(static_cast<float>(val.ToInt()));
                        }
                        else if (val.IsNull()) {
                            elevations.push_back(std::numeric_limits<float>::quiet_NaN());
                        }
                        else {
                            throw std::runtime_error("Invalid non-numeric/non-null data found in cache file elevations array.");
                        }
                    }
                    // std::cout << "Debug: Cache hit, loaded " << elevations.size() << " elevations." << std::endl;
                    return elevations; // Return the cached data
                }
                else {
                    throw std::runtime_error("Invalid cache format: Missing or non-array 'elevations' key.");
                }
            }
            catch (const std::exception& e) {
                std::cerr << "Warning: Failed to read or parse cache file '" << cacheFilePath << "': " << e.what() << std::endl;
                // Consider deleting the corrupted cache file here
                try {
                    std::filesystem::remove(cacheFilePath);
                    std::cerr << "Warning: Removed corrupted cache file: '" << cacheFilePath << "'" << std::endl;
                }
                catch (const std::filesystem::filesystem_error& remove_err) {
                    std::cerr << "Warning: Failed to remove corrupted cache file '" << cacheFilePath << "': " << remove_err.what() << std::endl;
                }
            }
            catch (...) { // Catch potential errors from SimpleJSON if it throws non-std exceptions
                std::cerr << "Warning: Unknown error parsing cache file '" << cacheFilePath << "'" << std::endl;
                try {
                    std::filesystem::remove(cacheFilePath);
                    std::cerr << "Warning: Removed corrupted cache file: '" << cacheFilePath << "'" << std::endl;
                }
                catch (const std::filesystem::filesystem_error& remove_err) {
                    std::cerr << "Warning: Failed to remove corrupted cache file '" << cacheFilePath << "': " << remove_err.what() << std::endl;
                }
            }
        }
        else {
            std::cerr << "Warning: Could not open cache file for reading (but it exists): '" << cacheFilePath << "'\n";
        }
        return std::nullopt; // Not found or error reading/parsing cache
    }

    void ApiClient::saveCache(const std::string& key, const std::vector<float>& elevations) const {
        if (config_.cacheDirectory.empty()) return; // Caching disabled

        std::filesystem::path cacheFilePath = std::filesystem::path(config_.cacheDirectory) / key;
        // std::cout << "Debug: Saving cache to file: " << cacheFilePath << std::endl;

        std::ofstream cacheFile(cacheFilePath);
        if (cacheFile.is_open()) {
            try {
                // Build JSON using SimpleJSON
                json::JSON root = json::JSON::Make(json::JSON::Class::Object);
                json::JSON arr = json::JSON::Make(json::JSON::Class::Array);

                for (float elev : elevations) {
                    if (std::isnan(elev)) {
                        arr.append(nullptr); // SimpleJSON append for null
                    }
                    else {
                        // Store as double for potentially better precision in JSON standard,
                        // although float might suffice depending on requirements.
                        arr.append(static_cast<double>(elev));
                    }
                }
                root["elevations"] = std::move(arr); // Assign the array to the root object

                // Dump the JSON to the file stream
                cacheFile << root.dump(); // Use dump() method
                cacheFile.close(); // Ensure file is closed and flushed
                // std::cout << "Debug: Cache saved successfully." << std::endl;
            }
            catch (const std::exception& e) {
                std::cerr << "Warning: Failed to build or write JSON cache file '" << cacheFilePath << "': " << e.what() << std::endl;
            }
            catch (...) { // Catch potential errors from SimpleJSON if it throws non-std exceptions
                std::cerr << "Warning: Unknown error building or writing JSON cache file '" << cacheFilePath << "'" << std::endl;
            }
        }
        else {
            std::cerr << "Warning: Could not open cache file for writing: '" << cacheFilePath << "'\n";
        }
    }


} // namespace ElevationFetching