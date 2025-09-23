#ifndef ADJFIND_GEOJSON_READER_HPP
#define ADJFIND_GEOJSON_READER_HPP

#include <string>
#include <optional>
#include <nlohmann/json.hpp>
#include "../graph/common.hpp"

namespace adjfind {
namespace io {

/**
 * GeoJSON reader for parsing GeoJSON files into GeospatialDataset objects
 */
class GeoJSONReader {
public:
    /**
     * Read a GeoJSON file and parse it into a GeospatialDataset
     * @param filepath Path to the GeoJSON file
     * @return GeospatialDataset containing all features and CRS information
     * @throws std::runtime_error if file cannot be read or parsed
     */
    static graph::GeospatialDataset readFromFile(const std::string& filepath);
    
    /**
     * Parse a GeoJSON string and convert it into a GeospatialDataset
     * @param geojson_string JSON string containing GeoJSON data
     * @return GeospatialDataset containing all features and CRS information
     * @throws std::runtime_error if string cannot be parsed
     */
    static graph::GeospatialDataset readFromString(const std::string& geojson_string);
    
    /**
     * Parse CRS information from a GeoJSON object
     * @param geojson JSON object containing GeoJSON data
     * @return CRS string (e.g., "EPSG:32633") or empty string if not found
     */
    static std::string parseCRS(const nlohmann::json& geojson);
    
    /**
     * Get the last error message
     * @return Error message from the last operation
     */
    static std::string getLastError() {
        return last_error_;
    }

private:
    static std::string last_error_;
    
    /**
     * Parse a single feature from GeoJSON format
     * @param feature_json JSON object representing a GeoJSON feature
     * @param id Feature ID (size_t starting from 0)
     * @return Optional GeospatialFeature object (nullopt if geometry is missing)
     */
    static std::optional<graph::GeospatialFeature> parseFeature(const nlohmann::json& feature_json, size_t id);
    
    /**
     * Set error message
     * @param error Error message to set
     */
    static void setError(const std::string& error) {
        last_error_ = error;
    }
    
    // Disable instantiation
    GeoJSONReader() = delete;
};

} // namespace io
} // namespace adjfind

#endif // ADJFIND_GEOJSON_READER_HPP
