#ifndef ADJFIND_GEOJSON_WRITER_HPP
#define ADJFIND_GEOJSON_WRITER_HPP

#include <string>
#include <nlohmann/json.hpp>
#include "../graph/common.hpp"

namespace adjfind {
namespace io {

/**
 * GeoJSON writer for converting GeospatialDataset objects to GeoJSON files
 */
class GeoJSONWriter {
public:
    /**
     * Write a GeospatialDataset to a GeoJSON file
     * @param dataset Dataset to write
     * @param filepath Path to the output GeoJSON file
     * @return true if successful, false otherwise
     */
    static bool writeToFile(const graph::GeospatialDataset& dataset, const std::string& filepath);
    
    /**
     * Convert a GeospatialDataset to a GeoJSON string
     * @param dataset Dataset to convert
     * @return GeoJSON string representation
     */
    static std::string writeToString(const graph::GeospatialDataset& dataset);
    
    /**
     * Set CRS information in a GeoJSON object
     * @param geojson JSON object to modify
     * @param crs CRS string (e.g., "EPSG:32633")
     */
    static void setCRS(nlohmann::json& geojson, const std::string& crs);
    
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
     * Convert a GeospatialFeature to GeoJSON format
     * @param feature Feature to convert
     * @return JSON object representing the feature
     */
    static nlohmann::json featureToGeoJSON(const graph::GeospatialFeature& feature);
    
    /**
     * Set error message
     * @param error Error message to set
     */
    static void setError(const std::string& error) {
        last_error_ = error;
    }
    
    // Disable instantiation
    GeoJSONWriter() = delete;
};

} // namespace io
} // namespace adjfind

#endif // ADJFIND_GEOJSON_WRITER_HPP
