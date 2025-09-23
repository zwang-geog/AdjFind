#ifndef ADJFIND_WRITER_CONVEX_PATH_HPP
#define ADJFIND_WRITER_CONVEX_PATH_HPP

#include <string>
#include <vector>
#include "graph/common.hpp"
#include "io/geojson_writer.hpp"

namespace adjfind {
namespace io {

/**
 * Configuration for convex path output writer
 */
struct ConvexPathWriterConfig {
    std::string linestring_output_file_path;  // Output file path for linestring features (GeoJSON)
    std::string point_output_file_path;       // Output file path for point features (GeoJSON)
    std::string crs;                          // Coordinate reference system (e.g., "EPSG:32633")
    
    ConvexPathWriterConfig() = default;
};

/**
 * Convex path output writer using GeoJSON
 * This class provides flexible output writing for convex path results
 * using the GeoJSON format
 */
class ConvexPathWriter {
public:
    ConvexPathWriter() = default;
    ~ConvexPathWriter() = default;
    
    // Disable copy constructor and assignment
    ConvexPathWriter(const ConvexPathWriter&) = delete;
    ConvexPathWriter& operator=(const ConvexPathWriter&) = delete;
    
    /**
     * Write convex path results to output files
     * @param config Writer configuration
     * @param polygon_results Vector of polygon processing results
     * @return true if successful, false otherwise
     */
    bool writeConvexPathResults(const ConvexPathWriterConfig& config, 
                               const std::vector<std::tuple<std::vector<graph::ConvexPathResult>, size_t, graph::Point>>& polygon_results);
    
    /**
     * Get the last error message
     * @return Error message string
     */
    std::string getLastError() const { return last_error_; }
    
    /**
     * Clear the last error message
     */
    void clearError() { last_error_.clear(); }
    
private:
    std::string last_error_;  // Last error message
    
    /**
     * Convert convex path results to GeoJSON linestring features
     * @param vertex_paths Vector of convex path results
     * @param polygon_feature_id Polygon feature ID
     * @param least_accessible_point Least accessible point
     * @param start_feature_id Starting feature ID
     * @return Vector of GeoJSON features
     */
    std::vector<graph::GeospatialFeature> convexPathResultsToLinestringFeatures(
        const std::vector<graph::ConvexPathResult>& vertex_paths, 
        size_t polygon_feature_id,
        const graph::Point& least_accessible_point,
        size_t start_feature_id);
    
    /**
     * Convert least accessible point to GeoJSON point feature
     * @param vertex_paths Vector of convex path results
     * @param polygon_feature_id Polygon feature ID
     * @param least_accessible_point Least accessible point
     * @param feature_id Feature ID
     * @return GeoJSON feature
     */
    graph::GeospatialFeature leastAccessiblePointToGeoJSONFeature(
        const std::vector<graph::ConvexPathResult>& vertex_paths,
        size_t polygon_feature_id,
        const graph::Point& least_accessible_point,
        size_t feature_id);
    
    /**
     * Convert ConvexPathResultType enum to string
     * @param path_type The path type enum value
     * @return String representation of the path type
     */
    std::string pathTypeToString(graph::ConvexPathResultType path_type);
};

} // namespace io
} // namespace adjfind

#endif // ADJFIND_WRITER_CONVEX_PATH_HPP
