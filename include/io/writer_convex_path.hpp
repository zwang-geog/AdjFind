#ifndef ADJFIND_WRITER_CONVEX_PATH_HPP
#define ADJFIND_WRITER_CONVEX_PATH_HPP

#include <string>
#include <vector>
#include <ogrsf_frmts.h>
#include "graph/common.hpp"
#include "io/gdal_utils.hpp"

namespace adjfind {
namespace io {

/**
 * Configuration for convex path output writer
 */
struct ConvexPathWriterConfig {
    std::string output_file_path;     // Output file path with extension
    std::string crs_wkt;              // Coordinate reference system WKT (optional)
    bool reproject_to_epsg4326;       // Whether to reproject output to EPSG:4326
    
    ConvexPathWriterConfig() : reproject_to_epsg4326(false) {}
};

/**
 * Convex path output writer using GDAL
 * This class provides flexible output writing for convex path results
 * and is designed to be compatible with both GDAL and GDAL3.js
 */
class ConvexPathWriter {
public:
    ConvexPathWriter();
    ~ConvexPathWriter();
    
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
     * Create GDAL dataset for writing linestring features
     * @param config Writer configuration
     * @param coord_trans Output parameter for coordinate transformation (can be nullptr)
     * @param output_file_path Output parameter for the actual file path used (can be modified for format fallback)
     * @return GDAL dataset pointer (caller owns the pointer)
     */
    void* createLinestringDataset(const ConvexPathWriterConfig& config, OGRCoordinateTransformation*& coord_trans, std::string& output_file_path);
    
    /**
     * Create GDAL dataset for writing point features
     * @param config Writer configuration
     * @param coord_trans Output parameter for coordinate transformation (can be nullptr)
     * @param output_file_path Output parameter for the actual file path used (can be modified for format fallback)
     * @return GDAL dataset pointer (caller owns the pointer)
     */
    void* createPointDataset(const ConvexPathWriterConfig& config, OGRCoordinateTransformation*& coord_trans, std::string& output_file_path);
    
    /**
     * Write a single linestring feature
     * @param dataset GDAL dataset
     * @param layer GDAL layer
     * @param coord_trans Coordinate transformation (can be nullptr)
     * @param vertex_paths Vector of convex path results
     * @param polygon_feature_id Polygon feature ID
     * @param least_accessible_point Least accessible point
     * @param feature_id Feature ID
     * @return true if successful, false otherwise
     */
    bool writeLinestringFeature(void* dataset, void* layer, 
                               OGRCoordinateTransformation* coord_trans,
                               const std::vector<graph::ConvexPathResult>& vertex_paths, 
                               size_t polygon_feature_id,
                               const graph::Point& least_accessible_point,
                               size_t feature_id);
    
    /**
     * Write a single point feature
     * @param dataset GDAL dataset
     * @param layer GDAL layer
     * @param coord_trans Coordinate transformation (can be nullptr)
     * @param vertex_paths Vector of convex path results
     * @param polygon_feature_id Polygon feature ID
     * @param least_accessible_point Least accessible point
     * @param feature_id Feature ID
     * @return true if successful, false otherwise
     */
    bool writePointFeature(void* dataset, void* layer, 
                          OGRCoordinateTransformation* coord_trans,
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
