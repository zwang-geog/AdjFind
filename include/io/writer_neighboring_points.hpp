#ifndef ADJFIND_WRITER_NEIGHBORING_POINTS_HPP
#define ADJFIND_WRITER_NEIGHBORING_POINTS_HPP

#include <string>
#include <vector>
#include <ogrsf_frmts.h>
#include "graph/common.hpp"
#include "io/gdal_utils.hpp"

namespace adjfind {
namespace graph {
    class NeighboringPoints;
}

namespace io {

/**
 * Configuration for neighboring points output writer
 */
struct NeighboringPointsWriterConfig {
    std::string output_file_path;     // Output file path with extension
    std::string crs_wkt;              // Coordinate reference system WKT (optional)
    bool reproject_to_epsg4326;       // Whether to reproject output to EPSG:4326
    
    NeighboringPointsWriterConfig() : reproject_to_epsg4326(false) {}
};

/**
 * Neighboring points output writer using GDAL
 * This class provides flexible output writing for neighboring points results
 * and is designed to be compatible with both GDAL and GDAL3.js
 */
class NeighboringPointsWriter {
public:
    NeighboringPointsWriter();
    ~NeighboringPointsWriter();
    
    // Disable copy constructor and assignment
    NeighboringPointsWriter(const NeighboringPointsWriter&) = delete;
    NeighboringPointsWriter& operator=(const NeighboringPointsWriter&) = delete;
    
    /**
     * Write neighboring points results to output files
     * @param config Writer configuration
     * @param neighboring_points_results Map of vertex pairs to shortest paths
     * @param neighboring_points_instance Reference to NeighboringPoints instance for vertex lookup
     * @return true if successful, false otherwise
     */
    bool writeNeighboringPointsResults(const NeighboringPointsWriterConfig& config, 
                                      const std::unordered_map<std::pair<size_t, size_t>, std::tuple<size_t, double, graph::MultiLineString>, graph::PairHash>& neighboring_points_results,
                                      const graph::NeighboringPoints& neighboring_points_instance);
    
    /**
     * Get the last error message
     * @return Error message string
     */
    std::string getLastError() const { return last_error_; }
    
    /**
     * Clear the last error message
     */
    void clearError() { last_error_.clear(); }
    
    /**
     * Generate the snapped points output file path
     * @param base_file_path Base file path
     * @return File path with "_snapped_points" inserted before extension
     */
    std::string generateSnappedPointsFilePath(const std::string& base_file_path) const;
    
private:
    std::string last_error_;  // Last error message
    
    /**
     * Create GDAL dataset for writing linestring features
     * @param config Writer configuration
     * @param coord_trans Output parameter for coordinate transformation (can be nullptr)
     * @param output_file_path Output parameter for the actual file path used (can be modified for format fallback)
     * @return GDAL dataset pointer (caller owns the pointer)
     */
    void* createLinestringDataset(const NeighboringPointsWriterConfig& config, OGRCoordinateTransformation*& coord_trans, std::string& output_file_path);
    
    /**
     * Create GDAL dataset for writing point features
     * @param config Writer configuration
     * @param coord_trans Output parameter for coordinate transformation (can be nullptr)
     * @param output_file_path Output parameter for the actual file path used (can be modified for format fallback)
     * @return GDAL dataset pointer (caller owns the pointer)
     */
    void* createPointDataset(const NeighboringPointsWriterConfig& config, OGRCoordinateTransformation*& coord_trans, std::string& output_file_path);
    
    /**
     * Write both linestring and point features efficiently in a single pass
     * @param linestring_dataset GDAL dataset for linestrings
     * @param linestring_layer GDAL layer for linestrings
     * @param coord_trans_linestring Coordinate transformation for linestrings (can be nullptr)
     * @param point_dataset GDAL dataset for points
     * @param point_layer GDAL layer for points
     * @param coord_trans_point Coordinate transformation for points (can be nullptr)
     * @param neighboring_points_results Map of vertex pairs to shortest paths
     * @param neighboring_points_instance Reference to NeighboringPoints instance for vertex lookup
     * @return true if successful, false otherwise
     */
    bool writeLinestringAndPointFeatures(void* linestring_dataset, void* linestring_layer, 
                                        OGRCoordinateTransformation* coord_trans_linestring,
                                        void* point_dataset, void* point_layer, 
                                        OGRCoordinateTransformation* coord_trans_point,
                                        const std::unordered_map<std::pair<size_t, size_t>, std::tuple<size_t, double, graph::MultiLineString>, graph::PairHash>& neighboring_points_results,
                                        const graph::NeighboringPoints& neighboring_points_instance);
    
    /**
     * Compute summary statistics for a point vertex
     * @param source_vertex_index Source vertex index
     * @param neighboring_points_results Map of vertex pairs to shortest paths
     * @param neighboring_points_instance Reference to NeighboringPoints instance
     * @return Tuple containing (count, min_distance, max_distance, avg_distance)
     */
    std::tuple<size_t, double, double, double> computePointStatistics(
        size_t source_vertex_index,
        const std::unordered_map<std::pair<size_t, size_t>, std::tuple<size_t, double, graph::MultiLineString>, graph::PairHash>& neighboring_points_results,
        const graph::NeighboringPoints& neighboring_points_instance) const;

private:
    /**
     * Update statistics for a vertex when processing a path
     * @param vertex_index Vertex index to update
     * @param distance Distance value to consider
     * @param count_map Reference to count map
     * @param min_map Reference to min distance map
     * @param max_map Reference to max distance map
     * @param sum_map Reference to sum distance map
     */
    void updateVertexStatistics(size_t vertex_index, double distance,
                               std::unordered_map<size_t, size_t>& count_map,
                               std::unordered_map<size_t, double>& min_map,
                               std::unordered_map<size_t, double>& max_map,
                               std::unordered_map<size_t, double>& sum_map);
};

} // namespace io
} // namespace adjfind

#endif // ADJFIND_WRITER_NEIGHBORING_POINTS_HPP
