#ifndef ADJFIND_WRITER_NEIGHBORING_POINTS_HPP
#define ADJFIND_WRITER_NEIGHBORING_POINTS_HPP

#include <string>
#include <vector>
#include "graph/common.hpp"
#include "io/geojson_writer.hpp"

namespace adjfind {
namespace graph {
    class NeighboringPoints;
}

namespace io {

/**
 * Configuration for neighboring points output writer
 */
struct NeighboringPointsWriterConfig {
    std::string linestring_output_file_path;  // Output file path for linestring features (GeoJSON)
    std::string point_output_file_path;       // Output file path for point features (GeoJSON)
    std::string crs;                          // Coordinate reference system (e.g., "EPSG:32633")
    
    NeighboringPointsWriterConfig() = default;
};

/**
 * Neighboring points output writer using GeoJSON
 * This class provides flexible output writing for neighboring points results
 * using the GeoJSON format
 */
class NeighboringPointsWriter {
public:
    NeighboringPointsWriter() = default;
    ~NeighboringPointsWriter() = default;
    
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
    
private:
    std::string last_error_;  // Last error message
    
    /**
     * Convert neighboring points results to GeoJSON linestring features
     * @param neighboring_points_results Map of vertex pairs to shortest paths
     * @param neighboring_points_instance Reference to NeighboringPoints instance for vertex lookup
     * @return Vector of GeoJSON linestring features
     */
    std::vector<graph::GeospatialFeature> neighboringPointsResultsToLinestringFeatures(
        const std::unordered_map<std::pair<size_t, size_t>, std::tuple<size_t, double, graph::MultiLineString>, graph::PairHash>& neighboring_points_results,
        const graph::NeighboringPoints& neighboring_points_instance);
    
    /**
     * Convert neighboring points results to GeoJSON point features
     * @param neighboring_points_results Map of vertex pairs to shortest paths
     * @param neighboring_points_instance Reference to NeighboringPoints instance for vertex lookup
     * @return Vector of GeoJSON point features
     */
    std::vector<graph::GeospatialFeature> neighboringPointsResultsToPointFeatures(
        const std::unordered_map<std::pair<size_t, size_t>, std::tuple<size_t, double, graph::MultiLineString>, graph::PairHash>& neighboring_points_results,
        const graph::NeighboringPoints& neighboring_points_instance);
    
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
