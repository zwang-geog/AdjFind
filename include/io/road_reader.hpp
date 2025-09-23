#ifndef ADJFIND_ROAD_READER_HPP
#define ADJFIND_ROAD_READER_HPP

#include <string>
#include <vector>
#include <memory>
#include <unordered_map>
#include <optional>
#include "graph/common.hpp"
#include "io/geojson_reader.hpp"

namespace adjfind {
namespace io {

// Road reader configuration
struct RoadReaderConfig {
    std::string file_path;          // Input file path (GeoJSON)
    std::string id_field;           // Field name for road ID (optional, uses feature index if not specified)
    std::string from_z_field;       // Field name for from z-level (optional)
    std::string to_z_field;         // Field name for to z-level (optional)
    std::string length_field;       // Field name for length (optional, computed if not specified)
    double default_from_z;          // Default from z-level (default: 0.0)
    double default_to_z;            // Default to z-level (default: 0.0)
    
    RoadReaderConfig()
        : default_from_z(0.0), default_to_z(0.0) {}
};

class RoadReader {
public:
    explicit RoadReader(const RoadReaderConfig& config);
    ~RoadReader();
    
    // Disable copy constructor and assignment
    RoadReader(const RoadReader&) = delete;
    RoadReader& operator=(const RoadReader&) = delete;
    
    /**
     * Read road features from file
     * @return true if successful, false otherwise
     */
    bool read();
    
    /**
     * Get the number of road features
     * @return Number of features
     */
    size_t getFeatureCount() const { return roads_.size(); }
    
    /**
     * Get a road feature by index
     * @param index Feature index
     * @return Road feature
     */
    graph::RoadFeature& getRoadFeature(size_t index);
    
    /**
     * Get all road features
     * @return Vector of road features
     */
    const std::vector<graph::RoadFeature>& getRoadFeatures() const { return roads_; }
    

    
    /**
     * Get the coordinate system CRS string
     * @return Coordinate system CRS string (e.g., "EPSG:32633")
     */
    std::string getCoordinateSystemCRS() const { return coordinate_system_crs_; }
    
    /**
     * Find the nearest road segment to a point
     * @param query_point Query point
     * @return Optional tuple of (road_index, segment_index, snapped_point, distance_to_first_point)
     */
    std::optional<std::tuple<size_t, size_t, graph::Point, double>> findNearestRoadSegment(const graph::Point& query_point) const;
    

    
    /**
     * Calculate the closest point on a segment to a query point
     * @param query_point Query point
     * @param segment_start_point Start point of the segment
     * @param segment_end_point End point of the segment
     * @return Closest point on segment
     */
    graph::Point calculateClosestPointOnSegment(const graph::Point& query_point, const graph::Point& segment_start_point, const graph::Point& segment_end_point) const;
    
    /**
     * Clear the spatial index to free memory
     */
    void clearSpatialIndex();
    
    /**
     * Clear all road data to free memory
     */
    void clearRoads();

private:
    RoadReaderConfig config_;
    std::vector<graph::RoadFeature> roads_;
    
    // Coordinate system information
    std::string coordinate_system_crs_;
    
    // R-tree for spatial queries
    using RoadRTree = graph::bgi::rtree<graph::RoadRTreeValue, graph::bgi::quadratic<16>>;
    RoadRTree rtree_;
    
    /**
     * Read features from GeoJSON dataset
     * @param dataset GeoJSON dataset to process
     * @return true if successful, false otherwise
     */
    bool readFeatures(const graph::GeospatialDataset& dataset);
    
    /**
     * Build spatial index for roads
     */
    void buildSpatialIndex();
    
};

} // namespace io
} // namespace adjfind

#endif // ADJFIND_ROAD_READER_HPP 