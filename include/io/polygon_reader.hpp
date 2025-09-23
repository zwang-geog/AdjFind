#ifndef ADJFIND_POLYGON_READER_HPP
#define ADJFIND_POLYGON_READER_HPP

#include <string>
#include <vector>
#include <memory>
#include <optional>
#include "graph/common.hpp"
#include "graph/adj_graph.hpp"
#include "io/geojson_reader.hpp"

namespace adjfind {
namespace io {

// Polygon reader configuration
struct PolygonReaderConfig {
    std::string file_path;                    // Input file path (GeoJSON)
    std::string id_field;                     // Field name for polygon ID (optional, uses feature index if not specified)
    std::string is_obstacle_only_field;       // Field name for obstacle-only flag (optional)
    std::string road_ids_snappable_field;     // Field name for comma-separated road IDs (optional)
    double min_polygon_boundary_segment_length_for_nearest_road_edge_detection; // Minimum polygon boundary segment length for nearest road edge detection (default: 80.0)
    
    PolygonReaderConfig() : min_polygon_boundary_segment_length_for_nearest_road_edge_detection(80.0) {}
};

class PolygonReader {
public:
    explicit PolygonReader(const PolygonReaderConfig& config);
    ~PolygonReader();
    
    // Disable copy constructor and assignment
    PolygonReader(const PolygonReader&) = delete;
    PolygonReader& operator=(const PolygonReader&) = delete;
    
    /**
     * Read polygon features from file
     * @return true if successful, false otherwise
     */
    bool read();
    
    /**
     * Get the number of polygon features
     * @return Number of features
     */
    size_t getFeatureCount() const { return polygons_.size(); }
    
    /**
     * Get a polygon feature by index
     * @param index Feature index
     * @return Polygon feature
     */
    const graph::PolygonFeature& getPolygonFeature(size_t index) const;
    
    /**
     * Get all polygon features
     * @return Vector of polygon features
     */
    const std::vector<graph::PolygonFeature>& getPolygonFeatures() const { return polygons_; }
    
    /**
     * Get the coordinate system CRS string
     * @return Coordinate system CRS string (e.g., "EPSG:32633")
     */
    std::string getCoordinateSystemCRS() const { return coordinate_system_crs_; }
    
    /**
     * Get the minimum polygon boundary segment length for nearest road edge detection
     * @return Minimum segment length value
     */
    double getMinPolygonBoundarySegmentLengthForNearestRoadEdgeDetection() const { 
        return config_.min_polygon_boundary_segment_length_for_nearest_road_edge_detection; 
    }
    
    /**
     * Populate snappable road IDs for polygons that don't have them
     * @param adj_graph Reference to AdjGraph for finding nearest edges
     */
    void populateSnappableRoadIds(graph::AdjGraph& adj_graph);
    
    /**
     * Query the polygon R-tree to find polygons that intersect with a line segment
     * @param segment Line segment to query against
     * @return Vector of intersecting polygon features
     */
    std::vector<graph::PolygonFeature> queryPolygonRTree(const graph::LineString& segment) const;
    
    /**
     * Clear the spatial index to free memory
     */
    void clearSpatialIndex();
    
    /**
     * Clear all polygon data to free memory
     */
    void clearPolygons();

private:
    PolygonReaderConfig config_;
    std::vector<graph::PolygonFeature> polygons_;
    
    // Coordinate system information
    std::string coordinate_system_crs_;
    
    // R-tree for spatial queries
    using PolygonRTree = graph::bgi::rtree<graph::PolygonRTreeValue, graph::bgi::quadratic<16>>;
    PolygonRTree rtree_;
    
    /**
     * Read features from GeoJSON dataset
     * @param dataset GeoJSON dataset to process
     * @return true if successful, false otherwise
     */
    bool readFeatures(const graph::GeospatialDataset& dataset);
    
    /**
     * Build spatial index for polygons
     */
    void buildSpatialIndex();
    
    /**
     * Parse comma-separated road IDs from string
     * @param road_ids_str Comma-separated string of road IDs
     * @return Vector of road IDs
     */
    std::vector<size_t> parseRoadIds(const std::string& road_ids_str) const;
    
    /**
     * Compute shrink geometry for a polygon
     * @param geometry Original polygon geometry
     * @param feature_id Feature ID for error reporting
     * @return Optional shrunk geometry
     */
    std::optional<graph::Polygon> computeShrinkGeometry(const graph::Polygon& geometry, size_t feature_id) const;
    
    /**
     * Process a single polygon geometry and add it to the polygons vector
     * @param geometry Boost polygon geometry
     * @param feature_id Feature ID
     * @param is_obstacle_only Whether this polygon is obstacle-only
     * @param snappable_road_ids Vector of snappable road IDs
     */
    void processSinglePolygon(const graph::Polygon& geometry, size_t feature_id, bool is_obstacle_only, 
                             const std::vector<size_t>& snappable_road_ids);
    
};

} // namespace io
} // namespace adjfind

#endif // ADJFIND_POLYGON_READER_HPP
