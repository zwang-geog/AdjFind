#ifndef ADJFIND_POLYGON_READER_HPP
#define ADJFIND_POLYGON_READER_HPP

#include <string>
#include <vector>
#include <memory>
#include <optional>
#include <ogrsf_frmts.h>
#include "graph/common.hpp"
#include "graph/adj_graph.hpp"
#include "io/coordinate_system_utils.hpp"

namespace adjfind {
namespace io {

// Polygon reader configuration
struct PolygonReaderConfig {
    std::string file_path;                    // Input file path
    std::string id_field;                     // Field name for polygon ID (optional, uses FID if not specified)
    std::string is_obstacle_only_field;       // Field name for obstacle-only flag (optional)
    std::string road_ids_snappable_field;     // Field name for comma-separated road IDs (optional)
    int layer_index;                          // Layer index to read (default: 0)
    double min_polygon_boundary_segment_length_for_nearest_road_edge_detection; // Minimum polygon boundary segment length for nearest road edge detection (default: 80.0)
    
    PolygonReaderConfig() : layer_index(0), min_polygon_boundary_segment_length_for_nearest_road_edge_detection(80.0) {}
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
     * Get the coordinate system WKT string
     * @return Coordinate system WKT string
     */
    std::string getCoordinateSystemWKT() const { return coordinate_system_wkt_; }
    
    /**
     * Get the coordinate system EPSG code
     * @return EPSG code if available, -1 otherwise
     */
    int getCoordinateSystemEPSG() const { return coordinate_system_epsg_; }
    
    /**
     * Get the spatial reference
     * @return Optional spatial reference
     */
    std::optional<OGRSpatialReference> getSpatialRef() const;
    
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
    std::unique_ptr<GDALDataset> dataset_;
    std::vector<graph::PolygonFeature> polygons_;
    
    // Coordinate system information
    std::string coordinate_system_wkt_;
    int coordinate_system_epsg_;
    OGRCoordinateTransformation* coordinate_transformation_;
    
    // R-tree for spatial queries
    using PolygonRTree = graph::bgi::rtree<graph::PolygonRTreeValue, graph::bgi::quadratic<16>>;
    PolygonRTree rtree_;
    
    /**
     * Initialize GDAL
     */
    void initGDAL();
    
    /**
     * Read features from the dataset
     * @return true if successful, false otherwise
     */
    bool readFeatures();
    
    /**
     * Build spatial index for polygons
     */
    void buildSpatialIndex();
    
    /**
     * Get field value as size_t
     * @param feature OGR feature
     * @param field_name Field name
     * @param default_value Default value if field not found
     * @return Field value or default
     */
    std::optional<size_t> getFieldValueAsSizeT(const OGRFeature* feature,
                                               const std::string& field_name) const;
    
    /**
     * Get field value as boolean
     * @param feature OGR feature
     * @param field_name Field name
     * @param default_value Default value if field not found
     * @return Field value or default
     */
    bool getFieldValueAsBool(const OGRFeature* feature,
                             const std::string& field_name,
                             bool default_value) const;
    
    /**
     * Get field value as string
     * @param feature OGR feature
     * @param field_name Field name
     * @return Field value or empty string
     */
    std::string getFieldValueAsString(const OGRFeature* feature,
                                     const std::string& field_name) const;
    
    /**
     * Parse comma-separated road IDs from string
     * @param road_ids_str Comma-separated string of road IDs
     * @return Vector of road IDs
     */
    std::vector<size_t> parseRoadIds(const std::string& road_ids_str) const;
    
    /**
     * Convert OGR geometry to boost geometry polygon
     * @param ogr_geom OGR geometry
     * @return Boost geometry polygon
     */
    graph::Polygon convertOGRToPolygon(const OGRGeometry* ogr_geom) const;
    
    /**
     * Transform geometry to UTM if needed
     * @param geom OGR geometry to transform
     * @return true if successful, false otherwise
     */
    bool transformGeometry(OGRGeometry* geom) const;
    
    /**
     * Handle coordinate system transformation
     * @return true if successful, false otherwise
     */
    bool handleCoordinateSystem();
    
    /**
     * Compute shrink geometry for a polygon
     * @param geometry Original polygon geometry
     * @param feature_id Feature ID for error reporting
     * @return Optional shrunk geometry
     */
    std::optional<graph::Polygon> computeShrinkGeometry(const graph::Polygon& geometry, size_t feature_id) const;
    
    /**
     * Process a single polygon geometry and add it to the polygons vector
     * @param geometry OGR polygon geometry
     * @param feature_id Feature ID
     * @param is_obstacle_only Whether this polygon is obstacle-only
     * @param snappable_road_ids Vector of snappable road IDs
     */
    void processSinglePolygon(OGRGeometry* geometry, size_t feature_id, bool is_obstacle_only, 
                             const std::vector<size_t>& snappable_road_ids);
};

} // namespace io
} // namespace adjfind

#endif // ADJFIND_POLYGON_READER_HPP
