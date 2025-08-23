#ifndef ADJFIND_ROAD_READER_HPP
#define ADJFIND_ROAD_READER_HPP

#include <string>
#include <vector>
#include <memory>
#include <unordered_map>
#include <ogrsf_frmts.h>
#include "graph/common.hpp"
#include "io/coordinate_system_utils.hpp"

namespace adjfind {
namespace io {

// Road reader configuration
struct RoadReaderConfig {
    std::string file_path;          // Input file path
    std::string id_field;           // Field name for road ID (optional, uses FID if not specified)
    std::string from_z_field;       // Field name for from z-level (optional)
    std::string to_z_field;         // Field name for to z-level (optional)
    std::string length_field;       // Field name for length (optional, computed if not specified)
    double default_from_z;          // Default from z-level (default: 0.0)
    double default_to_z;            // Default to z-level (default: 0.0)
    int layer_index;                // Layer index to read (default: 0)
    
    RoadReaderConfig()
        : default_from_z(0.0), default_to_z(0.0), layer_index(0) {}
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
    std::unique_ptr<GDALDataset> dataset_;
    std::vector<graph::RoadFeature> roads_;
    
    // Coordinate system information
    std::string coordinate_system_wkt_;
    int coordinate_system_epsg_;
    OGRCoordinateTransformation* coordinate_transformation_;
    
    // R-tree for spatial queries
    using RoadRTree = graph::bgi::rtree<graph::RoadRTreeValue, graph::bgi::quadratic<16>>;
    RoadRTree rtree_;
    
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
     * Build spatial index for roads
     */
    void buildSpatialIndex();
    
    /**
     * Get field value as size_t
     * @param feature OGR Feature
     * @param field_name Field name
     * @return Field value if exists and can be converted
     */
    std::optional<size_t> getFieldValueAsSizeT(const OGRFeature* feature,
                                              const std::string& field_name) const;
    
    /**
     * Get field value as double with default
     * @param feature OGR Feature
     * @param field_name Field name
     * @param default_value Default value
     * @return Field value or default
     */
    double getFieldValueAsDouble(const OGRFeature* feature,
                               const std::string& field_name,
                               double default_value) const;
    
    /**
     * Convert OGR geometry to Boost Geometry LineString
     * @param ogr_geom OGR geometry
     * @return Boost Geometry LineString
     */
    graph::LineString convertOGRToLineString(const OGRGeometry* ogr_geom) const;
    
    /**
     * Transform geometry coordinates if transformation is available
     * @param geom OGR geometry to transform
     * @return true if transformation was successful or not needed
     */
    bool transformGeometry(OGRGeometry* geom) const;
    
    /**
     * Handle coordinate system detection and reprojection
     * @return true if successful, false otherwise
     */
    bool handleCoordinateSystem();
    
};

} // namespace io
} // namespace adjfind

#endif // ADJFIND_ROAD_READER_HPP 