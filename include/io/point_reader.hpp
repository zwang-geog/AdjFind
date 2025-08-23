#ifndef ADJFIND_POINT_READER_HPP
#define ADJFIND_POINT_READER_HPP

#include <string>
#include <vector>
#include <memory>
#include <unordered_map>
#include <ogrsf_frmts.h>
#include "graph/common.hpp"
#include "io/coordinate_system_utils.hpp"

namespace adjfind {
namespace io {

// Point reader configuration
struct PointReaderConfig {
    std::string file_path;          // Input file path
    std::string id_field;           // Field name for point ID (optional, uses FID if not specified)
    int layer_index;                // Layer index to read (default: 0)
    
    PointReaderConfig() : layer_index(0) {}
};

class PointReader {
public:
    explicit PointReader(const PointReaderConfig& config);
    ~PointReader();
    
    // Disable copy constructor and assignment
    PointReader(const PointReader&) = delete;
    PointReader& operator=(const PointReader&) = delete;
    
    /**
     * Read point features from file
     * @return true if successful, false otherwise
     */
    bool read();
    
    /**
     * Get the number of point features
     * @return Number of features
     */
    size_t getFeatureCount() const { return points_.size(); }
    
    /**
     * Get a point feature by index
     * @param index Feature index
     * @return Point feature
     */
    const graph::PointFeature& getPointFeature(size_t index) const;
    
    /**
     * Get all point features
     * @return Vector of point features
     */
    const std::vector<graph::PointFeature>& getPointFeatures() const { return points_; }
    
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
     * Clear the spatial index to free memory
     */
    void clearSpatialIndex();
    
    /**
     * Clear all point data to free memory
     */
    void clearPoints();

private:
    PointReaderConfig config_;
    std::unique_ptr<GDALDataset> dataset_;
    std::vector<graph::PointFeature> points_;
    
    // Coordinate system information
    std::string coordinate_system_wkt_;
    int coordinate_system_epsg_;
    OGRCoordinateTransformation* coordinate_transformation_;
    
    // R-tree for spatial queries
    using PointRTree = graph::bgi::rtree<graph::PointRTreeValue, graph::bgi::quadratic<16>>;
    PointRTree rtree_;
    
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
     * Build spatial index for points
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
     * Convert OGR geometry to Boost Geometry Point
     * @param ogr_geom OGR geometry
     * @return Boost Geometry Point
     */
    graph::Point convertOGRToPoint(const OGRGeometry* ogr_geom) const;
    
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

#endif // ADJFIND_POINT_READER_HPP 