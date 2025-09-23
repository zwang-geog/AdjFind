#ifndef ADJFIND_POINT_READER_HPP
#define ADJFIND_POINT_READER_HPP

#include <string>
#include <vector>
#include <memory>
#include <unordered_map>
#include <optional>
#include "graph/common.hpp"
#include "io/geojson_reader.hpp"

namespace adjfind {
namespace io {

// Point reader configuration
struct PointReaderConfig {
    std::string file_path;          // Input file path (GeoJSON)
    std::string id_field;           // Field name for point ID (optional, uses feature index if not specified)
    
    PointReaderConfig() = default;
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
     * Get the coordinate system CRS string
     * @return Coordinate system CRS string (e.g., "EPSG:32633")
     */
    std::string getCoordinateSystemCRS() const { return coordinate_system_crs_; }
    
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
    std::vector<graph::PointFeature> points_;
    
    // Coordinate system information
    std::string coordinate_system_crs_;
    
    // R-tree for spatial queries
    using PointRTree = graph::bgi::rtree<graph::PointRTreeValue, graph::bgi::quadratic<16>>;
    PointRTree rtree_;
    
    /**
     * Read features from GeoJSON dataset
     * @param dataset GeoJSON dataset to process
     * @return true if successful, false otherwise
     */
    bool readFeatures(const graph::GeospatialDataset& dataset);
    
    /**
     * Build spatial index for points
     */
    void buildSpatialIndex();
    
};

} // namespace io
} // namespace adjfind

#endif // ADJFIND_POINT_READER_HPP 