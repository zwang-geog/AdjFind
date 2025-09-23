#ifndef ADJFIND_WRITER_ROAD_SEGMENTATION_HPP
#define ADJFIND_WRITER_ROAD_SEGMENTATION_HPP

#include <string>
#include <vector>
#include "graph/common.hpp"
#include "io/geojson_writer.hpp"

namespace adjfind {
namespace io {

/**
 * Configuration for road segmentation output writer
 */
struct RoadSegmentationWriterConfig {
    std::string output_file_path;     // Output file path (GeoJSON)
    std::string crs;                  // Coordinate reference system (e.g., "EPSG:32633")
    
    RoadSegmentationWriterConfig() = default;
};

/**
 * Road segmentation output writer using GeoJSON
 * This class provides flexible output writing for road segmentation results
 * using the GeoJSON format
 */
class RoadSegmentationWriter {
public:
    RoadSegmentationWriter() = default;
    ~RoadSegmentationWriter() = default;
    
    // Disable copy constructor and assignment
    RoadSegmentationWriter(const RoadSegmentationWriter&) = delete;
    RoadSegmentationWriter& operator=(const RoadSegmentationWriter&) = delete;
    
    /**
     * Write road segmentation results to output file
     * @param config Writer configuration
     * @param results Vector of road segmentation output results
     * @return true if successful, false otherwise
     */
    bool writeRoadSegmentationResults(const RoadSegmentationWriterConfig& config, 
                                    const std::vector<graph::RoadSplitByDistanceBracketsOutput>& results);
    
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
     * Convert a road segmentation result to GeoJSON feature
     * @param result Road segmentation result
     * @param feature_id Feature ID
     * @return GeoJSON feature
     */
    graph::GeospatialFeature resultToGeoJSONFeature(const graph::RoadSplitByDistanceBracketsOutput& result, 
                                                   size_t feature_id);
};

} // namespace io
} // namespace adjfind

#endif // ADJFIND_WRITER_ROAD_SEGMENTATION_HPP 