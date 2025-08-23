#ifndef ADJFIND_WRITER_ROAD_SEGMENTATION_HPP
#define ADJFIND_WRITER_ROAD_SEGMENTATION_HPP

#include <string>
#include <vector>
#include <ogrsf_frmts.h>
#include "graph/common.hpp"
#include "io/gdal_utils.hpp"

namespace adjfind {
namespace io {

/**
 * Configuration for road segmentation output writer
 */
struct RoadSegmentationWriterConfig {
    std::string output_file_path;     // Output file path with extension
    std::string crs_wkt;              // Coordinate reference system WKT (optional)
    bool reproject_to_epsg4326;       // Whether to reproject output to EPSG:4326
    
    RoadSegmentationWriterConfig() : reproject_to_epsg4326(false) {}
};

/**
 * Road segmentation output writer using GDAL
 * This class provides flexible output writing for road segmentation results
 * and is designed to be compatible with both GDAL and GDAL3.js
 */
class RoadSegmentationWriter {
public:
    RoadSegmentationWriter();
    ~RoadSegmentationWriter();
    
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
    
    // Removed determineFormatFromExtension - now using GDALUtils::determineFormatFromExtension
    
    /**
     * Create GDAL dataset for writing
     * @param config Writer configuration
     * @param coord_trans Output parameter for coordinate transformation (can be nullptr)
     * @param output_file_path Output parameter for the actual file path used (can be modified for format fallback)
     * @return GDAL dataset pointer (caller owns the pointer)
     */
    void* createGDALDataset(const RoadSegmentationWriterConfig& config, OGRCoordinateTransformation*& coord_trans, std::string& output_file_path);
    
    /**
     * Write a single road segment feature
     * @param dataset GDAL dataset
     * @param layer GDAL layer
     * @param coord_trans Coordinate transformation (can be nullptr)
     * @param result Road segmentation result
     * @param feature_id Feature ID
     * @return true if successful, false otherwise
     */
    bool writeFeature(void* dataset, void* layer, 
                     OGRCoordinateTransformation* coord_trans,
                     const graph::RoadSplitByDistanceBracketsOutput& result, 
                     size_t feature_id);
};

} // namespace io
} // namespace adjfind

#endif // ADJFIND_WRITER_ROAD_SEGMENTATION_HPP 