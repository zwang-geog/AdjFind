#ifndef ADJFIND_COORDINATE_SYSTEM_UTILS_HPP
#define ADJFIND_COORDINATE_SYSTEM_UTILS_HPP

#include <string>
#include <gdal.h>
#include <ogr_spatialref.h>

namespace adjfind {
namespace io {

/**
 * Coordinate system utility functions for UTM reprojection and validation
 */
class CoordinateSystemUtils {
public:
    /**
     * Determine UTM zone from longitude coordinate
     * @param longitude Longitude in degrees
     * @return UTM zone number (1-60)
     */
    static int determineUTMZone(double longitude);
    
    /**
     * Determine UTM EPSG code from longitude and latitude
     * @param longitude Longitude in degrees
     * @param latitude Latitude in degrees
     * @return EPSG code for UTM zone (326xx for northern, 327xx for southern)
     */
    static int determineUTMEPSG(double longitude, double latitude);
    
    /**
     * Get dataset extent and calculate center point
     * @param dataset GDAL dataset
     * @param center_x Output center longitude
     * @param center_y Output center latitude
     * @return true if successful, false otherwise
     */
    static bool getDatasetCenter(void* dataset, double& center_x, double& center_y);
    
    /**
     * Check if coordinate system is EPSG:4326 (WGS84)
     * @param spatial_ref OGRSpatialReference object
     * @return true if EPSG:4326, false otherwise
     */
    static bool isEPSG4326(OGRSpatialReference* spatial_ref);
    
    /**
     * Create coordinate transformation for UTM reprojection
     * @param source_srs Source spatial reference
     * @param target_epsg Target EPSG code for UTM zone
     * @return Coordinate transformation object (caller owns the pointer)
     */
    static OGRCoordinateTransformation* createUTMTransformation(OGRSpatialReference* source_srs, int target_epsg);
    
    /**
     * Get coordinate system as WKT string
     * @param dataset GDAL dataset
     * @return WKT string of coordinate system
     */
    static std::string getCoordinateSystemWKT(void* dataset);
    
    /**
     * Get coordinate system as WKT string from a specific layer
     * @param dataset GDAL dataset
     * @param layer_index Layer index to get coordinate system from
     * @return WKT string of coordinate system
     */
    static std::string getCoordinateSystemWKT(void* dataset, int layer_index);
    
    /**
     * Compare two coordinate systems
     * @param spatial_ref1 First coordinate system
     * @param spatial_ref2 Second coordinate system
     * @return true if they are the same, false otherwise
     */
    static bool compareCoordinateSystems(OGRSpatialReference* spatial_ref1, 
                                       OGRSpatialReference* spatial_ref2);

private:
    // Disable instantiation
    CoordinateSystemUtils() = delete;
};

} // namespace io
} // namespace adjfind

#endif // ADJFIND_COORDINATE_SYSTEM_UTILS_HPP 