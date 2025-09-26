#include "io/coordinate_system_utils.hpp"
#include <gdal.h>
#include <ogr_api.h>
#include <ogr_spatialref.h>
#include <iostream>
#include <cmath>
#include <fstream>
#include <cstring>

namespace adjfind {
namespace io {

int CoordinateSystemUtils::determineUTMZone(double longitude) {
    // UTM zones are 6 degrees wide, starting from -180
    // Zone 1 starts at -180, Zone 60 ends at +180
    int zone = static_cast<int>((longitude + 180.0) / 6.0) + 1;
    
    // Ensure zone is within valid range
    if (zone < 1) zone = 1;
    if (zone > 60) zone = 60;
    
    return zone;
}

int CoordinateSystemUtils::determineUTMEPSG(double longitude, double latitude) {
    int zone = determineUTMZone(longitude);
    
    // Determine hemisphere
    if (latitude >= 0) {
        // Northern hemisphere: EPSG 326xx
        return 32600 + zone;
    } else {
        // Southern hemisphere: EPSG 327xx
        return 32700 + zone;
    }
}

bool CoordinateSystemUtils::getDatasetCenter(GDALDatasetH dataset, double& center_x, double& center_y) {
    if (!dataset) {
        return false;
    }
    
    // Get the first layer
    OGRLayerH layer = GDALDatasetGetLayer(dataset, 0);
    if (!layer) {
        return false;
    }
    
    // Get extent using OGREnvelope
    OGREnvelope extent;
    OGRErr err = OGR_L_GetExtent(layer, &extent, 1);
    if (err != OGRERR_NONE) {
        return false;
    }
    
    // Calculate center
    center_x = (extent.MinX + extent.MaxX) / 2.0;
    center_y = (extent.MinY + extent.MaxY) / 2.0;
    
    return true;
}

bool CoordinateSystemUtils::isEPSG4326(OGRSpatialReferenceH spatial_ref) {
    if (!spatial_ref) {
        return false;
    }
    
    // Check if it's WGS84
    OGRSpatialReferenceH wgs84 = OSRNewSpatialReference(nullptr);
    OSRImportFromEPSG(wgs84, 4326);
    
    bool is_same = OSRIsSame(spatial_ref, wgs84);
    
    OSRDestroySpatialReference(wgs84);
    return is_same;
}

OGRCoordinateTransformationH CoordinateSystemUtils::createUTMTransformation(OGRSpatialReferenceH source_srs, int target_epsg) {
    if (!source_srs) {
        return nullptr;
    }
    
    // Create target spatial reference
    OGRSpatialReferenceH target_srs = OSRNewSpatialReference(nullptr);
    if (OSRImportFromEPSG(target_srs, target_epsg) != OGRERR_NONE) {
        OSRDestroySpatialReference(target_srs);
        return nullptr;
    }
    
    // Set axis mapping strategy for GDAL 3+
    OSRSetAxisMappingStrategy(source_srs, OAMS_TRADITIONAL_GIS_ORDER);
    OSRSetAxisMappingStrategy(target_srs, OAMS_TRADITIONAL_GIS_ORDER);
    
    // Create coordinate transformation
    OGRCoordinateTransformationH coord_trans = OCTNewCoordinateTransformation(source_srs, target_srs);
    
    // Clean up target spatial reference
    OSRDestroySpatialReference(target_srs);
    
    return coord_trans;
}

std::string CoordinateSystemUtils::getCoordinateSystemWKT(GDALDatasetH dataset) {
    if (!dataset) {
        return "";
    }
    
    // Try to get spatial reference from the first layer (more reliable than dataset level)
    OGRLayerH layer = GDALDatasetGetLayer(dataset, 0);
    if (layer) {
        OGRSpatialReferenceH srs = OGR_L_GetSpatialRef(layer);
        if (srs) {
            char* wkt = nullptr;
            OSRExportToWkt(srs, &wkt);
            if (wkt) {
                std::string result(wkt);
                CPLFree(wkt);
                return result;
            }
            CPLFree(wkt);
        }
    }
    
    // Fallback to dataset level projection
    const char* wkt = GDALGetProjectionRef(dataset);
    return wkt ? std::string(wkt) : "";
}

std::string CoordinateSystemUtils::getCoordinateSystemWKT(GDALDatasetH dataset, int layer_index) {
    if (!dataset) {
        return "";
    }
    
    // Check if this is a GeoJSON file by examining the driver
    GDALDriverH driver = GDALGetDatasetDriver(dataset);
    const char* driver_name = GDALGetDriverShortName(driver);
    if (driver_name && strcmp(driver_name, "GeoJSON") == 0) {
        // For GeoJSON, check if the file actually contains a CRS field
        // Get the dataset description (usually contains the file path)
        const char* description = GDALGetDescription(dataset);
        if (description) {
            std::ifstream file(description);
            if (file.is_open()) {
                std::string line;
                bool has_crs = false;
                while (std::getline(file, line)) {
                    // Look for "crs" field in the JSON
                    if (line.find("\"crs\"") != std::string::npos) {
                        has_crs = true;
                        break;
                    }
                }
                file.close();
                
                if (!has_crs) {
                    std::cout << "DEBUG: GeoJSON file has no explicit CRS field - treating as no coordinate system" << std::endl;
                    return "";
                }
            }
        }
    }
    
    // Check if layer index is valid
    if (layer_index < 0 || layer_index >= GDALDatasetGetLayerCount(dataset)) {
        std::cerr << "Warning: Layer index " << layer_index << " is out of range. Dataset has " 
                  << GDALDatasetGetLayerCount(dataset) << " layers. Using layer 0." << std::endl;
        layer_index = 0;
    }
    
    // Try to get spatial reference from the specified layer
    OGRLayerH layer = GDALDatasetGetLayer(dataset, layer_index);
    if (layer) {
        OGRSpatialReferenceH srs = OGR_L_GetSpatialRef(layer);
        if (srs) {
            char* wkt = nullptr;
            OSRExportToWkt(srs, &wkt);
            if (wkt) {
                std::string result(wkt);
                CPLFree(wkt);
                return result;
            }
            CPLFree(wkt);
        }
    }
    
    // Fallback to dataset level projection
    const char* wkt = GDALGetProjectionRef(dataset);
    return wkt ? std::string(wkt) : "";
}

bool CoordinateSystemUtils::compareCoordinateSystems(OGRSpatialReferenceH spatial_ref1, 
                                                   OGRSpatialReferenceH spatial_ref2) {
    if (!spatial_ref1 || !spatial_ref2) {
        return false;
    }
    
    return OSRIsSame(spatial_ref1, spatial_ref2);
}

} // namespace io
} // namespace adjfind 