#include "io/coordinate_system_utils.hpp"
#include <ogrsf_frmts.h>
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

bool CoordinateSystemUtils::getDatasetCenter(void* dataset_ptr, double& center_x, double& center_y) {
    GDALDataset* dataset = static_cast<GDALDataset*>(dataset_ptr);
    if (!dataset) {
        return false;
    }
    
    // Get the first layer
    OGRLayer* layer = dataset->GetLayer(0);
    if (!layer) {
        return false;
    }
    
    // Get extent using OGREnvelope
    OGREnvelope extent;
    OGRErr err = layer->GetExtent(&extent);
    if (err != OGRERR_NONE) {
        return false;
    }
    
    // Calculate center
    center_x = (extent.MinX + extent.MaxX) / 2.0;
    center_y = (extent.MinY + extent.MaxY) / 2.0;
    
    return true;
}

bool CoordinateSystemUtils::isEPSG4326(OGRSpatialReference* spatial_ref) {
    if (!spatial_ref) {
        return false;
    }
    
    // Check if it's WGS84
    OGRSpatialReference wgs84;
    wgs84.importFromEPSG(4326);
    
    return spatial_ref->IsSame(&wgs84);
}

OGRCoordinateTransformation* CoordinateSystemUtils::createUTMTransformation(OGRSpatialReference* source_srs, int target_epsg) {
    if (!source_srs) {
        return nullptr;
    }
    
    // Create target spatial reference
    OGRSpatialReference target_srs;
    if (target_srs.importFromEPSG(target_epsg) != OGRERR_NONE) {
        return nullptr;
    }
    
    // Set axis mapping strategy for GDAL 3+
    source_srs->SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
    target_srs.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
    
    // Create coordinate transformation
    OGRCoordinateTransformation* coord_trans = OGRCreateCoordinateTransformation(source_srs, &target_srs);
    if (!coord_trans) {
        return nullptr;
    }
    
    return coord_trans;
}

std::string CoordinateSystemUtils::getCoordinateSystemWKT(void* dataset_ptr) {
    GDALDataset* dataset = static_cast<GDALDataset*>(dataset_ptr);
    if (!dataset) {
        return "";
    }
    
    // Try to get spatial reference from the first layer (more reliable than dataset level)
    OGRLayer* layer = dataset->GetLayer(0);
    if (layer) {
        OGRSpatialReference* srs = layer->GetSpatialRef();
        if (srs) {
            char* wkt = nullptr;
            srs->exportToWkt(&wkt);
            if (wkt) {
                std::string result(wkt);
                CPLFree(wkt);
                return result;
            }
            CPLFree(wkt);
        }
    }
    
    // Fallback to dataset level projection
    const char* wkt = dataset->GetProjectionRef();
    return wkt ? std::string(wkt) : "";
}

std::string CoordinateSystemUtils::getCoordinateSystemWKT(void* dataset_ptr, int layer_index) {
    GDALDataset* dataset = static_cast<GDALDataset*>(dataset_ptr);
    if (!dataset) {
        return "";
    }
    
    // Check if this is a GeoJSON file by examining the driver
    const char* driver_name = dataset->GetDriverName();
    if (driver_name && strcmp(driver_name, "GeoJSON") == 0) {
        // For GeoJSON, check if the file actually contains a CRS field
        // Get the dataset description (usually contains the file path)
        const char* description = dataset->GetDescription();
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
    if (layer_index < 0 || layer_index >= dataset->GetLayerCount()) {
        std::cerr << "Warning: Layer index " << layer_index << " is out of range. Dataset has " 
                  << dataset->GetLayerCount() << " layers. Using layer 0." << std::endl;
        layer_index = 0;
    }
    
    // Try to get spatial reference from the specified layer
    OGRLayer* layer = dataset->GetLayer(layer_index);
    if (layer) {
        OGRSpatialReference* srs = layer->GetSpatialRef();
        if (srs) {
            char* wkt = nullptr;
            srs->exportToWkt(&wkt);
            if (wkt) {
                std::string result(wkt);
                CPLFree(wkt);
                return result;
            }
            CPLFree(wkt);
        }
    }
    
    // Fallback to dataset level projection
    const char* wkt = dataset->GetProjectionRef();
    return wkt ? std::string(wkt) : "";
}

bool CoordinateSystemUtils::compareCoordinateSystems(OGRSpatialReference* spatial_ref1, 
                                                   OGRSpatialReference* spatial_ref2) {
    if (!spatial_ref1 || !spatial_ref2) {
        return false;
    }
    
    return spatial_ref1->IsSame(spatial_ref2);
}

} // namespace io
} // namespace adjfind 