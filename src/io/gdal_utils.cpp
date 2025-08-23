#include "io/gdal_utils.hpp"
#include <gdal.h>
#include <gdal_priv.h>
#include <algorithm>
#include <iostream>
#include <cstring>

namespace adjfind {
namespace io {


bool GDALUtils::isDriverAvailable(const std::string& driver_name) {
    GDALDriver* driver = GetGDALDriverManager()->GetDriverByName(driver_name.c_str());
    if (!driver) {
        std::cout << "WARNING: GDAL driver '" << driver_name << "' is not available. Falling back to GeoJSON format." << std::endl;
        return false;
    }
    
    // Check if the driver supports creation
    const char* creation_support = driver->GetMetadataItem(GDAL_DCAP_CREATE);
    if (!creation_support || strcmp(creation_support, "YES") != 0) {
        std::cout << "WARNING: GDAL driver '" << driver_name << "' does not support creation. Falling back to GeoJSON format." << std::endl;
        return false;
    }
    
    return true;
}

std::string GDALUtils::determineFormatAndModifyPath(std::string& file_path) {
    std::filesystem::path path(file_path);
    std::string extension = path.extension().string();
    
    // Convert to lowercase for case-insensitive comparison
    std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);
    
    // Check format and modify path if needed
    std::string format;
    bool needs_modification = false;
    
    switch (extension.length()) {
        case 4: // .shp, .gml, .kml, .csv
            if (extension == ".shp") {
                format = "ESRI Shapefile";
            } else if (extension == ".gml") {
                if (isDriverAvailable("GML")) {
                    format = "GML";
                } else {
                    format = "GeoJSON";
                    needs_modification = true;
                }
            } else if (extension == ".kml") {
                if (isDriverAvailable("KML")) {
                    format = "KML";
                } else {
                    format = "GeoJSON";
                    needs_modification = true;
                }
            } else if (extension == ".csv") {
                format = "CSV";
            } else {
                format = "GeoJSON";
                needs_modification = true;
            }
            break;
            
        case 5: // .gpkg, .json
            if (extension == ".gpkg") {
                if (isDriverAvailable("GPKG")) {
                    format = "GPKG";
                } else {
                    format = "GeoJSON";
                    needs_modification = true;
                }
            } else if (extension == ".json") {
                format = "GeoJSON";
            } else {
                format = "GeoJSON";
                needs_modification = true;
            }
            break;
            
        case 6: // .sqlite
            if (extension == ".sqlite") {
                if (isDriverAvailable("SQLite")) {
                    format = "SQLite";
                } else {
                    format = "GeoJSON";
                    needs_modification = true;
                }
            } else {
                format = "GeoJSON";
                needs_modification = true;
            }
            break;
            
        case 8: // .geojson
            if (extension == ".geojson") {
                format = "GeoJSON";
            } else {
                format = "GeoJSON";
                needs_modification = true;
            }
            break;
            
        case 10: // .geojsonseq
            if (extension == ".geojsonseq") {
                format = "GeoJSONSeq";
            } else {
                format = "GeoJSON";
                needs_modification = true;
            }
            break;
            
        case 11: // .flatgeobuf
            if (extension == ".flatgeobuf") {
                format = "FlatGeobuf";
            } else {
                format = "GeoJSON";
                needs_modification = true;
            }
            break;
            
        default:
            // Unknown extension, default to GeoJSON
            format = "GeoJSON";
            needs_modification = true;
            break;
    }
    
    // Modify file path if format fallback occurred
    if (needs_modification) {
        std::filesystem::path original_path(file_path);
        std::string new_path = original_path.replace_extension(".geojson").string();
        std::cout << "WARNING: Changing output file extension from '" << extension 
                  << "' to '.geojson' due to format fallback." << std::endl;
        std::cout << "New output file: " << new_path << std::endl;
        file_path = new_path;
    }
    
    return format;
}

} // namespace io
} // namespace adjfind 