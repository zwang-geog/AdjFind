#include "io/writer_road_segmentation.hpp"
#include <gdal.h>
#include <ogr_api.h>
#include <ogr_spatialref.h>
#include <iostream>
#include <boost/filesystem.hpp>
#include <algorithm>

namespace adjfind {
namespace io {

RoadSegmentationWriter::RoadSegmentationWriter() {
    // Register GDAL drivers
    GDALAllRegister();
}

RoadSegmentationWriter::~RoadSegmentationWriter() {
    // GDAL cleanup is handled automatically
}

// Removed determineFormatFromExtension method - now using GDALUtils::determineFormatFromExtension

GDALDatasetH RoadSegmentationWriter::createGDALDataset(const RoadSegmentationWriterConfig& config, OGRCoordinateTransformationH& coord_trans, std::string& output_file_path) {
    // Initialize coordinate transformation
    coord_trans = nullptr;
    
    // Create coordinate transformation if needed
    bool reprojection_succeeded = false;
    if (config.reproject_to_epsg4326) {
        if (!config.crs_wkt.empty()) {
            OGRSpatialReferenceH source_srs = OSRNewSpatialReference(nullptr);
            OGRSpatialReferenceH target_srs = OSRNewSpatialReference(nullptr);
            
            char* wkt_copy = const_cast<char*>(config.crs_wkt.c_str());
            OSRImportFromWkt(source_srs, &wkt_copy);
            OSRImportFromEPSG(target_srs, 4326);
            
            // Set axis mapping strategy for GDAL 3+
            OSRSetAxisMappingStrategy(source_srs, OAMS_TRADITIONAL_GIS_ORDER);
            OSRSetAxisMappingStrategy(target_srs, OAMS_TRADITIONAL_GIS_ORDER);
            
            coord_trans = OCTNewCoordinateTransformation(source_srs, target_srs);
            if (coord_trans) {
                reprojection_succeeded = true;
            } else {
                std::cerr << "Warning: Failed to create coordinate transformation to EPSG:4326" << std::endl;
                std::cerr << "Output will use original coordinate system" << std::endl;
            }
            
            // Clean up spatial references
            OSRDestroySpatialReference(source_srs);
            OSRDestroySpatialReference(target_srs);
        } else {
            std::cerr << "Warning: Cannot reproject to EPSG:4326 - no source coordinate system specified" << std::endl;
            std::cerr << "Output will use original coordinate system" << std::endl;
        }
    }
    
    // Determine format from file extension and modify path if needed
    output_file_path = config.output_file_path;  // Set the output parameter
    std::string format = GDALUtils::determineFormatAndModifyPath(output_file_path);
    
    // Get driver
    GDALDriverH driver = GDALGetDriverByName(format.c_str());
    if (!driver) {
        last_error_ = "Failed to get GDAL driver for format: " + format;
        if (coord_trans) {
            OCTDestroyCoordinateTransformation(coord_trans);
            coord_trans = nullptr;
        }
        return nullptr;
    }
    
    // Create dataset using the potentially modified file path
    GDALDatasetH dataset = GDALCreate(driver, output_file_path.c_str(), 0, 0, 0, GDT_Unknown, nullptr);
    if (!dataset) {
        last_error_ = "Failed to create GDAL dataset: " + output_file_path;
        if (coord_trans) {
            OCTDestroyCoordinateTransformation(coord_trans);
            coord_trans = nullptr;
        }
        return nullptr;
    }
    
    // Create layer with appropriate CRS
    OGRSpatialReferenceH layer_srs = nullptr;
    
    if (config.reproject_to_epsg4326 && reprojection_succeeded) {
        // If reprojection is enabled and succeeded, use WGS84 for layer CRS
        layer_srs = OSRNewSpatialReference(nullptr);
        OSRImportFromEPSG(layer_srs, 4326);
        OSRSetAxisMappingStrategy(layer_srs, OAMS_TRADITIONAL_GIS_ORDER);
    } else if (!config.crs_wkt.empty()) {
        // If no reprojection or reprojection failed, use source CRS
        layer_srs = OSRNewSpatialReference(nullptr);
        char* wkt_copy = const_cast<char*>(config.crs_wkt.c_str());
        if (OSRImportFromWkt(layer_srs, &wkt_copy) != OGRERR_NONE) {
            OSRDestroySpatialReference(layer_srs);
            layer_srs = nullptr;
        } else {
            OSRSetAxisMappingStrategy(layer_srs, OAMS_TRADITIONAL_GIS_ORDER);
        }
    }
    
    // Use default layer name
    std::string layer_name = "road_segments";
    OGRLayerH layer = GDALDatasetCreateLayer(dataset, layer_name.c_str(), layer_srs, wkbLineString, nullptr);
    if (!layer) {
        last_error_ = "Failed to create layer: " + layer_name;
        if (coord_trans) {
            OCTDestroyCoordinateTransformation(coord_trans);
            coord_trans = nullptr;
        }
        if (layer_srs) OSRDestroySpatialReference(layer_srs);
        GDALClose(dataset);
        return nullptr;
    }
    
    // Create fields
    OGRFieldDefnH id_field = OGR_Fld_Create("id", OFTInteger64);
    OGR_L_CreateField(layer, id_field, 1);
    OGR_Fld_Destroy(id_field);
    
    OGRFieldDefnH road_id_field = OGR_Fld_Create("road_id", OFTInteger64);
    OGR_L_CreateField(layer, road_id_field, 1);
    OGR_Fld_Destroy(road_id_field);
    
    OGRFieldDefnH point_id_field = OGR_Fld_Create("point_id", OFTInteger64);
    OGR_L_CreateField(layer, point_id_field, 1);
    OGR_Fld_Destroy(point_id_field);
    
    OGRFieldDefnH length_field = OGR_Fld_Create("length", OFTReal);
    OGR_L_CreateField(layer, length_field, 1);
    OGR_Fld_Destroy(length_field);
    
    OGRFieldDefnH distance_category_field = OGR_Fld_Create("distance_category", OFTString);
    OGR_L_CreateField(layer, distance_category_field, 1);
    OGR_Fld_Destroy(distance_category_field);
    
    OGRFieldDefnH from_distance_field = OGR_Fld_Create("from_distance", OFTReal);
    OGR_L_CreateField(layer, from_distance_field, 1);
    OGR_Fld_Destroy(from_distance_field);
    
    OGRFieldDefnH to_distance_field = OGR_Fld_Create("to_distance", OFTReal);
    OGR_L_CreateField(layer, to_distance_field, 1);
    OGR_Fld_Destroy(to_distance_field);
    
    if (layer_srs) OSRDestroySpatialReference(layer_srs);
    return dataset;
}

bool RoadSegmentationWriter::writeFeature(GDALDatasetH dataset, OGRLayerH layer, 
                                        OGRCoordinateTransformationH coord_trans,
                                        const graph::RoadSplitByDistanceBracketsOutput& result, 
                                        size_t feature_id) {
    // Create feature
    OGRFeatureDefnH layer_defn = OGR_L_GetLayerDefn(layer);
    OGRFeatureH feature = OGR_F_Create(layer_defn);
    if (!feature) {
        last_error_ = "Failed to create feature";
        return false;
    }
    
    // Set geometry
    OGRGeometryH line_string = OGR_G_CreateGeometry(wkbLineString);
    for (const auto& point : result.geometry) {
        OGR_G_AddPoint(line_string, point.get<0>(), point.get<1>(), 0.0);
    }
    
    // Reproject geometry if transformation is provided
    if (coord_trans) {
        if (OGR_G_Transform(line_string, coord_trans) != OGRERR_NONE) {
            std::cerr << "Warning: Failed to reproject geometry to EPSG:4326" << std::endl;
        }
    }
    
    OGR_F_SetGeometry(feature, line_string);
    
    // Set fields
    OGR_F_SetFieldInteger64(feature, OGR_F_GetFieldIndex(feature, "id"), static_cast<GIntBig>(feature_id));
    OGR_F_SetFieldInteger64(feature, OGR_F_GetFieldIndex(feature, "road_id"), static_cast<GIntBig>(result.raw_feature_id));
    OGR_F_SetFieldInteger64(feature, OGR_F_GetFieldIndex(feature, "point_id"), static_cast<GIntBig>(result.nearest_point_id));
    OGR_F_SetFieldDouble(feature, OGR_F_GetFieldIndex(feature, "length"), result.length);
    OGR_F_SetFieldString(feature, OGR_F_GetFieldIndex(feature, "distance_category"), result.distance_category.c_str());
    OGR_F_SetFieldDouble(feature, OGR_F_GetFieldIndex(feature, "from_distance"), result.from_distance);
    OGR_F_SetFieldDouble(feature, OGR_F_GetFieldIndex(feature, "to_distance"), result.to_distance);
    
    // Create feature in layer
    if (OGR_L_CreateFeature(layer, feature) != OGRERR_NONE) {
        last_error_ = "Failed to create feature in layer";
        OGR_F_Destroy(feature);
        return false;
    }
    
    OGR_F_Destroy(feature);
    return true;
}

bool RoadSegmentationWriter::writeRoadSegmentationResults(const RoadSegmentationWriterConfig& config, 
                                                        const std::vector<graph::RoadSplitByDistanceBracketsOutput>& results) {
    clearError();
    
    if (results.empty()) {
        last_error_ = "No results to write";
        return false;
    }
    
    // Create dataset and coordinate transformation
    OGRCoordinateTransformationH coord_trans = nullptr;
    std::string output_file_path;  // Will be set by createGDALDataset
    GDALDatasetH dataset = createGDALDataset(config, coord_trans, output_file_path);
    if (!dataset) {
        // createGDALDataset already sets last_error_ if it fails
        return false;
    }
    
    OGRLayerH layer = GDALDatasetGetLayer(dataset, 0);
    
    if (!layer) {
        last_error_ = "Failed to get layer from dataset";
        if (coord_trans) {
            OCTDestroyCoordinateTransformation(coord_trans);
        }
        GDALClose(dataset);
        return false;
    }
    
    // Write features
    size_t feature_id = 0;
    for (const auto& result : results) {
        if (!writeFeature(dataset, layer, coord_trans, result, feature_id)) {
            if (coord_trans) OCTDestroyCoordinateTransformation(coord_trans);
            GDALClose(dataset);
            return false;
        }
        feature_id++;
    }
    
    // Clean up coordinate transformation
    if (coord_trans) {
        OCTDestroyCoordinateTransformation(coord_trans);
    }
    
    // Close dataset
    GDALClose(dataset);
    
    std::cout << "Successfully wrote " << results.size() << " road segments to: " << output_file_path << std::endl;
    return true;
}

} // namespace io
} // namespace adjfind 