#include "io/writer_road_segmentation.hpp"
#include <gdal.h>
#include <gdal_priv.h>
#include <ogr_feature.h>
#include <ogr_geometry.h>
#include <iostream>
#include <filesystem>
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

void* RoadSegmentationWriter::createGDALDataset(const RoadSegmentationWriterConfig& config, OGRCoordinateTransformation*& coord_trans, std::string& output_file_path) {
    // Initialize coordinate transformation
    coord_trans = nullptr;
    
    // Create coordinate transformation if needed
    bool reprojection_succeeded = false;
    if (config.reproject_to_epsg4326) {
        if (!config.crs_wkt.empty()) {
            OGRSpatialReference source_srs, target_srs;
            source_srs.importFromWkt(config.crs_wkt.c_str());
            target_srs.importFromEPSG(4326);
            
            // Set axis mapping strategy for GDAL 3+
            source_srs.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
            target_srs.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
            
            coord_trans = OGRCreateCoordinateTransformation(&source_srs, &target_srs);
            if (coord_trans) {
                reprojection_succeeded = true;
            } else {
                std::cerr << "Warning: Failed to create coordinate transformation to EPSG:4326" << std::endl;
                std::cerr << "Output will use original coordinate system" << std::endl;
            }
        } else {
            std::cerr << "Warning: Cannot reproject to EPSG:4326 - no source coordinate system specified" << std::endl;
            std::cerr << "Output will use original coordinate system" << std::endl;
        }
    }
    
    // Determine format from file extension and modify path if needed
    output_file_path = config.output_file_path;  // Set the output parameter
    std::string format = GDALUtils::determineFormatAndModifyPath(output_file_path);
    
    // Get driver
    GDALDriver* driver = GetGDALDriverManager()->GetDriverByName(format.c_str());
    if (!driver) {
        last_error_ = "Failed to get GDAL driver for format: " + format;
        if (coord_trans) {
            OCTDestroyCoordinateTransformation(coord_trans);
            coord_trans = nullptr;
        }
        return nullptr;
    }
    
    // Create dataset using the potentially modified file path
    GDALDataset* dataset = driver->Create(output_file_path.c_str(), 0, 0, 0, GDT_Unknown, nullptr);
    if (!dataset) {
        last_error_ = "Failed to create GDAL dataset: " + output_file_path;
        if (coord_trans) {
            OCTDestroyCoordinateTransformation(coord_trans);
            coord_trans = nullptr;
        }
        return nullptr;
    }
    
    // Create layer with appropriate CRS
    OGRSpatialReference* layer_srs = nullptr;
    OGRSpatialReference wgs84_srs;
    
    if (config.reproject_to_epsg4326 && reprojection_succeeded) {
        // If reprojection is enabled and succeeded, use WGS84 for layer CRS
        wgs84_srs.importFromEPSG(4326);
        wgs84_srs.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
        layer_srs = &wgs84_srs;
    } else if (!config.crs_wkt.empty()) {
        // If no reprojection or reprojection failed, use source CRS
        layer_srs = new OGRSpatialReference();
        if (layer_srs->importFromWkt(config.crs_wkt.c_str()) != OGRERR_NONE) {
            delete layer_srs;
            layer_srs = nullptr;
        } else {
            layer_srs->SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
        }
    }
    
    // Use default layer name
    std::string layer_name = "road_segments";
    OGRLayer* layer = dataset->CreateLayer(layer_name.c_str(), layer_srs, wkbLineString, nullptr);
    if (!layer) {
        last_error_ = "Failed to create layer: " + layer_name;
        if (coord_trans) {
            OCTDestroyCoordinateTransformation(coord_trans);
            coord_trans = nullptr;
        }
        if (layer_srs && layer_srs != &wgs84_srs) delete layer_srs;
        GDALClose(dataset);
        return nullptr;
    }
    
    // Create fields
    OGRFieldDefn id_field("id", OFTInteger64);
    layer->CreateField(&id_field);
    
    OGRFieldDefn road_id_field("road_id", OFTInteger64);
    layer->CreateField(&road_id_field);
    
    OGRFieldDefn point_id_field("point_id", OFTInteger64);
    layer->CreateField(&point_id_field);
    
    OGRFieldDefn length_field("length", OFTReal);
    layer->CreateField(&length_field);
    
    OGRFieldDefn distance_category_field("distance_category", OFTString);
    layer->CreateField(&distance_category_field);
    
    OGRFieldDefn from_distance_field("from_distance", OFTReal);
    layer->CreateField(&from_distance_field);
    
    OGRFieldDefn to_distance_field("to_distance", OFTReal);
    layer->CreateField(&to_distance_field);
    
    if (layer_srs && layer_srs != &wgs84_srs) delete layer_srs;
    return dataset;
}

bool RoadSegmentationWriter::writeFeature(void* dataset_ptr, void* layer_ptr, 
                                        OGRCoordinateTransformation* coord_trans,
                                        const graph::RoadSplitByDistanceBracketsOutput& result, 
                                        size_t feature_id) {
    GDALDataset* dataset = static_cast<GDALDataset*>(dataset_ptr);
    OGRLayer* layer = static_cast<OGRLayer*>(layer_ptr);
    
    // Create feature
    OGRFeature* feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
    if (!feature) {
        last_error_ = "Failed to create feature";
        return false;
    }
    
    // Set geometry
    OGRLineString* line_string = new OGRLineString();
    for (const auto& point : result.geometry) {
        line_string->addPoint(point.get<0>(), point.get<1>());
    }
    
    // Reproject geometry if transformation is provided
    if (coord_trans) {
        if (!line_string->transform(coord_trans)) {
            std::cerr << "Warning: Failed to reproject geometry to EPSG:4326" << std::endl;
        }
    }
    
    feature->SetGeometry(line_string);
    
    // Set fields
    feature->SetField("id", static_cast<GIntBig>(feature_id));
    feature->SetField("road_id", static_cast<GIntBig>(result.raw_feature_id));
    feature->SetField("point_id", static_cast<GIntBig>(result.nearest_point_id));
    feature->SetField("length", result.length);
    feature->SetField("distance_category", result.distance_category.c_str());
    feature->SetField("from_distance", result.from_distance);
    feature->SetField("to_distance", result.to_distance);
    
    // Create feature in layer
    if (layer->CreateFeature(feature) != OGRERR_NONE) {
        last_error_ = "Failed to create feature in layer";
        OGRFeature::DestroyFeature(feature);
        return false;
    }
    
    OGRFeature::DestroyFeature(feature);
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
    OGRCoordinateTransformation* coord_trans = nullptr;
    std::string output_file_path;  // Will be set by createGDALDataset
    void* dataset_ptr = createGDALDataset(config, coord_trans, output_file_path);
    if (!dataset_ptr) {
        // createGDALDataset already sets last_error_ if it fails
        return false;
    }
    
    GDALDataset* dataset = static_cast<GDALDataset*>(dataset_ptr);
    OGRLayer* layer = dataset->GetLayer(0);
    
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
        if (!writeFeature(dataset_ptr, layer, coord_trans, result, feature_id)) {
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