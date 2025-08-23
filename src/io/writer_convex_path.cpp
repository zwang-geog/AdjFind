#include "io/writer_convex_path.hpp"
#include <gdal.h>
#include <gdal_priv.h>
#include <ogr_feature.h>
#include <ogr_geometry.h>
#include <iostream>
#include <filesystem>
#include <algorithm>
#include <sstream>
#include <boost/geometry.hpp>

namespace adjfind {
namespace io {

// Boost Geometry namespace alias
namespace bg = boost::geometry;

ConvexPathWriter::ConvexPathWriter() {
    // Register GDAL drivers
    GDALAllRegister();
}

ConvexPathWriter::~ConvexPathWriter() {
    // GDAL cleanup is handled automatically
}

void* ConvexPathWriter::createLinestringDataset(const ConvexPathWriterConfig& config, OGRCoordinateTransformation*& coord_trans, std::string& output_file_path) {
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
    std::string layer_name = "convex_paths";
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
    OGRFieldDefn row_id_field("row_id", OFTInteger64);
    layer->CreateField(&row_id_field);
    
    OGRFieldDefn polygon_feature_id_field("polygon_feature_id", OFTInteger64);
    layer->CreateField(&polygon_feature_id_field);
    
    OGRFieldDefn path_type_field("path_type", OFTString);
    layer->CreateField(&path_type_field);
    
    OGRFieldDefn assigned_point_feature_id_field("assigned_point_feature_id", OFTInteger64);
    layer->CreateField(&assigned_point_feature_id_field);
    
    OGRFieldDefn snapped_road_feature_id_field("snapped_road_feature_id", OFTInteger64);
    layer->CreateField(&snapped_road_feature_id_field);
    
    OGRFieldDefn distance_to_assigned_point_field("distance_to_assigned_point", OFTReal);
    layer->CreateField(&distance_to_assigned_point_field);
    
    OGRFieldDefn access_distance_field("access_distance", OFTReal);
    layer->CreateField(&access_distance_field);
    
    if (layer_srs && layer_srs != &wgs84_srs) delete layer_srs;
    return dataset;
}

void* ConvexPathWriter::createPointDataset(const ConvexPathWriterConfig& config, OGRCoordinateTransformation*& coord_trans, std::string& output_file_path) {
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
    
    // Modify output file path for point dataset
    output_file_path = config.output_file_path;
    std::filesystem::path path(output_file_path);
    std::string stem = path.stem().string();
    std::string extension = path.extension().string();
    output_file_path = (path.parent_path() / (stem + "_least_accessible_point" + extension)).string();
    
    // Determine format from file extension and modify path if needed
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
    std::string layer_name = "least_accessible_points";
    OGRLayer* layer = dataset->CreateLayer(layer_name.c_str(), layer_srs, wkbPoint, nullptr);
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
    OGRFieldDefn row_id_field("row_id", OFTInteger64);
    layer->CreateField(&row_id_field);
    
    OGRFieldDefn polygon_feature_id_field("polygon_feature_id", OFTInteger64);
    layer->CreateField(&polygon_feature_id_field);
    
    OGRFieldDefn assigned_point_feature_id_field("assigned_point_feature_id", OFTInteger64);
    layer->CreateField(&assigned_point_feature_id_field);
    
    OGRFieldDefn snapped_road_feature_id_field("snapped_road_feature_id", OFTInteger64);
    layer->CreateField(&snapped_road_feature_id_field);
    
    OGRFieldDefn distance_to_assigned_point_field("distance_to_assigned_point", OFTReal);
    layer->CreateField(&distance_to_assigned_point_field);
    
    OGRFieldDefn access_distance_field("access_distance", OFTReal);
    layer->CreateField(&access_distance_field);
    
    if (layer_srs && layer_srs != &wgs84_srs) delete layer_srs;
    return dataset;
}

bool ConvexPathWriter::writeLinestringFeature(void* dataset_ptr, void* layer_ptr, 
                                             OGRCoordinateTransformation* coord_trans,
                                             const std::vector<graph::ConvexPathResult>& vertex_paths, 
                                             size_t polygon_feature_id,
                                             const graph::Point& least_accessible_point,
                                             size_t feature_id) {
    GDALDataset* dataset = static_cast<GDALDataset*>(dataset_ptr);
    OGRLayer* layer = static_cast<OGRLayer*>(layer_ptr);
    
    // Write a feature for each ConvexPathResult
    for (const auto& path_result : vertex_paths) {
        // Create feature
        OGRFeature* feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
        if (!feature) {
            last_error_ = "Failed to create feature";
            return false;
        }
        
        // Set geometry - create empty linestring if path not found
        OGRLineString* line_string = new OGRLineString();
        if (path_result.path_found && !path_result.path_geometry.empty()) {
            for (const auto& point : path_result.path_geometry) {
                line_string->addPoint(point.get<0>(), point.get<1>());
            }
            
            // Reproject geometry if transformation is provided
            if (coord_trans) {
                if (!line_string->transform(coord_trans)) {
                    std::cerr << "Warning: Failed to reproject geometry to EPSG:4326" << std::endl;
                }
            }
        }
        // If path not found, line_string remains empty (which is valid)
        
        feature->SetGeometry(line_string);
        
        // Set fields
        feature->SetField("row_id", static_cast<GIntBig>(feature_id++));
        feature->SetField("polygon_feature_id", static_cast<GIntBig>(polygon_feature_id));
        feature->SetField("path_type", pathTypeToString(path_result.start_point_type).c_str());
        
        // Set assigned_point_feature_id (using nearest_point_vertex_position_index)
        if (path_result.path_found && path_result.nearest_point_vertex_position_index != std::numeric_limits<size_t>::max()) {
            feature->SetField("assigned_point_feature_id", static_cast<GIntBig>(path_result.nearest_point_vertex_position_index));
        } else {
            feature->SetFieldNull(feature->GetFieldIndex("assigned_point_feature_id"));
        }
        
        // Set snapped_road_feature_id (using edge_index)
        if (path_result.path_found && path_result.edge_index != std::numeric_limits<size_t>::max()) {
            feature->SetField("snapped_road_feature_id", static_cast<GIntBig>(path_result.edge_index));
        } else {
            feature->SetFieldNull(feature->GetFieldIndex("snapped_road_feature_id"));
        }
        
        // Set distance_to_assigned_point
        if (path_result.path_found) {
            feature->SetField("distance_to_assigned_point", path_result.total_length);
        } else {
            feature->SetFieldNull(feature->GetFieldIndex("distance_to_assigned_point"));
        }
        
        // Set access_distance (length of linestring geometry)
        if (path_result.path_found && !path_result.path_geometry.empty()) {
            double access_distance = bg::length(path_result.path_geometry);
            feature->SetField("access_distance", access_distance);
        } else {
            feature->SetFieldNull(feature->GetFieldIndex("access_distance"));
        }
        
        // Create feature in layer
        if (layer->CreateFeature(feature) != OGRERR_NONE) {
            last_error_ = "Failed to create feature in layer";
            OGRFeature::DestroyFeature(feature);
            return false;
        }
        
        OGRFeature::DestroyFeature(feature);
    }
    
    return true;
}

bool ConvexPathWriter::writePointFeature(void* dataset_ptr, void* layer_ptr, 
                                        OGRCoordinateTransformation* coord_trans,
                                        const std::vector<graph::ConvexPathResult>& vertex_paths, 
                                        size_t polygon_feature_id,
                                        const graph::Point& least_accessible_point,
                                        size_t feature_id) {
    GDALDataset* dataset = static_cast<GDALDataset*>(dataset_ptr);
    OGRLayer* layer = static_cast<OGRLayer*>(layer_ptr);
    
    // Find the ConvexPathResult with LEAST_ACCESSIBLE_POINT type
    const graph::ConvexPathResult* least_accessible_result = nullptr;
    for (const auto& path_result : vertex_paths) {
        if (path_result.start_point_type == graph::ConvexPathResultType::LEAST_ACCESSIBLE_POINT) {
            least_accessible_result = &path_result;
            break;
        }
    }
    
    if (!least_accessible_result) {
        // No least accessible point found, skip this polygon
        std::cerr << "WARNING: No least accessible point found for polygon " << polygon_feature_id << std::endl;
        return true;
    }
    
    // Create feature
    OGRFeature* feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
    if (!feature) {
        last_error_ = "Failed to create feature";
        return false;
    }
    
    // Set geometry
    OGRPoint* point_geom = new OGRPoint(least_accessible_point.get<0>(), least_accessible_point.get<1>());
    
    // Reproject geometry if transformation is provided
    if (coord_trans) {
        if (!point_geom->transform(coord_trans)) {
            std::cerr << "Warning: Failed to reproject geometry to EPSG:4326" << std::endl;
        }
    }
    
    feature->SetGeometry(point_geom);
    
    // Set fields
    feature->SetField("row_id", static_cast<GIntBig>(feature_id));
    feature->SetField("polygon_feature_id", static_cast<GIntBig>(polygon_feature_id));
    
    // Set assigned_point_feature_id (using nearest_point_vertex_position_index)
    if (least_accessible_result->nearest_point_vertex_position_index != std::numeric_limits<size_t>::max()) {
        feature->SetField("assigned_point_feature_id", static_cast<GIntBig>(least_accessible_result->nearest_point_vertex_position_index));
    } else {
        feature->SetFieldNull(feature->GetFieldIndex("assigned_point_feature_id"));
    }
    
    // Set snapped_road_feature_id (using edge_index)
    if (least_accessible_result->edge_index != std::numeric_limits<size_t>::max()) {
        feature->SetField("snapped_road_feature_id", static_cast<GIntBig>(least_accessible_result->edge_index));
    } else {
        feature->SetFieldNull(feature->GetFieldIndex("snapped_road_feature_id"));
    }
    
    // Set distance_to_assigned_point
    feature->SetField("distance_to_assigned_point", least_accessible_result->total_length);
    
    // Set access_distance (length of linestring geometry)
    if (least_accessible_result->path_found && !least_accessible_result->path_geometry.empty()) {
        double access_distance = bg::length(least_accessible_result->path_geometry);
        feature->SetField("access_distance", access_distance);
    } else {
        feature->SetFieldNull(feature->GetFieldIndex("access_distance"));
    }
    
    // Create feature in layer
    if (layer->CreateFeature(feature) != OGRERR_NONE) {
        last_error_ = "Failed to create feature in layer";
        OGRFeature::DestroyFeature(feature);
        return false;
    }
    
    OGRFeature::DestroyFeature(feature);
    return true;
}

std::string ConvexPathWriter::pathTypeToString(graph::ConvexPathResultType path_type) {
    switch (path_type) {
        case graph::ConvexPathResultType::NOT_FOUND:
            return "NOT_FOUND";
        case graph::ConvexPathResultType::BUILDING_CORNER:
            return "BUILDING_CORNER";
        case graph::ConvexPathResultType::LEAST_ACCESSIBLE_POINT:
            return "LEAST_ACCESSIBLE_POINT";
        default:
            return "UNKNOWN";
    }
}

bool ConvexPathWriter::writeConvexPathResults(const ConvexPathWriterConfig& config, 
                                             const std::vector<std::tuple<std::vector<graph::ConvexPathResult>, size_t, graph::Point>>& polygon_results) {
    clearError();
    
    if (polygon_results.empty()) {
        last_error_ = "No results to write";
        return false;
    }
    
    // Create linestring dataset and coordinate transformation
    OGRCoordinateTransformation* linestring_coord_trans = nullptr;
    std::string linestring_output_file_path;
    void* linestring_dataset = createLinestringDataset(config, linestring_coord_trans, linestring_output_file_path);
    if (!linestring_dataset) {
        if (linestring_coord_trans) {
            OCTDestroyCoordinateTransformation(linestring_coord_trans);
        }
        return false;
    }
    
    // Create point dataset and coordinate transformation
    OGRCoordinateTransformation* point_coord_trans = nullptr;
    std::string point_output_file_path;
    void* point_dataset = createPointDataset(config, point_coord_trans, point_output_file_path);
    if (!point_dataset) {
        if (linestring_coord_trans) {
            OCTDestroyCoordinateTransformation(linestring_coord_trans);
        }
        if (point_coord_trans) {
            OCTDestroyCoordinateTransformation(point_coord_trans);
        }
        GDALClose(static_cast<GDALDataset*>(linestring_dataset));
        return false;
    }
    
    // Get layers
    GDALDataset* linestring_ds = static_cast<GDALDataset*>(linestring_dataset);
    GDALDataset* point_ds = static_cast<GDALDataset*>(point_dataset);
    
    OGRLayer* linestring_layer = linestring_ds->GetLayer(0);
    OGRLayer* point_layer = point_ds->GetLayer(0);
    
    if (!linestring_layer || !point_layer) {
        last_error_ = "Failed to get layers from datasets";
        if (linestring_coord_trans) {
            OCTDestroyCoordinateTransformation(linestring_coord_trans);
        }
        if (point_coord_trans) {
            OCTDestroyCoordinateTransformation(point_coord_trans);
        }
        GDALClose(linestring_ds);
        GDALClose(point_ds);
        return false;
    }
    
    // Write features
    size_t feature_id = 0;
    size_t point_feature_id = 0;
    
    for (const auto& polygon_result : polygon_results) {
        const auto& [vertex_paths, polygon_feature_id, least_accessible_point] = polygon_result;
        
        // Write linestring features
        if (!writeLinestringFeature(linestring_dataset, linestring_layer, linestring_coord_trans, 
                                   vertex_paths, polygon_feature_id, least_accessible_point, feature_id)) {
            last_error_ = "Failed to write linestring feature for polygon " + std::to_string(polygon_feature_id);
            break;
        }
        
        // Write point feature (only for LEAST_ACCESSIBLE_POINT)
        if (!writePointFeature(point_dataset, point_layer, point_coord_trans, 
                              vertex_paths, polygon_feature_id, least_accessible_point, point_feature_id)) {
            last_error_ = "Failed to write point feature for polygon " + std::to_string(polygon_feature_id);
            break;
        }
        
        // Increment feature IDs
        feature_id += vertex_paths.size();
        point_feature_id++; // Only one point per polygon
    }
    
    // Cleanup
    if (linestring_coord_trans) {
        OCTDestroyCoordinateTransformation(linestring_coord_trans);
    }
    if (point_coord_trans) {
        OCTDestroyCoordinateTransformation(point_coord_trans);
    }
    
    GDALClose(linestring_ds);
    GDALClose(point_ds);
    
    if (last_error_.empty()) {
        std::cout << "Successfully wrote convex path results:" << std::endl;
        std::cout << "  Linestring dataset: " << linestring_output_file_path << std::endl;
        std::cout << "  Point dataset: " << point_output_file_path << std::endl;
        std::cout << "  Total features written: " << feature_id << " linestrings, " << point_feature_id << " points" << std::endl;
    }
    
    return last_error_.empty();
}

} // namespace io
} // namespace adjfind
