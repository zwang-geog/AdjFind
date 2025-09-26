#include "io/writer_convex_path.hpp"
#include <gdal.h>
#include <ogr_api.h>
#include <ogr_spatialref.h>
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

GDALDatasetH ConvexPathWriter::createLinestringDataset(const ConvexPathWriterConfig& config, OGRCoordinateTransformationH& coord_trans, std::string& output_file_path) {
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
    std::string layer_name = "convex_paths";
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
    OGRFieldDefnH row_id_field = OGR_Fld_Create("row_id", OFTInteger64);
    OGR_L_CreateField(layer, row_id_field, 1);
    OGR_Fld_Destroy(row_id_field);
    
    OGRFieldDefnH polygon_feature_id_field = OGR_Fld_Create("polygon_feature_id", OFTInteger64);
    OGR_L_CreateField(layer, polygon_feature_id_field, 1);
    OGR_Fld_Destroy(polygon_feature_id_field);
    
    OGRFieldDefnH path_type_field = OGR_Fld_Create("path_type", OFTString);
    OGR_L_CreateField(layer, path_type_field, 1);
    OGR_Fld_Destroy(path_type_field);
    
    OGRFieldDefnH assigned_point_feature_id_field = OGR_Fld_Create("assigned_point_feature_id", OFTInteger64);
    OGR_L_CreateField(layer, assigned_point_feature_id_field, 1);
    OGR_Fld_Destroy(assigned_point_feature_id_field);
    
    OGRFieldDefnH snapped_road_feature_id_field = OGR_Fld_Create("snapped_road_feature_id", OFTInteger64);
    OGR_L_CreateField(layer, snapped_road_feature_id_field, 1);
    OGR_Fld_Destroy(snapped_road_feature_id_field);
    
    OGRFieldDefnH distance_to_assigned_point_field = OGR_Fld_Create("distance_to_assigned_point", OFTReal);
    OGR_L_CreateField(layer, distance_to_assigned_point_field, 1);
    OGR_Fld_Destroy(distance_to_assigned_point_field);
    
    OGRFieldDefnH access_distance_field = OGR_Fld_Create("access_distance", OFTReal);
    OGR_L_CreateField(layer, access_distance_field, 1);
    OGR_Fld_Destroy(access_distance_field);
    
    if (layer_srs) OSRDestroySpatialReference(layer_srs);
    return dataset;
}

GDALDatasetH ConvexPathWriter::createPointDataset(const ConvexPathWriterConfig& config, OGRCoordinateTransformationH& coord_trans, std::string& output_file_path) {
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
    
    // Modify output file path for point dataset
    output_file_path = config.output_file_path;
    std::filesystem::path path(output_file_path);
    std::string stem = path.stem().string();
    std::string extension = path.extension().string();
    output_file_path = (path.parent_path() / (stem + "_least_accessible_point" + extension)).string();
    
    // Determine format from file extension and modify path if needed
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
    std::string layer_name = "least_accessible_points";
    OGRLayerH layer = GDALDatasetCreateLayer(dataset, layer_name.c_str(), layer_srs, wkbPoint, nullptr);
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
    OGRFieldDefnH row_id_field = OGR_Fld_Create("row_id", OFTInteger64);
    OGR_L_CreateField(layer, row_id_field, 1);
    OGR_Fld_Destroy(row_id_field);
    
    OGRFieldDefnH polygon_feature_id_field = OGR_Fld_Create("polygon_feature_id", OFTInteger64);
    OGR_L_CreateField(layer, polygon_feature_id_field, 1);
    OGR_Fld_Destroy(polygon_feature_id_field);
    
    OGRFieldDefnH assigned_point_feature_id_field = OGR_Fld_Create("assigned_point_feature_id", OFTInteger64);
    OGR_L_CreateField(layer, assigned_point_feature_id_field, 1);
    OGR_Fld_Destroy(assigned_point_feature_id_field);
    
    OGRFieldDefnH snapped_road_feature_id_field = OGR_Fld_Create("snapped_road_feature_id", OFTInteger64);
    OGR_L_CreateField(layer, snapped_road_feature_id_field, 1);
    OGR_Fld_Destroy(snapped_road_feature_id_field);
    
    OGRFieldDefnH distance_to_assigned_point_field = OGR_Fld_Create("distance_to_assigned_point", OFTReal);
    OGR_L_CreateField(layer, distance_to_assigned_point_field, 1);
    OGR_Fld_Destroy(distance_to_assigned_point_field);
    
    OGRFieldDefnH access_distance_field = OGR_Fld_Create("access_distance", OFTReal);
    OGR_L_CreateField(layer, access_distance_field, 1);
    OGR_Fld_Destroy(access_distance_field);
    
    if (layer_srs) OSRDestroySpatialReference(layer_srs);
    return dataset;
}

bool ConvexPathWriter::writeLinestringFeature(GDALDatasetH dataset, OGRLayerH layer, 
                                             OGRCoordinateTransformationH coord_trans,
                                             const std::vector<graph::ConvexPathResult>& vertex_paths, 
                                             size_t polygon_feature_id,
                                             const graph::Point& least_accessible_point,
                                             size_t feature_id) {
    // Write a feature for each ConvexPathResult
    for (const auto& path_result : vertex_paths) {
        // Create feature
        OGRFeatureDefnH layer_defn = OGR_L_GetLayerDefn(layer);
        OGRFeatureH feature = OGR_F_Create(layer_defn);
        if (!feature) {
            last_error_ = "Failed to create feature";
            return false;
        }
        
        // Set geometry - create empty linestring if path not found
        OGRGeometryH line_string = OGR_G_CreateGeometry(wkbLineString);
        if (path_result.path_found && !path_result.path_geometry.empty()) {
            for (const auto& point : path_result.path_geometry) {
                OGR_G_AddPoint(line_string, point.get<0>(), point.get<1>(), 0.0);
            }
            
            // Reproject geometry if transformation is provided
            if (coord_trans) {
                if (OGR_G_Transform(line_string, coord_trans) != OGRERR_NONE) {
                    std::cerr << "Warning: Failed to reproject geometry to EPSG:4326" << std::endl;
                }
            }
        }
        // If path not found, line_string remains empty (which is valid)
        
        OGR_F_SetGeometry(feature, line_string);
        
        // Set fields
        OGR_F_SetFieldInteger64(feature, OGR_F_GetFieldIndex(feature, "row_id"), static_cast<GIntBig>(feature_id++));
        OGR_F_SetFieldInteger64(feature, OGR_F_GetFieldIndex(feature, "polygon_feature_id"), static_cast<GIntBig>(polygon_feature_id));
        OGR_F_SetFieldString(feature, OGR_F_GetFieldIndex(feature, "path_type"), pathTypeToString(path_result.start_point_type).c_str());
        
        // Set assigned_point_feature_id (using nearest_point_vertex_position_index)
        if (path_result.path_found && path_result.nearest_point_vertex_position_index != std::numeric_limits<size_t>::max()) {
            OGR_F_SetFieldInteger64(feature, OGR_F_GetFieldIndex(feature, "assigned_point_feature_id"), static_cast<GIntBig>(path_result.nearest_point_vertex_position_index));
        } else {
            OGR_F_SetFieldNull(feature, OGR_F_GetFieldIndex(feature, "assigned_point_feature_id"));
        }
        
        // Set snapped_road_feature_id (using edge_index)
        if (path_result.path_found && path_result.edge_index != std::numeric_limits<size_t>::max()) {
            OGR_F_SetFieldInteger64(feature, OGR_F_GetFieldIndex(feature, "snapped_road_feature_id"), static_cast<GIntBig>(path_result.edge_index));
        } else {
            OGR_F_SetFieldNull(feature, OGR_F_GetFieldIndex(feature, "snapped_road_feature_id"));
        }
        
        // Set distance_to_assigned_point
        if (path_result.path_found) {
            OGR_F_SetFieldDouble(feature, OGR_F_GetFieldIndex(feature, "distance_to_assigned_point"), path_result.total_length);
        } else {
            OGR_F_SetFieldNull(feature, OGR_F_GetFieldIndex(feature, "distance_to_assigned_point"));
        }
        
        // Set access_distance (length of linestring geometry)
        if (path_result.path_found && !path_result.path_geometry.empty()) {
            double access_distance = bg::length(path_result.path_geometry);
            OGR_F_SetFieldDouble(feature, OGR_F_GetFieldIndex(feature, "access_distance"), access_distance);
        } else {
            OGR_F_SetFieldNull(feature, OGR_F_GetFieldIndex(feature, "access_distance"));
        }
        
        // Create feature in layer
        if (OGR_L_CreateFeature(layer, feature) != OGRERR_NONE) {
            last_error_ = "Failed to create feature in layer";
            OGR_F_Destroy(feature);
            return false;
        }
        
        OGR_F_Destroy(feature);
    }
    
    return true;
}

bool ConvexPathWriter::writePointFeature(GDALDatasetH dataset, OGRLayerH layer, 
                                        OGRCoordinateTransformationH coord_trans,
                                        const std::vector<graph::ConvexPathResult>& vertex_paths, 
                                        size_t polygon_feature_id,
                                        const graph::Point& least_accessible_point,
                                        size_t feature_id) {
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
    OGRFeatureDefnH layer_defn = OGR_L_GetLayerDefn(layer);
    OGRFeatureH feature = OGR_F_Create(layer_defn);
    if (!feature) {
        last_error_ = "Failed to create feature";
        return false;
    }
    
    // Set geometry
    OGRGeometryH point_geom = OGR_G_CreateGeometry(wkbPoint);
    OGR_G_SetPoint(point_geom, 0, least_accessible_point.get<0>(), least_accessible_point.get<1>(), 0.0);
    
    // Reproject geometry if transformation is provided
    if (coord_trans) {
        if (OGR_G_Transform(point_geom, coord_trans) != OGRERR_NONE) {
            std::cerr << "Warning: Failed to reproject geometry to EPSG:4326" << std::endl;
        }
    }
    
    OGR_F_SetGeometry(feature, point_geom);
    
    // Set fields
    OGR_F_SetFieldInteger64(feature, OGR_F_GetFieldIndex(feature, "row_id"), static_cast<GIntBig>(feature_id));
    OGR_F_SetFieldInteger64(feature, OGR_F_GetFieldIndex(feature, "polygon_feature_id"), static_cast<GIntBig>(polygon_feature_id));
    
    // Set assigned_point_feature_id (using nearest_point_vertex_position_index)
    if (least_accessible_result->nearest_point_vertex_position_index != std::numeric_limits<size_t>::max()) {
        OGR_F_SetFieldInteger64(feature, OGR_F_GetFieldIndex(feature, "assigned_point_feature_id"), static_cast<GIntBig>(least_accessible_result->nearest_point_vertex_position_index));
    } else {
        OGR_F_SetFieldNull(feature, OGR_F_GetFieldIndex(feature, "assigned_point_feature_id"));
    }
    
    // Set snapped_road_feature_id (using edge_index)
    if (least_accessible_result->edge_index != std::numeric_limits<size_t>::max()) {
        OGR_F_SetFieldInteger64(feature, OGR_F_GetFieldIndex(feature, "snapped_road_feature_id"), static_cast<GIntBig>(least_accessible_result->edge_index));
    } else {
        OGR_F_SetFieldNull(feature, OGR_F_GetFieldIndex(feature, "snapped_road_feature_id"));
    }
    
    // Set distance_to_assigned_point
    OGR_F_SetFieldDouble(feature, OGR_F_GetFieldIndex(feature, "distance_to_assigned_point"), least_accessible_result->total_length);
    
    // Set access_distance (length of linestring geometry)
    if (least_accessible_result->path_found && !least_accessible_result->path_geometry.empty()) {
        double access_distance = bg::length(least_accessible_result->path_geometry);
        OGR_F_SetFieldDouble(feature, OGR_F_GetFieldIndex(feature, "access_distance"), access_distance);
    } else {
        OGR_F_SetFieldNull(feature, OGR_F_GetFieldIndex(feature, "access_distance"));
    }
    
    // Create feature in layer
    if (OGR_L_CreateFeature(layer, feature) != OGRERR_NONE) {
        last_error_ = "Failed to create feature in layer";
        OGR_F_Destroy(feature);
        return false;
    }
    
    OGR_F_Destroy(feature);
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
    OGRCoordinateTransformationH linestring_coord_trans = nullptr;
    std::string linestring_output_file_path;
    GDALDatasetH linestring_dataset = createLinestringDataset(config, linestring_coord_trans, linestring_output_file_path);
    if (!linestring_dataset) {
        if (linestring_coord_trans) {
            OCTDestroyCoordinateTransformation(linestring_coord_trans);
        }
        return false;
    }
    
    // Create point dataset and coordinate transformation
    OGRCoordinateTransformationH point_coord_trans = nullptr;
    std::string point_output_file_path;
    GDALDatasetH point_dataset = createPointDataset(config, point_coord_trans, point_output_file_path);
    if (!point_dataset) {
        if (linestring_coord_trans) {
            OCTDestroyCoordinateTransformation(linestring_coord_trans);
        }
        if (point_coord_trans) {
            OCTDestroyCoordinateTransformation(point_coord_trans);
        }
        GDALClose(linestring_dataset);
        return false;
    }
    
    // Get layers
    OGRLayerH linestring_layer = GDALDatasetGetLayer(linestring_dataset, 0);
    OGRLayerH point_layer = GDALDatasetGetLayer(point_dataset, 0);
    
    if (!linestring_layer || !point_layer) {
        last_error_ = "Failed to get layers from datasets";
        if (linestring_coord_trans) {
            OCTDestroyCoordinateTransformation(linestring_coord_trans);
        }
        if (point_coord_trans) {
            OCTDestroyCoordinateTransformation(point_coord_trans);
        }
        GDALClose(linestring_dataset);
        GDALClose(point_dataset);
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
    
    GDALClose(linestring_dataset);
    GDALClose(point_dataset);
    
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
