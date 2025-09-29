#include "io/writer_neighboring_points.hpp"
#include "graph/neighboring_points.hpp"
#include <gdal.h>
#include <ogr_api.h>
#include <ogr_spatialref.h>
#include <iostream>
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;
#include <algorithm>
#include <sstream>
#include <unordered_set>
#include <limits>

// Boost Geometry namespace alias
namespace bg = boost::geometry;



namespace adjfind {
namespace io {

NeighboringPointsWriter::NeighboringPointsWriter() {
    // Register GDAL drivers
    GDALAllRegister();
}

NeighboringPointsWriter::~NeighboringPointsWriter() {
    // GDAL cleanup is handled automatically
}

bool NeighboringPointsWriter::writeNeighboringPointsResults(const NeighboringPointsWriterConfig& config, 
                                                           const std::unordered_map<std::pair<size_t, size_t>, std::tuple<size_t, double, graph::MultiLineString>, graph::PairHash>& neighboring_points_results,
                                                           const graph::NeighboringPoints& neighboring_points_instance) {
    
    // Create linestring dataset
    OGRCoordinateTransformationH coord_trans_linestring = nullptr;
    std::string linestring_file_path = config.output_file_path;
    GDALDatasetH linestring_dataset = createLinestringDataset(config, coord_trans_linestring, linestring_file_path);
    if (!linestring_dataset) {
        return false;
    }
    
    // Create point dataset
    OGRCoordinateTransformationH coord_trans_point = nullptr;
    std::string point_file_path = generateSnappedPointsFilePath(config.output_file_path);
    GDALDatasetH point_dataset = createPointDataset(config, coord_trans_point, point_file_path);
    if (!point_dataset) {
        if (coord_trans_linestring) {
            OCTDestroyCoordinateTransformation(coord_trans_linestring);
        }
        GDALClose(linestring_dataset);
        return false;
    }
    
    // Get layers
    OGRLayerH linestring_layer = GDALDatasetGetLayer(linestring_dataset, 0);
    OGRLayerH point_layer = GDALDatasetGetLayer(point_dataset, 0);
    
    if (!linestring_layer || !point_layer) {
        last_error_ = "Failed to get layers from datasets";
        if (coord_trans_linestring) {
            OCTDestroyCoordinateTransformation(coord_trans_linestring);
        }
        if (coord_trans_point) {
            OCTDestroyCoordinateTransformation(coord_trans_point);
        }
        GDALClose(linestring_dataset);
        GDALClose(point_dataset);
        return false;
    }
    
    // Write features (combined approach for efficiency)
    bool success = writeLinestringAndPointFeatures(linestring_dataset, linestring_layer, coord_trans_linestring,
                                                  point_dataset, point_layer, coord_trans_point,
                                                  neighboring_points_results, neighboring_points_instance);
    
    // Cleanup
    if (coord_trans_linestring) {
        OCTDestroyCoordinateTransformation(coord_trans_linestring);
    }
    if (coord_trans_point) {
        OCTDestroyCoordinateTransformation(coord_trans_point);
    }
    GDALClose(linestring_dataset);
    GDALClose(point_dataset);
    
    return success;
}

GDALDatasetH NeighboringPointsWriter::createLinestringDataset(const NeighboringPointsWriterConfig& config, OGRCoordinateTransformationH& coord_trans, std::string& output_file_path) {
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
    
    // Create layer
    std::string layer_name = "neighboring_paths";
    OGRLayerH layer = GDALDatasetCreateLayer(dataset, layer_name.c_str(), layer_srs, wkbMultiLineString, nullptr);
    if (!layer) {
        last_error_ = "Failed to create layer in dataset";
        if (layer_srs) {
            OSRDestroySpatialReference(layer_srs);
        }
        GDALClose(dataset);
        if (coord_trans) {
            OCTDestroyCoordinateTransformation(coord_trans);
            coord_trans = nullptr;
        }
        return nullptr;
    }
    
    // Create fields
    OGRFieldDefnH path_id_field = OGR_Fld_Create("path_id", OFTInteger);
    OGR_L_CreateField(layer, path_id_field, 1);
    OGR_Fld_Destroy(path_id_field);
    
    OGRFieldDefnH source_point_id_field = OGR_Fld_Create("source_point_id", OFTInteger);
    OGR_L_CreateField(layer, source_point_id_field, 1);
    OGR_Fld_Destroy(source_point_id_field);
    
    OGRFieldDefnH target_point_id_field = OGR_Fld_Create("target_point_id", OFTInteger);
    OGR_L_CreateField(layer, target_point_id_field, 1);
    OGR_Fld_Destroy(target_point_id_field);
    
    OGRFieldDefnH distance_field = OGR_Fld_Create("distance", OFTReal);
    OGR_L_CreateField(layer, distance_field, 1);
    OGR_Fld_Destroy(distance_field);
    
    if (layer_srs) {
        OSRDestroySpatialReference(layer_srs);
    }
    
    return dataset;
}

GDALDatasetH NeighboringPointsWriter::createPointDataset(const NeighboringPointsWriterConfig& config, OGRCoordinateTransformationH& coord_trans, std::string& output_file_path) {
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
    
    // Create layer
    std::string layer_name = "snapped_points";
    OGRLayerH layer = GDALDatasetCreateLayer(dataset, layer_name.c_str(), layer_srs, wkbPoint, nullptr);
    if (!layer) {
        last_error_ = "Failed to create layer in dataset";
        if (layer_srs) {
            OSRDestroySpatialReference(layer_srs);
        }
        GDALClose(dataset);
        if (coord_trans) {
            OCTDestroyCoordinateTransformation(coord_trans);
            coord_trans = nullptr;
        }
        return nullptr;
    }
    
    // Create fields
    OGRFieldDefnH feature_id_field = OGR_Fld_Create("feature_id", OFTInteger);
    OGR_L_CreateField(layer, feature_id_field, 1);
    OGR_Fld_Destroy(feature_id_field);
    
    OGRFieldDefnH count_field = OGR_Fld_Create("count", OFTInteger);
    OGR_L_CreateField(layer, count_field, 1);
    OGR_Fld_Destroy(count_field);
    
    OGRFieldDefnH min_field = OGR_Fld_Create("min", OFTReal);
    OGR_L_CreateField(layer, min_field, 1);
    OGR_Fld_Destroy(min_field);
    
    OGRFieldDefnH max_field = OGR_Fld_Create("max", OFTReal);
    OGR_L_CreateField(layer, max_field, 1);
    OGR_Fld_Destroy(max_field);
    
    OGRFieldDefnH avg_field = OGR_Fld_Create("avg", OFTReal);
    OGR_L_CreateField(layer, avg_field, 1);
    OGR_Fld_Destroy(avg_field);
    
    if (layer_srs) {
        OSRDestroySpatialReference(layer_srs);
    }
    
    return dataset;
}

std::string NeighboringPointsWriter::generateSnappedPointsFilePath(const std::string& base_file_path) const {
    fs::path path(base_file_path);
    std::string stem = path.stem().string();
    std::string extension = path.extension().string();
    return (path.parent_path() / (stem + "_snapped_points" + extension)).string();
}


    


bool NeighboringPointsWriter::writeLinestringAndPointFeatures(GDALDatasetH linestring_dataset, OGRLayerH linestring_layer, 
                                                             OGRCoordinateTransformationH coord_trans_linestring,
                                                             GDALDatasetH point_dataset, OGRLayerH point_layer, 
                                                             OGRCoordinateTransformationH coord_trans_point,
                                                             const std::unordered_map<std::pair<size_t, size_t>, std::tuple<size_t, double, graph::MultiLineString>, graph::PairHash>& neighboring_points_results,
                                                             const graph::NeighboringPoints& neighboring_points_instance) {
    
    size_t path_id = 1;
    
    // Statistics maps for efficient point feature writing
    std::unordered_map<size_t, size_t> count_map;      // vertex_index -> count
    std::unordered_map<size_t, double> min_map;        // vertex_index -> min_distance
    std::unordered_map<size_t, double> max_map;        // vertex_index -> max_distance
    std::unordered_map<size_t, double> sum_map;        // vertex_index -> sum_distance
    
    // First pass: Write linestring features and collect statistics
    for (const auto& [vertex_pair, path_result] : neighboring_points_results) {
        size_t source_vertex_index = vertex_pair.first;   // Always the smaller vertex ID
        size_t target_vertex_index = vertex_pair.second;  // Always the larger vertex ID
        
        // Get feature IDs from vertex indices
        const auto& source_vertex = neighboring_points_instance.getVertex(source_vertex_index);
        const auto& target_vertex = neighboring_points_instance.getVertex(target_vertex_index);
        size_t source_feature_id = source_vertex.feature_id;
        size_t target_feature_id = target_vertex.feature_id;
        
        // Extract path information
        size_t actual_target_vertex = std::get<0>(path_result);  // This is the actual target from the path
        double distance = std::get<1>(path_result);
        const graph::MultiLineString& path_geometry = std::get<2>(path_result);
        
        // Write linestring feature
        OGRFeatureDefnH layer_defn = OGR_L_GetLayerDefn(linestring_layer);
        OGRFeatureH feature = OGR_F_Create(layer_defn);
        if (!feature) {
            last_error_ = "Failed to create OGR feature";
            return false;
        }
        
        // Set fields
        OGR_F_SetFieldInteger(feature, OGR_F_GetFieldIndex(feature, "path_id"), static_cast<int>(path_id));
        OGR_F_SetFieldInteger(feature, OGR_F_GetFieldIndex(feature, "source_point_id"), static_cast<int>(source_feature_id));
        OGR_F_SetFieldInteger(feature, OGR_F_GetFieldIndex(feature, "target_point_id"), static_cast<int>(target_feature_id));
        OGR_F_SetFieldDouble(feature, OGR_F_GetFieldIndex(feature, "distance"), distance);
        
        // Set geometry - always create multilinestring for consistency
        OGRGeometryH ogr_geometry = OGR_G_CreateGeometry(wkbMultiLineString);
        for (const auto& linestring : path_geometry) {
            OGRGeometryH ogr_linestring = OGR_G_CreateGeometry(wkbLineString);
            for (const auto& point : linestring) {
                OGR_G_AddPoint(ogr_linestring, bg::get<0>(point), bg::get<1>(point), 0.0);
            }
            OGR_G_AddGeometry(ogr_geometry, ogr_linestring);
        }
        
        // Apply coordinate transformation if needed
        if (coord_trans_linestring && ogr_geometry) {
            OGR_G_Transform(ogr_geometry, coord_trans_linestring);
        }
        
        OGR_F_SetGeometry(feature, ogr_geometry);
        
        // Create feature in layer
        if (OGR_L_CreateFeature(linestring_layer, feature) != OGRERR_NONE) {
            last_error_ = "Failed to create feature in linestring layer";
            OGR_F_Destroy(feature);
            return false;
        }
        
        // Cleanup
        OGR_F_Destroy(feature);
        
        // Update statistics for both source and target vertices
        updateVertexStatistics(source_vertex_index, distance, count_map, min_map, max_map, sum_map);
        updateVertexStatistics(target_vertex_index, distance, count_map, min_map, max_map, sum_map);
        
        path_id++;
    }
    
    // Second pass: Write point features using pre-computed statistics
    // We need to write points for all point vertices, not just those with paths
    // Get all point vertices from the neighboring points instance
    std::vector<size_t> point_vertices = neighboring_points_instance.getPointVertices();
    
    for (size_t vertex_index : point_vertices) {
        // Get vertex information
        const auto& source_vertex = neighboring_points_instance.getVertex(vertex_index);
        size_t feature_id = source_vertex.feature_id;
        const auto& point_geometry = source_vertex.geometry;
        
        // Get statistics from maps (default to 0 if no paths found)
        size_t count = count_map[vertex_index];
        double min_distance = (count > 0) ? min_map[vertex_index] : 0.0;
        double max_distance = (count > 0) ? max_map[vertex_index] : 0.0;
        double sum_distance = (count > 0) ? sum_map[vertex_index] : 0.0;
        double avg_distance = (count > 0) ? sum_distance / count : 0.0;
        
        // Create feature
        OGRFeatureDefnH point_layer_defn = OGR_L_GetLayerDefn(point_layer);
        OGRFeatureH feature = OGR_F_Create(point_layer_defn);
        if (!feature) {
            last_error_ = "Failed to create OGR feature";
            return false;
        }
        
        // Set fields
        OGR_F_SetFieldInteger(feature, OGR_F_GetFieldIndex(feature, "feature_id"), static_cast<int>(feature_id));
        OGR_F_SetFieldInteger(feature, OGR_F_GetFieldIndex(feature, "count"), static_cast<int>(count));
        OGR_F_SetFieldDouble(feature, OGR_F_GetFieldIndex(feature, "min"), min_distance);
        OGR_F_SetFieldDouble(feature, OGR_F_GetFieldIndex(feature, "max"), max_distance);
        OGR_F_SetFieldDouble(feature, OGR_F_GetFieldIndex(feature, "avg"), avg_distance);
        
        // Set geometry (point)
        OGRGeometryH ogr_point = OGR_G_CreateGeometry(wkbPoint);
        OGR_G_SetPoint(ogr_point, 0, bg::get<0>(point_geometry), bg::get<1>(point_geometry), 0.0);
        
        // Apply coordinate transformation if needed
        if (coord_trans_point) {
            OGR_G_Transform(ogr_point, coord_trans_point);
        }
        
        OGR_F_SetGeometry(feature, ogr_point);
        
        // Create feature in layer
        if (OGR_L_CreateFeature(point_layer, feature) != OGRERR_NONE) {
            last_error_ = "Failed to create feature in point layer";
            OGR_F_Destroy(feature);
            return false;
        }
        
        // Cleanup
        OGR_F_Destroy(feature);
    }
    
    return true;
}

void NeighboringPointsWriter::updateVertexStatistics(size_t vertex_index, double distance,
                                                   std::unordered_map<size_t, size_t>& count_map,
                                                   std::unordered_map<size_t, double>& min_map,
                                                   std::unordered_map<size_t, double>& max_map,
                                                   std::unordered_map<size_t, double>& sum_map) {
    
    // Update count
    count_map[vertex_index]++;
    
    // Update min
    if (min_map.find(vertex_index) == min_map.end() || distance < min_map[vertex_index]) {
        min_map[vertex_index] = distance;
    }
    
    // Update max
    if (max_map.find(vertex_index) == max_map.end() || distance > max_map[vertex_index]) {
        max_map[vertex_index] = distance;
    }
    
    // Update sum
    sum_map[vertex_index] += distance;
}

} // namespace io
} // namespace adjfind
