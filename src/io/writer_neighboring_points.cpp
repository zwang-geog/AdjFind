#include "io/writer_neighboring_points.hpp"
#include "graph/neighboring_points.hpp"
#include <gdal.h>
#include <gdal_priv.h>
#include <ogr_feature.h>
#include <ogr_geometry.h>
#include <iostream>
#include <filesystem>
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
    OGRCoordinateTransformation* coord_trans_linestring = nullptr;
    std::string linestring_file_path = config.output_file_path;
    void* linestring_dataset = createLinestringDataset(config, coord_trans_linestring, linestring_file_path);
    if (!linestring_dataset) {
        return false;
    }
    
    // Create point dataset
    OGRCoordinateTransformation* coord_trans_point = nullptr;
    std::string point_file_path = generateSnappedPointsFilePath(config.output_file_path);
    void* point_dataset = createPointDataset(config, coord_trans_point, point_file_path);
    if (!point_dataset) {
        if (coord_trans_linestring) {
            OCTDestroyCoordinateTransformation(coord_trans_linestring);
        }
        GDALClose(linestring_dataset);
        return false;
    }
    
    // Get layers
    OGRLayer* linestring_layer = static_cast<GDALDataset*>(linestring_dataset)->GetLayer(0);
    OGRLayer* point_layer = static_cast<GDALDataset*>(point_dataset)->GetLayer(0);
    
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

void* NeighboringPointsWriter::createLinestringDataset(const NeighboringPointsWriterConfig& config, OGRCoordinateTransformation*& coord_trans, std::string& output_file_path) {
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
    
    // Create layer
    std::string layer_name = "neighboring_paths";
    OGRLayer* layer = dataset->CreateLayer(layer_name.c_str(), layer_srs, wkbMultiLineString, nullptr);
    if (!layer) {
        last_error_ = "Failed to create layer in dataset";
        if (layer_srs && layer_srs != &wgs84_srs) {
            delete layer_srs;
        }
        GDALClose(dataset);
        if (coord_trans) {
            OCTDestroyCoordinateTransformation(coord_trans);
            coord_trans = nullptr;
        }
        return nullptr;
    }
    
    // Create fields
    OGRFieldDefn path_id_field("path_id", OFTInteger);
    layer->CreateField(&path_id_field);
    
    OGRFieldDefn source_point_id_field("source_point_id", OFTInteger);
    layer->CreateField(&source_point_id_field);
    
    OGRFieldDefn target_point_id_field("target_point_id", OFTInteger);
    layer->CreateField(&target_point_id_field);
    
    OGRFieldDefn distance_field("distance", OFTReal);
    layer->CreateField(&distance_field);
    
    if (layer_srs && layer_srs != &wgs84_srs) {
        delete layer_srs;
    }
    
    return dataset;
}

void* NeighboringPointsWriter::createPointDataset(const NeighboringPointsWriterConfig& config, OGRCoordinateTransformation*& coord_trans, std::string& output_file_path) {
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
    
    // Create layer
    std::string layer_name = "snapped_points";
    OGRLayer* layer = dataset->CreateLayer(layer_name.c_str(), layer_srs, wkbPoint, nullptr);
    if (!layer) {
        last_error_ = "Failed to create layer in dataset";
        if (layer_srs && layer_srs != &wgs84_srs) {
            delete layer_srs;
        }
        GDALClose(dataset);
        if (coord_trans) {
            OCTDestroyCoordinateTransformation(coord_trans);
            coord_trans = nullptr;
        }
        return nullptr;
    }
    
    // Create fields
    OGRFieldDefn feature_id_field("feature_id", OFTInteger);
    layer->CreateField(&feature_id_field);
    
    OGRFieldDefn count_field("count", OFTInteger);
    layer->CreateField(&count_field);
    
    OGRFieldDefn min_field("min", OFTReal);
    layer->CreateField(&min_field);
    
    OGRFieldDefn max_field("max", OFTReal);
    layer->CreateField(&max_field);
    
    OGRFieldDefn avg_field("avg", OFTReal);
    layer->CreateField(&avg_field);
    
    if (layer_srs && layer_srs != &wgs84_srs) {
        delete layer_srs;
    }
    
    return dataset;
}

std::string NeighboringPointsWriter::generateSnappedPointsFilePath(const std::string& base_file_path) const {
    std::filesystem::path path(base_file_path);
    std::string stem = path.stem().string();
    std::string extension = path.extension().string();
    return (path.parent_path() / (stem + "_snapped_points" + extension)).string();
}


    


bool NeighboringPointsWriter::writeLinestringAndPointFeatures(void* linestring_dataset, void* linestring_layer, 
                                                             OGRCoordinateTransformation* coord_trans_linestring,
                                                             void* point_dataset, void* point_layer, 
                                                             OGRCoordinateTransformation* coord_trans_point,
                                                             const std::unordered_map<std::pair<size_t, size_t>, std::tuple<size_t, double, graph::MultiLineString>, graph::PairHash>& neighboring_points_results,
                                                             const graph::NeighboringPoints& neighboring_points_instance) {
    
    OGRLayer* ogr_linestring_layer = static_cast<OGRLayer*>(linestring_layer);
    OGRLayer* ogr_point_layer = static_cast<OGRLayer*>(point_layer);
    
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
        OGRFeature* feature = OGRFeature::CreateFeature(ogr_linestring_layer->GetLayerDefn());
        if (!feature) {
            last_error_ = "Failed to create OGR feature";
            return false;
        }
        
        // Set fields
        feature->SetField("path_id", static_cast<int>(path_id));
        feature->SetField("source_point_id", static_cast<int>(source_feature_id));
        feature->SetField("target_point_id", static_cast<int>(target_feature_id));
        feature->SetField("distance", distance);
        
        // Set geometry - always create multilinestring for consistency
        OGRGeometry* ogr_geometry = new OGRMultiLineString();
        for (const auto& linestring : path_geometry) {
            OGRLineString* ogr_linestring = new OGRLineString();
            for (const auto& point : linestring) {
                ogr_linestring->addPoint(bg::get<0>(point), bg::get<1>(point));
            }
            static_cast<OGRMultiLineString*>(ogr_geometry)->addGeometry(ogr_linestring);
            delete ogr_linestring;
        }
        
        // Apply coordinate transformation if needed
        if (coord_trans_linestring && ogr_geometry) {
            ogr_geometry->transform(coord_trans_linestring);
        }
        
        feature->SetGeometry(ogr_geometry);
        
        // Create feature in layer
        if (ogr_linestring_layer->CreateFeature(feature) != OGRERR_NONE) {
            last_error_ = "Failed to create feature in linestring layer";
            OGRFeature::DestroyFeature(feature);
            if (ogr_geometry) {
                OGRGeometryFactory::destroyGeometry(ogr_geometry);
            }
            return false;
        }
        
        // Cleanup
        OGRFeature::DestroyFeature(feature);
        if (ogr_geometry) {
            OGRGeometryFactory::destroyGeometry(ogr_geometry);
        }
        
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
        OGRFeature* feature = OGRFeature::CreateFeature(ogr_point_layer->GetLayerDefn());
        if (!feature) {
            last_error_ = "Failed to create OGR feature";
            return false;
        }
        
        // Set fields
        feature->SetField("feature_id", static_cast<int>(feature_id));
        feature->SetField("count", static_cast<int>(count));
        feature->SetField("min", min_distance);
        feature->SetField("max", max_distance);
        feature->SetField("avg", avg_distance);
        
        // Set geometry (point)
        OGRPoint* ogr_point = new OGRPoint(bg::get<0>(point_geometry), bg::get<1>(point_geometry));
        
        // Apply coordinate transformation if needed
        if (coord_trans_point) {
            ogr_point->transform(coord_trans_point);
        }
        
        feature->SetGeometry(ogr_point);
        
        // Create feature in layer
        if (ogr_point_layer->CreateFeature(feature) != OGRERR_NONE) {
            last_error_ = "Failed to create feature in point layer";
            OGRFeature::DestroyFeature(feature);
            delete ogr_point;
            return false;
        }
        
        // Cleanup
        OGRFeature::DestroyFeature(feature);
        delete ogr_point;
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
