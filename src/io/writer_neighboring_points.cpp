#include "io/writer_neighboring_points.hpp"
#include "io/geojson_writer.hpp"
#include "graph/common.hpp"
#include "graph/neighboring_points.hpp"
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

// Constructor and destructor are now defaulted in header

void NeighboringPointsWriter::updateVertexStatistics(size_t vertex_index, double distance,
                                                    std::unordered_map<size_t, size_t>& count_map,
                                                    std::unordered_map<size_t, double>& min_map,
                                                    std::unordered_map<size_t, double>& max_map,
                                                    std::unordered_map<size_t, double>& sum_map) {
    count_map[vertex_index]++;
    sum_map[vertex_index] += distance;
    
    // Update min
    if (min_map.find(vertex_index) == min_map.end() || distance < min_map[vertex_index]) {
        min_map[vertex_index] = distance;
    }
    
    // Update max
    if (max_map.find(vertex_index) == max_map.end() || distance > max_map[vertex_index]) {
        max_map[vertex_index] = distance;
    }
}


std::vector<graph::GeospatialFeature> NeighboringPointsWriter::neighboringPointsResultsToLinestringFeatures(
    const std::unordered_map<std::pair<size_t, size_t>, std::tuple<size_t, double, graph::MultiLineString>, graph::PairHash>& neighboring_points_results,
    const graph::NeighboringPoints& neighboring_points_instance) {
    
    std::vector<graph::GeospatialFeature> features;
    features.reserve(neighboring_points_results.size());
    
    size_t path_id = 1;
    
    for (const auto& [vertex_pair, path_data] : neighboring_points_results) {
        size_t source_vertex_index = vertex_pair.first;   // Always the smaller vertex ID
        size_t target_vertex_index = vertex_pair.second;  // Always the larger vertex ID
        
        // Get feature IDs from vertex indices
        const auto& source_vertex = neighboring_points_instance.getVertex(source_vertex_index);
        const auto& target_vertex = neighboring_points_instance.getVertex(target_vertex_index);
        size_t source_feature_id = source_vertex.feature_id;
        size_t target_feature_id = target_vertex.feature_id;
        
        // Extract path information
        size_t actual_target_vertex = std::get<0>(path_data);  // This is the actual target from the path
        double distance = std::get<1>(path_data);
        const graph::MultiLineString& path_geometry = std::get<2>(path_data);
        
        // Convert geometry to GeoJSON
        nlohmann::json geometry = graph::multiLineStringToGeoJSON(path_geometry);
        
        // Create properties (matching original GDAL implementation)
        nlohmann::json properties = nlohmann::json::object();
        properties["path_id"] = path_id;
        properties["source_point_id"] = source_feature_id;
        properties["target_point_id"] = target_feature_id;
        properties["distance"] = distance;
        
        // Create feature
        graph::GeospatialFeature feature;
        feature.id = path_id;
        feature.geometry = geometry;
        feature.properties = properties;
        
        features.push_back(feature);
        path_id++;
    }
    
    return features;
}

std::vector<graph::GeospatialFeature> NeighboringPointsWriter::neighboringPointsResultsToPointFeatures(
    const std::unordered_map<std::pair<size_t, size_t>, std::tuple<size_t, double, graph::MultiLineString>, graph::PairHash>& neighboring_points_results,
    const graph::NeighboringPoints& neighboring_points_instance) {
    
    std::vector<graph::GeospatialFeature> features;
    
    // Statistics maps for efficient point feature writing
    std::unordered_map<size_t, size_t> count_map;      // vertex_index -> count
    std::unordered_map<size_t, double> min_map;        // vertex_index -> min_distance
    std::unordered_map<size_t, double> max_map;        // vertex_index -> max_distance
    std::unordered_map<size_t, double> sum_map;        // vertex_index -> sum_distance
    
    // First pass: Collect statistics from all paths
    for (const auto& [vertex_pair, path_data] : neighboring_points_results) {
        size_t source_vertex_index = vertex_pair.first;   // Always the smaller vertex ID
        size_t target_vertex_index = vertex_pair.second;  // Always the larger vertex ID
        
        // Extract path information
        double distance = std::get<1>(path_data);
        
        // Update statistics for both source and target vertices
        updateVertexStatistics(source_vertex_index, distance, count_map, min_map, max_map, sum_map);
        updateVertexStatistics(target_vertex_index, distance, count_map, min_map, max_map, sum_map);
    }
    
    // Second pass: Write point features using pre-computed statistics
    // We need to write points for all point vertices, not just those with paths
    // Get all point vertices from the neighboring points instance
    std::vector<size_t> point_vertices = neighboring_points_instance.getPointVertices();
    
    features.reserve(point_vertices.size());
    
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
        
        // Convert 3D geometry to 2D for GeoJSON
        graph::Point point_2d(graph::bg::get<0>(point_geometry), graph::bg::get<1>(point_geometry));
        nlohmann::json geometry = graph::pointToGeoJSON(point_2d);
        
        // Create properties (matching original GDAL implementation)
        nlohmann::json properties = nlohmann::json::object();
        properties["feature_id"] = feature_id;
        properties["count"] = count;
        properties["min"] = min_distance;
        properties["max"] = max_distance;
        properties["avg"] = avg_distance;
        
        // Create feature
        graph::GeospatialFeature feature;
        feature.id = feature_id;
        feature.geometry = geometry;
        feature.properties = properties;
        
        features.push_back(feature);
    }
    
    return features;
}

bool NeighboringPointsWriter::writeNeighboringPointsResults(const NeighboringPointsWriterConfig& config, 
                                                           const std::unordered_map<std::pair<size_t, size_t>, std::tuple<size_t, double, graph::MultiLineString>, graph::PairHash>& neighboring_points_results,
                                                           const graph::NeighboringPoints& neighboring_points_instance) {
    clearError();
    
    if (neighboring_points_results.empty()) {
        last_error_ = "No results to write";
        return false;
    }
    
    try {
        // Create linestring dataset
        graph::GeospatialDataset linestring_dataset;
        linestring_dataset.crs = config.crs;
        
        // Create point dataset
        graph::GeospatialDataset point_dataset;
        point_dataset.crs = config.crs;
        
        // Convert results to GeoJSON features
        linestring_dataset.features = neighboringPointsResultsToLinestringFeatures(
            neighboring_points_results, neighboring_points_instance);
        
        point_dataset.features = neighboringPointsResultsToPointFeatures(
            neighboring_points_results, neighboring_points_instance);
        
        // Write datasets to files
        GeoJSONWriter::writeToFile(linestring_dataset, config.linestring_output_file_path);
        GeoJSONWriter::writeToFile(point_dataset, config.point_output_file_path);
        
        std::cout << "Successfully wrote neighboring points results:" << std::endl;
        std::cout << "  Linestring dataset: " << config.linestring_output_file_path << std::endl;
        std::cout << "  Point dataset: " << config.point_output_file_path << std::endl;
        std::cout << "  Total features written: " << linestring_dataset.features.size() 
                  << " linestrings, " << point_dataset.features.size() << " points" << std::endl;
        
        return true;
        
    } catch (const std::exception& e) {
        last_error_ = "Failed to write neighboring points results: " + std::string(e.what());
        std::cerr << last_error_ << std::endl;
        return false;
    }
}

} // namespace io
} // namespace adjfind