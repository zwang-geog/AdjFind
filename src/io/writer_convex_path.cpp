#include "io/writer_convex_path.hpp"
#include "io/geojson_writer.hpp"
#include "graph/common.hpp"
#include <iostream>
#include <filesystem>
#include <algorithm>
#include <sstream>
#include <boost/geometry.hpp>

namespace adjfind {
namespace io {

// Boost Geometry namespace alias
namespace bg = boost::geometry;

// Constructor and destructor are now defaulted in header

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

std::vector<graph::GeospatialFeature> ConvexPathWriter::convexPathResultsToLinestringFeatures(
    const std::vector<graph::ConvexPathResult>& vertex_paths, 
    size_t polygon_feature_id,
    const graph::Point& least_accessible_point,
    size_t start_feature_id) {
    
    std::vector<graph::GeospatialFeature> features;
    features.reserve(vertex_paths.size());
    
    for (size_t i = 0; i < vertex_paths.size(); ++i) {
        const auto& vertex_path = vertex_paths[i];
        
        // Convert geometry to GeoJSON
        nlohmann::json geometry = graph::lineStringToGeoJSON(vertex_path.path_geometry);
        
        // Create properties
        nlohmann::json properties = nlohmann::json::object();
        properties["polygon_feature_id"] = polygon_feature_id;
        properties["path_type"] = pathTypeToString(vertex_path.start_point_type);
        
        // assigned_point_feature_id - only add if valid
        if (vertex_path.path_found && vertex_path.nearest_point_vertex_position_index != std::numeric_limits<size_t>::max()) {
            properties["assigned_point_feature_id"] = vertex_path.nearest_point_vertex_position_index;
        }
        
        // snapped_road_feature_id - only add if valid
        if (vertex_path.path_found && vertex_path.edge_index != std::numeric_limits<size_t>::max()) {
            properties["snapped_road_feature_id"] = vertex_path.edge_index;
        }
        
        // distance_to_assigned_point - only add if path found
        if (vertex_path.path_found) {
            properties["distance_to_assigned_point"] = vertex_path.total_length;
        }
        
        // access_distance - only add if path found and geometry exists
        if (vertex_path.path_found && !vertex_path.path_geometry.empty()) {
            double access_distance = bg::length(vertex_path.path_geometry);
            properties["access_distance"] = access_distance;
        }
        
        // Create feature
        graph::GeospatialFeature feature;
        feature.id = start_feature_id + i;
        feature.geometry = geometry;
        feature.properties = properties;
        
        features.push_back(feature);
    }
    
    return features;
}

graph::GeospatialFeature ConvexPathWriter::leastAccessiblePointToGeoJSONFeature(
    const std::vector<graph::ConvexPathResult>& vertex_paths,
    size_t polygon_feature_id,
    const graph::Point& least_accessible_point,
    size_t feature_id) {
    
    // Convert geometry to GeoJSON
    nlohmann::json geometry = graph::pointToGeoJSON(least_accessible_point);
    
    // Create properties
    nlohmann::json properties = nlohmann::json::object();
    properties["polygon_feature_id"] = polygon_feature_id;
    
    // Find the LEAST_ACCESSIBLE_POINT path result to extract attributes
    for (const auto& vertex_path : vertex_paths) {
        if (vertex_path.start_point_type == graph::ConvexPathResultType::LEAST_ACCESSIBLE_POINT) {
            // assigned_point_feature_id
            if (vertex_path.path_found && vertex_path.nearest_point_vertex_position_index != std::numeric_limits<size_t>::max()) {
                properties["assigned_point_feature_id"] = vertex_path.nearest_point_vertex_position_index;
            }
            
            // snapped_road_feature_id
            if (vertex_path.path_found && vertex_path.edge_index != std::numeric_limits<size_t>::max()) {
                properties["snapped_road_feature_id"] = vertex_path.edge_index;
            }
            
            // distance_to_assigned_point
            if (vertex_path.path_found) {
                properties["distance_to_assigned_point"] = vertex_path.total_length;
            }
            
            // access_distance
            if (vertex_path.path_found && !vertex_path.path_geometry.empty()) {
                double access_distance = bg::length(vertex_path.path_geometry);
                properties["access_distance"] = access_distance;
            }
            
            break; // Found the LEAST_ACCESSIBLE_POINT, no need to continue
        }
    }
    
    // Create feature
    graph::GeospatialFeature feature;
    feature.id = feature_id;
    feature.geometry = geometry;
    feature.properties = properties;
    
    return feature;
}

bool ConvexPathWriter::writeConvexPathResults(const ConvexPathWriterConfig& config, 
                                             const std::vector<std::tuple<std::vector<graph::ConvexPathResult>, size_t, graph::Point>>& polygon_results) {
    clearError();
    
    if (polygon_results.empty()) {
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
        size_t linestring_feature_id = 0;
        size_t point_feature_id = 0;
        
        for (const auto& polygon_result : polygon_results) {
            const auto& [vertex_paths, polygon_feature_id, least_accessible_point] = polygon_result;
            
            // Convert linestring features
            auto linestring_features = convexPathResultsToLinestringFeatures(
                vertex_paths, polygon_feature_id, least_accessible_point, linestring_feature_id);
            
            // Add to linestring dataset
            linestring_dataset.features.insert(
                linestring_dataset.features.end(), 
                linestring_features.begin(), 
                linestring_features.end());
            
            // Convert point feature
            auto point_feature = leastAccessiblePointToGeoJSONFeature(
                vertex_paths, polygon_feature_id, least_accessible_point, point_feature_id);
            
            // Add to point dataset
            point_dataset.features.push_back(point_feature);
            
            // Update feature IDs
            linestring_feature_id += vertex_paths.size();
            point_feature_id++;
        }
        
        // Write datasets to files
        GeoJSONWriter::writeToFile(linestring_dataset, config.linestring_output_file_path);
        GeoJSONWriter::writeToFile(point_dataset, config.point_output_file_path);
        
        std::cout << "Successfully wrote convex path results:" << std::endl;
        std::cout << "  Linestring dataset: " << config.linestring_output_file_path << std::endl;
        std::cout << "  Point dataset: " << config.point_output_file_path << std::endl;
        std::cout << "  Total features written: " << linestring_feature_id << " linestrings, " << point_feature_id << " points" << std::endl;
        
        return true;
        
    } catch (const std::exception& e) {
        last_error_ = "Failed to write convex path results: " + std::string(e.what());
        std::cerr << last_error_ << std::endl;
        return false;
    }
}

} // namespace io
} // namespace adjfind