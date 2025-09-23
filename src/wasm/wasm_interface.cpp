#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>
#include <memory>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <cstring>
#include <nlohmann/json.hpp>

// Define EMSCRIPTEN_KEEPALIVE if not already defined
#ifndef EMSCRIPTEN_KEEPALIVE
#define EMSCRIPTEN_KEEPALIVE
#endif

// Include adjfind headers
#include "io/road_reader.hpp"
#include "io/point_reader.hpp"
#include "io/polygon_reader.hpp"
#include "io/writer_road_segmentation.hpp"
#include "io/writer_convex_path.hpp"
#include "io/writer_neighboring_points.hpp"
#include "io/geojson_reader.hpp"
#include "io/geojson_writer.hpp"
#include "graph/adj_graph.hpp"
#include "graph/road_segmentation.hpp"
#include "graph/convex_path.hpp"
#include "graph/neighboring_points.hpp"

using namespace adjfind;

namespace wasm_interface {

// Helper function to parse RoadReaderConfig from JSON
io::RoadReaderConfig parseRoadReaderConfig(const nlohmann::json& config_json) {
    io::RoadReaderConfig config;
    
    if (config_json.contains("file_path")) {
        config.file_path = config_json["file_path"];
    }
    if (config_json.contains("id_field")) {
        config.id_field = config_json["id_field"];
    }
    if (config_json.contains("from_z_field")) {
        config.from_z_field = config_json["from_z_field"];
    }
    if (config_json.contains("to_z_field")) {
        config.to_z_field = config_json["to_z_field"];
    }
    if (config_json.contains("length_field")) {
        config.length_field = config_json["length_field"];
    }
    
    return config;
}

// Helper function to parse PointReaderConfig from JSON
io::PointReaderConfig parsePointReaderConfig(const nlohmann::json& config_json) {
    io::PointReaderConfig config;
    
    if (config_json.contains("file_path")) {
        config.file_path = config_json["file_path"];
    }
    if (config_json.contains("id_field")) {
        config.id_field = config_json["id_field"];
    }
    
    return config;
}

// Helper function to parse PolygonReaderConfig from JSON
io::PolygonReaderConfig parsePolygonReaderConfig(const nlohmann::json& config_json) {
    io::PolygonReaderConfig config;
    
    if (config_json.contains("file_path")) {
        config.file_path = config_json["file_path"];
    }
    if (config_json.contains("id_field")) {
        config.id_field = config_json["id_field"];
    }
    if (config_json.contains("is_obstacle_only_field")) {
        config.is_obstacle_only_field = config_json["is_obstacle_only_field"];
    }
    if (config_json.contains("road_ids_snappable_field")) {
        config.road_ids_snappable_field = config_json["road_ids_snappable_field"];
    }
    if (config_json.contains("min_polygon_boundary_segment_length_for_nearest_road_edge_detection")) {
        config.min_polygon_boundary_segment_length_for_nearest_road_edge_detection = config_json["min_polygon_boundary_segment_length_for_nearest_road_edge_detection"];
    }
    
    return config;
}

// Helper function to parse RoadSegmentationWriterConfig from JSON
io::RoadSegmentationWriterConfig parseRoadSegmentationWriterConfig(const nlohmann::json& config_json) {
    io::RoadSegmentationWriterConfig config;
    
    if (config_json.contains("output_file_path")) {
        config.output_file_path = config_json["output_file_path"];
    }
    if (config_json.contains("crs")) {
        config.crs = config_json["crs"];
    }
    
    return config;
}

// Helper function to parse ConvexPathWriterConfig from JSON
io::ConvexPathWriterConfig parseConvexPathWriterConfig(const nlohmann::json& config_json) {
    io::ConvexPathWriterConfig config;
    
    if (config_json.contains("linestring_output_file_path")) {
        config.linestring_output_file_path = config_json["linestring_output_file_path"];
    }
    if (config_json.contains("point_output_file_path")) {
        config.point_output_file_path = config_json["point_output_file_path"];
    }
    if (config_json.contains("crs")) {
        config.crs = config_json["crs"];
    }
    
    return config;
}

// Helper function to parse NeighboringPointsWriterConfig from JSON
io::NeighboringPointsWriterConfig parseNeighboringPointsWriterConfig(const nlohmann::json& config_json) {
    io::NeighboringPointsWriterConfig config;
    
    if (config_json.contains("linestring_output_file_path")) {
        config.linestring_output_file_path = config_json["linestring_output_file_path"];
    }
    if (config_json.contains("point_output_file_path")) {
        config.point_output_file_path = config_json["point_output_file_path"];
    }
    if (config_json.contains("crs")) {
        config.crs = config_json["crs"];
    }
    
    return config;
}

// Road Segmentation Tool
std::string processRoadSegmentationTool(
    const std::string& writer_config_json,
    const std::string& road_config_json,
    const std::string& point_config_json,
    const std::string& road_segmentation_config_json) {
    
    try {
        // Parse configurations
        nlohmann::json writer_config = nlohmann::json::parse(writer_config_json);
        nlohmann::json road_config = nlohmann::json::parse(road_config_json);
        nlohmann::json point_config = nlohmann::json::parse(point_config_json);
        nlohmann::json road_segmentation_config = nlohmann::json::parse(road_segmentation_config_json);
        
        // Create configurations
        io::RoadReaderConfig road_cfg = parseRoadReaderConfig(road_config);
        io::PointReaderConfig point_cfg = parsePointReaderConfig(point_config);
        
        // Extract distance breakpoints string
        std::string distance_breakpoints_str = road_segmentation_config.value("distance_breakpoints_str", "");
        
        // Create road segmentation instance
        graph::RoadSegmentation road_segmentation;
        
        // Process road segmentation
        std::vector<graph::RoadSplitByDistanceBracketsOutput> results = 
            road_segmentation.processRoadSegmentationMode(road_cfg, point_cfg, distance_breakpoints_str);
        
        if (results.empty()) {
            return "Error: No results generated from road segmentation";
        }
        
        // Get CRS from road segmentation and add to writer config
        std::string crs = road_segmentation.getCoordinateSystemCRS();
        writer_config["crs"] = crs;
        io::RoadSegmentationWriterConfig writer_cfg = parseRoadSegmentationWriterConfig(writer_config);
        
        // Write results
        io::RoadSegmentationWriter writer;
        if (!writer.writeRoadSegmentationResults(writer_cfg, results)) {
            return "Error: Failed to write road segmentation results: " + writer.getLastError();
        }
        
        return "Success: Road segmentation completed with " + std::to_string(results.size()) + " segments";
        
    } catch (const std::exception& e) {
        return "Error: " + std::string(e.what());
    }
}

// Convex Path Tool (Structure Access)
std::string processConvexPathTool(
    const std::string& writer_config_json,
    const std::string& road_config_json,
    const std::string& point_config_json,
    const std::string& building_config_json) {
    
    try {
        // Parse configurations
        nlohmann::json writer_config = nlohmann::json::parse(writer_config_json);
        nlohmann::json road_config = nlohmann::json::parse(road_config_json);
        nlohmann::json point_config = nlohmann::json::parse(point_config_json);
        nlohmann::json building_config = nlohmann::json::parse(building_config_json);
        
        // Create configurations
        io::RoadReaderConfig road_cfg = parseRoadReaderConfig(road_config);
        io::PointReaderConfig point_cfg = parsePointReaderConfig(point_config);
        io::PolygonReaderConfig building_cfg = parsePolygonReaderConfig(building_config);
        
        // Create convex path instance
        graph::ConvexPath convex_path;
        
        // Process convex path
        if (!convex_path.processConvexPathMode(road_cfg, point_cfg, building_cfg)) {
            return "Error: Failed to process convex path analysis";
        }
        
        // Get results from convex path
        const auto& results = convex_path.getPolygonResults();
        
        if (results.empty()) {
            return "Error: No results generated from convex path analysis";
        }
        
        // Get CRS from convex path and add to writer config
        std::string crs = convex_path.getCoordinateSystemCRS();
        writer_config["crs"] = crs;
        io::ConvexPathWriterConfig writer_cfg = parseConvexPathWriterConfig(writer_config);
        
        // Write results
        io::ConvexPathWriter writer;
        if (!writer.writeConvexPathResults(writer_cfg, results)) {
            return "Error: Failed to write convex path results: " + writer.getLastError();
        }
        
        return "Success: Convex path analysis completed with " + std::to_string(results.size()) + " results";
        
    } catch (const std::exception& e) {
        return "Error: " + std::string(e.what());
    }
}

// Neighboring Points Tool
std::string processNeighboringPointsTool(
    const std::string& writer_config_json,
    const std::string& road_config_json,
    const std::string& point_config_json,
    const std::string& neighboring_points_config_json) {
    
    try {
        // Parse configurations
        nlohmann::json writer_config = nlohmann::json::parse(writer_config_json);
        nlohmann::json road_config = nlohmann::json::parse(road_config_json);
        nlohmann::json point_config = nlohmann::json::parse(point_config_json);
        nlohmann::json neighboring_points_config = nlohmann::json::parse(neighboring_points_config_json);
        
        // Create configurations
        io::RoadReaderConfig road_cfg = parseRoadReaderConfig(road_config);
        io::PointReaderConfig point_cfg = parsePointReaderConfig(point_config);
        
        // Extract neighboring points parameters
        double intersection_vertex_distance_threshold = neighboring_points_config.value("intersection_vertex_distance_threshold", 60.0);
        
        // Create neighboring points instance
        graph::NeighboringPoints neighboring_points;
        
        // Process neighboring points with optional cutoff
        bool success;
        if (neighboring_points_config.contains("cutoff")) {
            double cutoff = neighboring_points_config["cutoff"];
            success = neighboring_points.processNeighboringPoints(road_cfg, point_cfg, intersection_vertex_distance_threshold, cutoff);
        } else {
            success = neighboring_points.processNeighboringPoints(road_cfg, point_cfg, intersection_vertex_distance_threshold);
        }
        
        if (!success) {
            return "Error: Failed to process neighboring points analysis";
        }
        
        // Get results from neighboring points
        const auto& results = neighboring_points.getNeighboringPointsResults();
        
        if (results.empty()) {
            return "Error: No results generated from neighboring points analysis";
        }
        
        // Get CRS from neighboring points and add to writer config
        std::string crs = neighboring_points.getCoordinateSystemCRS();
        writer_config["crs"] = crs;
        io::NeighboringPointsWriterConfig writer_cfg = parseNeighboringPointsWriterConfig(writer_config);
        
        // Write results
        io::NeighboringPointsWriter writer;
        if (!writer.writeNeighboringPointsResults(writer_cfg, results, neighboring_points)) {
            return "Error: Failed to write neighboring points results: " + writer.getLastError();
        }
        
        return "Success: Neighboring points analysis completed with " + std::to_string(results.size()) + " paths";
        
    } catch (const std::exception& e) {
        return "Error: " + std::string(e.what());
    }
}

} // namespace wasm_interface

// Export functions for WASM
extern "C" {
    EMSCRIPTEN_KEEPALIVE
    const char* processRoadSegmentationTool(
        const char* writer_config_json,
        const char* road_config_json,
        const char* point_config_json,
        const char* road_segmentation_config_json
    ) {
        std::string result = wasm_interface::processRoadSegmentationTool(
            std::string(writer_config_json),
            std::string(road_config_json),
            std::string(point_config_json),
            std::string(road_segmentation_config_json)
        );
        
        // Allocate memory for the result string (caller must free this)
        char* result_cstr = (char*)malloc(result.length() + 1);
        strcpy(result_cstr, result.c_str());
        return result_cstr;
    }
    
    EMSCRIPTEN_KEEPALIVE
    const char* processConvexPathTool(
        const char* writer_config_json,
        const char* road_config_json,
        const char* point_config_json,
        const char* building_config_json
    ) {
        std::string result = wasm_interface::processConvexPathTool(
            std::string(writer_config_json),
            std::string(road_config_json),
            std::string(point_config_json),
            std::string(building_config_json)
        );
        
        // Allocate memory for the result string (caller must free this)
        char* result_cstr = (char*)malloc(result.length() + 1);
        strcpy(result_cstr, result.c_str());
        return result_cstr;
    }
    
    EMSCRIPTEN_KEEPALIVE
    const char* processNeighboringPointsTool(
        const char* writer_config_json,
        const char* road_config_json,
        const char* point_config_json,
        const char* neighboring_points_config_json
    ) {
        std::string result = wasm_interface::processNeighboringPointsTool(
            std::string(writer_config_json),
            std::string(road_config_json),
            std::string(point_config_json),
            std::string(neighboring_points_config_json)
        );
        
        // Allocate memory for the result string (caller must free this)
        char* result_cstr = (char*)malloc(result.length() + 1);
        strcpy(result_cstr, result.c_str());
        return result_cstr;
    }
}