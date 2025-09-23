#include <iostream>
#include <string>
#include <unordered_map>
#include <iomanip>
#include <vector>
#include <tuple>
#include <algorithm>
#include <limits>
#include <optional>
#include <stdexcept>
#include <filesystem>
#include <sstream>
#include <nlohmann/json.hpp>
#include "wasm/wasm_interface.hpp"

using namespace wasm_interface;


void printUsage(const char* programName) {
    std::cout << "Usage: " << programName << " --road-file-path <path> --point-file-path <path> --mode <mode> [options]\n"
              << "\nRequired arguments:\n"
              << "  --road-file-path <path>      Path to the road network file\n"
              << "  --point-file-path <path>     Path to the point file\n"
              << "  --mode <mode>                Operation mode: 'road-segmentation', 'structure-access', or 'neighboring-points'\n"
              << "  --output-file <path>         Output file path with extension (required for all modes)\n"
              << "\nOptional arguments:\n"
              << "  --road-id-field <name>       Field name for road ID (defaults to OGR FID)\n"
              << "  --road-from-z-field <name>   Field name for from z-level\n"
              << "  --road-to-z-field <name>     Field name for to z-level\n"
              << "  --road-length-field <name>   Field name for length (computed if not specified)\n"
              << "  --point-id-field <name>      Field name for point ID (defaults to OGR FID)\n"
              << "\nFor road-segmentation mode, additional optional arguments:\n"
              << "  --distance-breakpoints <values>   Comma-separated positive numbers for distance breakpoints that are used to split road linestrings to discrete distance categories (if not specified, the output delineates service area of each point feature on the network)\n"
              << "\nFor structure-access mode, additional required arguments:\n"
              << "  --building-file-path <path>  Path to the building dataset file\n"
              << "\nFor structure-access mode, additional optional arguments:\n"
              << "  --building-id-field <name>   Field name for building ID (defaults to OGR FID)\n"
              << "  --is-obstacle-only-field <name> Field name for obstacle-only flag\n"
              << "  --road-ids-snappable-field <name> Field name for comma-separated road IDs\n"
              << "  --min-polygon-boundary-segment-length-for-nearest-road-edge-detection <length> Minimum polygon boundary segment length for nearest road edge detection (default: 80.0)\n"
              << "\nFor neighboring-points mode, additional optional arguments:\n"
              << "  --intersection-vertex-distance-threshold <value> Any point snapped to within this threshold from a road intersection will be subject to neighbor search from all outgoing directions from the intersection (default: 60.0)\n"
              << "  --cutoff <value>               If the path distance exceeds this cutoff and still no neighbor found along a given travel direction, the search along this direction will stop\n"
              << "\nExamples:\n"
              << "  " << programName << " --road-file-path roads.geojson --point-file-path points.geojson --mode road-segmentation --output-file results.geojson --distance-breakpoints 100,200,300\n"
              << "  " << programName << " --road-file-path roads.geojson --point-file-path points.geojson --building-file-path buildings.geojson --mode structure-access --output-file structure_results.geojson\n"
              << "  " << programName << " --road-file-path roads.geojson --point-file-path points.geojson --mode neighboring-points --output-file neighboring_results.geojson --intersection-vertex-distance-threshold 50.0 --cutoff 200.0\n"
              << "\nUse --help for detailed parameter explanations and examples.\n"
              << "Use --version to display version information.\n";
}

void printDetailedHelp(const char* programName) {
    std::cout << "AdjFind - Transportation Network Analysis Tool\n"
              << "==============================================\n\n"
              << "AdjFind is a C++ application that provides specialized transportation network algorithms.\n\n"
              << "MODES:\n\n"
              << "1. ROAD SEGMENTATION MODE (--mode road-segmentation)\n"
              << "   Determines which service points (e.g., hydrants) are closest to each road segment using network distance.\n"
              << "   Optionally splits roads into discrete distance categories for service area analysis.\n\n"
              << "   Required Arguments:\n"
              << "     --road-file-path <path>     Path to the road network file\n"
              << "     --point-file-path <path>    Path to the point file\n"
              << "     --output-file <path>        Output file path with extension\n\n"
              << "   Optional Arguments:\n"
              << "     --road-id-field <name>      Field name for road ID (defaults to OGR FID)\n"
              << "     --road-from-z-field <name>  Field name for from z-level\n"
              << "     --road-to-z-field <name>    Field name for to z-level\n"
              << "     --road-length-field <name>  Field name for length (computed if not specified)\n"
              << "     --point-id-field <name>     Field name for point ID (defaults to OGR FID)\n"
              << "     --distance-breakpoints <values> Comma-separated positive numbers for distance breakpoints\n\n"
              << "   Example:\n"
              << "     " << programName << " --road-file-path roads.geojson --point-file-path points.geojson --mode road-segmentation --output-file results.geojson --distance-breakpoints 100,200,300\n\n"
              << "2. NEIGHBORING POINTS MODE (--mode neighboring-points)\n"
              << "   Identifies the nearest neighboring service points along a road network in each direction.\n"
              << "   Calculates spacing distances between facilities like hydrants for regulatory compliance assessment.\n\n"
              << "   Required Arguments:\n"
              << "     --road-file-path <path>     Path to the road network file\n"
              << "     --point-file-path <path>    Path to the point file\n"
              << "     --output-file <path>        Output file path with extension\n\n"
              << "   Optional Arguments:\n"
              << "     --road-id-field <name>      Field name for road ID (defaults to OGR FID)\n"
              << "     --road-from-z-field <name>  Field name for from z-level\n"
              << "     --road-to-z-field <name>    Field name for to z-level\n"
              << "     --road-length-field <name>  Field name for length (computed if not specified)\n"
              << "     --point-id-field <name>     Field name for point ID (defaults to OGR FID)\n"
              << "     --intersection-vertex-distance-threshold <value> Distance threshold for intersection vertices (default: 60.0)\n"
              << "     --cutoff <value>            Distance cutoff for neighbor search (optional)\n\n"
              << "   Example:\n"
              << "     " << programName << " --road-file-path roads.geojson --point-file-path points.geojson --mode neighboring-points --output-file neighboring_results.geojson\n\n"
              << "3. STRUCTURE ACCESS MODE (--mode structure-access)\n"
              << "   Computes shortest unobstructed paths from building corners to road networks.\n"
              << "   Finds the least accessible points on buildings for emergency response planning.\n\n"
              << "   Required Arguments:\n"
              << "     --road-file-path <path>     Path to the road network file\n"
              << "     --point-file-path <path>    Path to the point file\n"
              << "     --building-file-path <path> Path to the building dataset file\n"
              << "     --output-file <path>        Output file path with extension\n\n"
              << "   Optional Arguments:\n"
              << "     --road-id-field <name>      Field name for road ID (defaults to OGR FID)\n"
              << "     --road-from-z-field <name>  Field name for from z-level\n"
              << "     --road-to-z-field <name>    Field name for to z-level\n"
              << "     --road-length-field <name>  Field name for length (computed if not specified)\n"
              << "     --point-id-field <name>     Field name for point ID (defaults to OGR FID)\n"
              << "     --building-id-field <name>  Field name for building ID (defaults to OGR FID)\n"
              << "     --is-obstacle-only-field <name> Field name for obstacle-only flag\n"
              << "     --road-ids-snappable-field <name> Field name for comma-separated road IDs\n"
              << "     --min-polygon-boundary-segment-length-for-nearest-road-edge-detection <length> Minimum polygon boundary segment length (default: 80.0)\n\n"
              << "   Example:\n"
              << "     " << programName << " --road-file-path roads.geojson --point-file-path points.geojson --building-file-path buildings.geojson --mode structure-access --output-file structure_results.geojson\n\n"
              << "INPUT DATASET RECOMMENDATIONS:\n\n"
              << "Coordinate System:\n"
              << "  - Use projected coordinate systems in meters or feet\n"
              << "  - All input datasets should have the same coordinate system\n"
              << "  - GeoJSON format is required for input files\n\n"
              << "Road Network:\n"
              << "  - Roads must be split/planarized at intersections\n"
              << "  - Graph vertices created at linestring endpoints\n\n"
              << "Point Dataset:\n"
              << "  - Points don't need to be on road linestrings\n"
              << "  - Euclidean-distance based snapping will be performed\n\n"
              << "OTHER OPTIONS:\n"
              << "  --help, -h     Show this detailed help message\n"
              << "  --version, -v  Show version information\n";
}

std::unordered_map<std::string, std::string> parseArgs(int argc, char* argv[]) {
    std::unordered_map<std::string, std::string> args;
    
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        
        if (arg.substr(0, 2) == "--") {
            std::string key = arg.substr(2);
            
            // Check if this is a flag argument (no value) or a key-value argument
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                // This is a key-value argument
                std::string value = argv[i + 1];
                args[key] = value;
                i++; // Skip the value in next iteration
            } else {
                // This is a flag argument - set it to "true"
                args[key] = "true";
            }
        }
    }
    
    return args;
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        printUsage(argv[0]);
        return 1;
    }
    
    try {
        auto args = parseArgs(argc, argv);
        
        // Check for help flag first
        if (args.count("help") > 0 || args.count("h") > 0) {
            printDetailedHelp(argv[0]);
            return 0;
        }
        
        // Check for version flag
        if (args.count("version") > 0 || args.count("v") > 0) {
            std::cout << "AdjFind v1.0.0\n";
            std::cout << "Transportation Network Analysis Tool\n";
            std::cout << "Copyright (c) 2025 Zifan Wang\n";
            std::cout << "MIT License\n";
            return 0;
        }
        
        // Check for required arguments
        if (args.count("road-file-path") == 0) {
            std::cerr << "Error: --road-file-path is required" << std::endl;
            printUsage(argv[0]);
            return 1;
        }
        
        if (args.count("point-file-path") == 0) {
            std::cerr << "Error: --point-file-path is required" << std::endl;
            printUsage(argv[0]);
            return 1;
        }
        
        if (args.count("output-file") == 0) {
            std::cerr << "Error: --output-file is required" << std::endl;
            printUsage(argv[0]);
            return 1;
        }
        
        if (args.count("mode") == 0) {
            std::cerr << "Error: --mode is required" << std::endl;
            printUsage(argv[0]);
            return 1;
        }
        
        std::string mode = args.at("mode");
        
        if (mode == "structure-access" && args.count("building-file-path") == 0) {
            std::cerr << "Error: --building-file-path is required for structure-access mode" << std::endl;
            printUsage(argv[0]);
            return 1;
        }
        
        // Convert arguments to JSON configurations
        nlohmann::json writer_config = nlohmann::json::object();
        
        // Set writer config based on mode (matching virtualFileSystem.js pattern)
        if (mode == "road-segmentation") {
            writer_config["output_file_path"] = args.at("output-file");
        } else if (mode == "neighboring-points") {
            std::string base_name = args.at("output-file");
            // Remove .geojson extension if present
            if (base_name.length() > 8 && base_name.substr(base_name.length() - 8) == ".geojson") {
                base_name = base_name.substr(0, base_name.length() - 8);
            }
            writer_config["linestring_output_file_path"] = base_name + ".geojson";
            writer_config["point_output_file_path"] = base_name + "_snapped_points.geojson";
        } else if (mode == "structure-access") {
            std::string base_name = args.at("output-file");
            // Remove .geojson extension if present
            if (base_name.length() > 8 && base_name.substr(base_name.length() - 8) == ".geojson") {
                base_name = base_name.substr(0, base_name.length() - 8);
            }
            writer_config["linestring_output_file_path"] = base_name + ".geojson";
            writer_config["point_output_file_path"] = base_name + "_least_accessible_point.geojson";
        }
        
        nlohmann::json road_config = nlohmann::json::object();
        road_config["file_path"] = args.at("road-file-path");
        if (args.count("road-id-field")) road_config["id_field"] = args.at("road-id-field");
        if (args.count("road-from-z-field")) road_config["from_z_field"] = args.at("road-from-z-field");
        if (args.count("road-to-z-field")) road_config["to_z_field"] = args.at("road-to-z-field");
        if (args.count("road-length-field")) road_config["length_field"] = args.at("road-length-field");
        
        nlohmann::json point_config = nlohmann::json::object();
        point_config["file_path"] = args.at("point-file-path");
        if (args.count("point-id-field")) point_config["id_field"] = args.at("point-id-field");
        
        std::string result;
        
        if (mode == "road-segmentation") {
            nlohmann::json road_segmentation_config = nlohmann::json::object();
            if (args.count("distance-breakpoints")) {
                road_segmentation_config["distance_breakpoints_str"] = args.at("distance-breakpoints");
            }
            
            result = processRoadSegmentationTool(
                writer_config.dump(),
                road_config.dump(),
                point_config.dump(),
                road_segmentation_config.dump()
            );
        } else if (mode == "structure-access") {
            nlohmann::json building_config = nlohmann::json::object();
            building_config["file_path"] = args.at("building-file-path");
            if (args.count("building-id-field")) building_config["id_field"] = args.at("building-id-field");
            if (args.count("is-obstacle-only-field")) building_config["is_obstacle_only_field"] = args.at("is-obstacle-only-field");
            if (args.count("road-ids-snappable-field")) building_config["road_ids_snappable_field"] = args.at("road-ids-snappable-field");
            if (args.count("min-polygon-boundary-segment-length-for-nearest-road-edge-detection")) {
                building_config["min_polygon_boundary_segment_length_for_nearest_road_edge_detection"] = std::stod(args.at("min-polygon-boundary-segment-length-for-nearest-road-edge-detection"));
            }
            
            result = processConvexPathTool(
                writer_config.dump(),
                road_config.dump(),
                point_config.dump(),
                building_config.dump()
            );
        } else if (mode == "neighboring-points") {
            nlohmann::json neighboring_points_config = nlohmann::json::object();
            if (args.count("intersection-vertex-distance-threshold")) {
                neighboring_points_config["intersection_vertex_distance_threshold"] = std::stod(args.at("intersection-vertex-distance-threshold"));
            }
            if (args.count("cutoff")) {
                neighboring_points_config["cutoff"] = std::stod(args.at("cutoff"));
            }
            
            result = processNeighboringPointsTool(
                writer_config.dump(),
                road_config.dump(),
                point_config.dump(),
                neighboring_points_config.dump()
            );
        } else {
            std::cerr << "Error: Unknown mode '" << mode << "'" << std::endl;
            printUsage(argv[0]);
            return 1;
        }
        
        std::cout << result << std::endl;
        
        // Check if result indicates an error
        if (result.substr(0, 5) == "Error") {
            return 1;
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
} 