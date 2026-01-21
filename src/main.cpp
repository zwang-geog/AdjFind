#include <iostream>
#include <string>
#include <unordered_map>
#include <iomanip>
#include <vector>
#include <tuple>
#include <algorithm>
#include <limits>
#include "io/road_reader.hpp"
#include "io/point_reader.hpp"
#include "io/writer_road_segmentation.hpp"
#include "io/writer_convex_path.hpp"
#include "io/writer_neighboring_points.hpp"
#include "io/gdal_utils.hpp"
#include "graph/adj_graph.hpp"
#include "graph/road_segmentation.hpp"
#include "graph/convex_path.hpp"
#include "graph/neighboring_points.hpp"
#include <optional>
#include <stdexcept>
#include <boost/filesystem.hpp>
#include <sstream>

using namespace adjfind;

// Helper function to parse comma-separated string into vector of doubles
std::vector<double> parseDistanceVector(const std::string& distance_str) {
    std::vector<double> distances;
    
    // Handle empty string
    if (distance_str.empty()) {
        return distances;
    }
    
    std::stringstream ss(distance_str);
    std::string token;
    
    while (std::getline(ss, token, ',')) {
        // Trim whitespace
        token.erase(0, token.find_first_not_of(" \t"));
        token.erase(token.find_last_not_of(" \t") + 1);
        
        if (!token.empty()) {
            double distance = std::stod(token);
            if (distance <= 0) {
                throw std::invalid_argument("Distance values must be positive");
            }
            distances.push_back(distance);
        }
    }
    
    return distances;
}

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
              << "  --road-layer-index <index>   Layer index to read from road file; typically this parameter should not be set unless the input dataset is a geopackage with multiple layers (default: 0)\n"
              << "  --point-id-field <name>      Field name for point ID (defaults to OGR FID)\n"
              << "  --point-layer-index <index>  Layer index to read from point file; typically this parameter should not be set unless the input dataset is a geopackage with multiple layers (default: 0)\n"
              << "  --reproject-to-epsg4326      A flag variable that will reproject output to EPSG:4326 (WGS84)\n"
              << "\nFor road-segmentation mode, additional optional arguments:\n"
              << "  --distance-breakpoints <values>   Comma-separated positive numbers for distance breakpoints that are used to split road linestrings to discrete distance categories (if not specified, the output delineates service area of each point feature on the network)\n"
              << "\nFor structure-access mode, additional required arguments:\n"
              << "  --building-file-path <path>  Path to the building dataset file\n"
              << "\nFor structure-access mode, additional optional arguments:\n"
              << "  --building-id-field <name>   Field name for building ID (defaults to OGR FID)\n"
              << "  --building-layer-index <index> Layer index to read from building file (default: 0)\n"
              << "  --is-obstacle-only-field <name> Field name for obstacle-only flag\n"
              << "  --snappable-ids-field <name> Field name for comma-separated road IDs or point IDs (when road dataset not provided)\n"
              << "  --candidate-access-points-search-distance <value> Buffer distance for candidate access points search when road dataset not provided (default: 1500)\n"
              << "  --min-polygon-boundary-segment-length-for-nearest-road-edge-detection <length> Minimum polygon boundary segment length for nearest road edge detection (default: 80.0)\n"
              << "  --include-network-distance    A flag argument that will include network distance in addition to access distance (structure-road) in the objective function of path finding algorithm (default: not included)\n"
              << "  --graph-vertex-snapping-tolerance <value> Distance tolerance for snapping graph vertices (default: 1e-6)\n"
              << "  --output-road-access-point    A flag argument to output road access point file (last point of each path linestring) when road dataset is provided (default: false)\n"
              << "\nFor neighboring-points mode, additional optional arguments:\n"
              << "  --intersection-vertex-distance-threshold <value> Any point snapped to within this threshold from a road intersection will be subject to neighbor search from all outgoing directions from the intersection (default: 60.0)\n"
              << "  --cutoff <value>               If the path distance exceeds this cutoff and still no neighbor found along a given travel direction, the search along this direction will stop\n"
              << "\nExamples:\n"
              << "  " << programName << " --road-file-path roads.gpkg --point-file-path points.gpkg --mode road-segmentation --output-file results.geojson --distance-breakpoints 100,200,300 --reproject-to-epsg4326\n"
              << "  " << programName << " --road-file-path roads.gpkg --point-file-path points.gpkg --building-file-path buildings.gpkg --mode structure-access --output-file structure_results.geojson\n"
              << "  " << programName << " --road-file-path roads.gpkg --point-file-path points.gpkg --mode neighboring-points --output-file neighboring_results.geojson --intersection-vertex-distance-threshold 50.0 --cutoff 200.0\n"
              << "\nUse --help for detailed parameter explanations and examples.\n"
              << "Use --version to display version information.\n";
}

void printDetailedHelp(const char* programName) {
    std::cout << "AdjFind - Adjacency/Proximity Path Finding Tools\n"
              << "==============================================\n\n"
              << "AdjFind is a C++ application that provides specialized path finding algorithms related to adjacency/proximity.\n\n"
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
              << "     --road-layer-index <index>  Layer index to read from road file (default: 0)\n"
              << "     --point-id-field <name>     Field name for point ID (defaults to OGR FID)\n"
              << "     --point-layer-index <index> Layer index to read from point file (default: 0)\n"
              << "     --reproject-to-epsg4326     Reproject output to EPSG:4326 (WGS84)\n"
              << "     --distance-breakpoints <values> Comma-separated positive numbers for distance breakpoints\n\n"
              << "   Example:\n"
              << "     " << programName << " --road-file-path roads.gpkg --point-file-path points.gpkg --mode road-segmentation --output-file results.geojson --distance-breakpoints 100,200,300 --reproject-to-epsg4326\n\n"
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
              << "     --road-layer-index <index>  Layer index to read from road file (default: 0)\n"
              << "     --point-id-field <name>     Field name for point ID (defaults to OGR FID)\n"
              << "     --point-layer-index <index> Layer index to read from point file (default: 0)\n"
              << "     --reproject-to-epsg4326     Reproject output to EPSG:4326 (WGS84)\n"
              << "     --intersection-vertex-distance-threshold <value> Distance threshold for intersection vertices (default: 60.0)\n"
              << "     --cutoff <value>            Distance cutoff for neighbor search\n\n"
              << "   Example:\n"
              << "     " << programName << " --road-file-path roads.gpkg --point-file-path points.gpkg --mode neighboring-points --output-file neighboring_results.geojson\n\n"
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
              << "     --road-layer-index <index>  Layer index to read from road file (default: 0)\n"
              << "     --point-id-field <name>     Field name for point ID (defaults to OGR FID)\n"
              << "     --point-layer-index <index> Layer index to read from point file (default: 0)\n"
              << "     --building-id-field <name>  Field name for building ID (defaults to OGR FID)\n"
              << "     --building-layer-index <index> Layer index to read from building file (default: 0)\n"
              << "     --is-obstacle-only-field <name> Field name for obstacle-only flag\n"
              << "     --snappable-ids-field <name> Field name for comma-separated road IDs or point IDs (when road dataset not provided)\n"
              << "     --candidate-access-points-search-distance <value> Buffer distance for candidate access points search when road dataset not provided (default: 1500)\n"
              << "     --min-polygon-boundary-segment-length-for-nearest-road-edge-detection <length> Minimum polygon boundary segment length (default: 80.0)\n"
              << "     --include-network-distance    Include network distance in objective function\n"
              << "     --graph-vertex-snapping-tolerance <value> Distance tolerance for snapping graph vertices (default: 1e-6)\n"
              << "     --output-road-access-point    Output road access point file (last point of each path linestring) when road dataset is provided (default: false)\n"
              << "     --reproject-to-epsg4326     Reproject output to EPSG:4326 (WGS84)\n\n"
              << "   Example:\n"
              << "     " << programName << " --road-file-path roads.gpkg --point-file-path points.gpkg --building-file-path buildings.gpkg --mode structure-access --output-file structure_results.geojson\n\n"
              << "INPUT DATASET RECOMMENDATIONS:\n\n"
              << "Coordinate System:\n"
              << "  - Use projected coordinate systems in meters or feet\n"
              << "  - All input datasets should have the same coordinate system\n"
              << "  - WGS84 (EPSG:4326) will be internally reprojected to UTM\n\n"
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

// Helper function to parse boolean flag values
// Returns true if flag exists and value is "true" (case-insensitive), false otherwise
bool parseBooleanFlag(const std::unordered_map<std::string, std::string>& args, const std::string& flag_name, bool default_value = false) {
    if (args.count(flag_name) == 0) {
        return default_value;
    }
    
    std::string value = args.at(flag_name);
    // Convert to lowercase for case-insensitive comparison
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);
    
    // Check for true values
    if (value == "true" || value == "1" || value == "yes" || value == "on") {
        return true;
    }
    
    // Check for false values
    if (value == "false" || value == "0" || value == "no" || value == "off" || value.empty()) {
        return false;
    }
    
    // If value is provided but not recognized, default to true (for backward compatibility with flag-only usage)
    return true;
}

void processRoadSegmentationMode(const std::unordered_map<std::string, std::string>& args) {
    // Parse required arguments
    std::string road_file_path = args.at("road-file-path");
    std::string point_file_path = args.at("point-file-path");
    
    // Parse optional arguments
    std::string road_id_field = args.count("road-id-field") ? args.at("road-id-field") : "";
    std::string road_from_z_field = args.count("road-from-z-field") ? args.at("road-from-z-field") : "";
    std::string road_to_z_field = args.count("road-to-z-field") ? args.at("road-to-z-field") : "";
    std::string road_length_field = args.count("road-length-field") ? args.at("road-length-field") : "";
    std::string point_id_field = args.count("point-id-field") ? args.at("point-id-field") : "";
    
    // Parse layer indices
    int road_layer_index = 0;
    int point_layer_index = 0;
    if (args.count("road-layer-index") > 0) {
        try {
            road_layer_index = std::stoi(args.at("road-layer-index"));
            if (road_layer_index < 0) {
                throw std::invalid_argument("Road layer index must be non-negative");
            }
        } catch (const std::exception& e) {
            std::cerr << "Error: Invalid road-layer-index: " << e.what() << std::endl;
            return;
        }
    }
    if (args.count("point-layer-index") > 0) {
        try {
            point_layer_index = std::stoi(args.at("point-layer-index"));
            if (point_layer_index < 0) {
                throw std::invalid_argument("Point layer index must be non-negative");
            }
        } catch (const std::exception& e) {
            std::cerr << "Error: Invalid point-layer-index: " << e.what() << std::endl;
            return;
        }
    }
    
    // Parse output arguments
    std::string output_file = args.count("output-file") ? args.at("output-file") : "output.geojson";
    bool reproject_to_epsg4326 = parseBooleanFlag(args, "reproject-to-epsg4326", false);
    
    // Parse distance breakpoints (optional)
    std::vector<double> distance_breakpoints;
    if (args.count("distance-breakpoints") > 0) {
        std::string distance_breakpoints_str = args.at("distance-breakpoints");
        if (!distance_breakpoints_str.empty()) {
            distance_breakpoints = parseDistanceVector(distance_breakpoints_str);
        }
    }
    
    // Create configurations
    io::RoadReaderConfig road_config;
    road_config.file_path = road_file_path;
    road_config.id_field = road_id_field;
    road_config.from_z_field = road_from_z_field;
    road_config.to_z_field = road_to_z_field;
    road_config.length_field = road_length_field;
    road_config.default_from_z = 0.0;  // Always default to 0.0
    road_config.default_to_z = 0.0;    // Always default to 0.0
    road_config.layer_index = road_layer_index;
    
    io::PointReaderConfig point_config;
    point_config.file_path = point_file_path;
    point_config.id_field = point_id_field;
    point_config.layer_index = point_layer_index;
    
    // Create road segmentation and process
    graph::RoadSegmentation road_segmentation;
    std::vector<graph::RoadSplitByDistanceBracketsOutput> output = road_segmentation.processRoadSegmentationMode(road_config, point_config, distance_breakpoints);
    
    // Write output to file
    io::RoadSegmentationWriter writer;
    io::RoadSegmentationWriterConfig writer_config;
    writer_config.output_file_path = output_file;
    writer_config.crs_wkt = road_segmentation.getCoordinateSystemWKT();
    writer_config.reproject_to_epsg4326 = reproject_to_epsg4326;
    
    if (!writer.writeRoadSegmentationResults(writer_config, output)) {
        std::cerr << "Error writing output file: " << writer.getLastError() << std::endl;
        return;
    }
    
    std::cout << "\nOutput written successfully to: " << output_file << std::endl;
}

void processStructureAccessMode(const std::unordered_map<std::string, std::string>& args) {
    // Parse required arguments
    bool has_road_file = args.count("road-file-path") > 0;
    std::string road_file_path = has_road_file ? args.at("road-file-path") : "";
    std::string point_file_path = args.at("point-file-path");
    std::string building_file_path = args.at("building-file-path");
    
    // Parse optional arguments
    std::string road_id_field = args.count("road-id-field") ? args.at("road-id-field") : "";
    std::string road_from_z_field = args.count("road-from-z-field") ? args.at("road-from-z-field") : "";
    std::string road_to_z_field = args.count("road-to-z-field") ? args.at("road-to-z-field") : "";
    std::string road_length_field = args.count("road-length-field") ? args.at("road-length-field") : "";
    int road_layer_index = 0;
    if (args.count("road-layer-index") > 0) {
        try {
            road_layer_index = std::stoi(args.at("road-layer-index"));
        } catch (const std::exception& e) {
            std::cerr << "Error: Invalid road-layer-index: " << e.what() << std::endl;
            return;
        }
    }
    
    std::string point_id_field = args.count("point-id-field") ? args.at("point-id-field") : "";
    int point_layer_index = 0;
    if (args.count("point-layer-index") > 0) {
        try {
            point_layer_index = std::stoi(args.at("point-layer-index"));
        } catch (const std::exception& e) {
            std::cerr << "Error: Invalid point-layer-index: " << e.what() << std::endl;
            return;
        }
    }
    
    std::string building_id_field = args.count("building-id-field") ? args.at("building-id-field") : "";
    int building_layer_index = 0;
    if (args.count("building-layer-index") > 0) {
        try {
            building_layer_index = std::stoi(args.at("building-layer-index"));
        } catch (const std::exception& e) {
            std::cerr << "Error: Invalid building-layer-index: " << e.what() << std::endl;
            return;
        }
    }
    
    std::string is_obstacle_only_field = args.count("is-obstacle-only-field") ? args.at("is-obstacle-only-field") : "";
    // snappable_ids_field: prefer --snappable-ids-field; if not specified and road dataset is provided, use --road-ids-snappable-field for backward compatibility
    std::string snappable_ids_field = args.count("snappable-ids-field") ? args.at("snappable-ids-field") : (args.count("road-ids-snappable-field") ? args.at("road-ids-snappable-field") : "");
    
    // Parse candidate-access-points-search-distance (used only when road dataset is not provided; default 1500)
    double candidate_access_points_search_distance = 1500.0;
    if (args.count("candidate-access-points-search-distance") > 0) {
        try {
            candidate_access_points_search_distance = std::stod(args.at("candidate-access-points-search-distance"));
            if (candidate_access_points_search_distance <= 0) {
                std::cerr << "Error: candidate-access-points-search-distance must be positive" << std::endl;
                return;
            }
        } catch (const std::exception& e) {
            std::cerr << "Error: Invalid candidate-access-points-search-distance: " << e.what() << std::endl;
            return;
        }
    }
    
    // Parse min polygon boundary segment length for nearest road edge detection
    double min_polygon_boundary_segment_length = 80.0; // Default value
    if (args.count("min-polygon-boundary-segment-length-for-nearest-road-edge-detection") > 0) {
        try {
            min_polygon_boundary_segment_length = std::stod(args.at("min-polygon-boundary-segment-length-for-nearest-road-edge-detection"));
            if (min_polygon_boundary_segment_length <= 0) {
                std::cerr << "Error: min-polygon-boundary-segment-length-for-nearest-road-edge-detection must be positive" << std::endl;
                return;
            }
        } catch (const std::exception& e) {
            std::cerr << "Error: Invalid min-polygon-boundary-segment-length-for-nearest-road-edge-detection: " << e.what() << std::endl;
            return;
        }
    }
    
    // Parse output arguments
    std::string output_file = args.count("output-file") ? args.at("output-file") : "output.geojson";
    bool reproject_to_epsg4326 = parseBooleanFlag(args, "reproject-to-epsg4326", false);
    
    // Parse include-network-distance flag
    bool include_network_distance = parseBooleanFlag(args, "include-network-distance", false);
    
    // Parse output-road-access-point flag
    bool output_road_access_point = parseBooleanFlag(args, "output-road-access-point", false);
    
    // Parse graph-vertex-snapping-tolerance
    double graph_vertex_snapping_tolerance = 1e-6; // Default value
    if (args.count("graph-vertex-snapping-tolerance") > 0) {
        try {
            graph_vertex_snapping_tolerance = std::stod(args.at("graph-vertex-snapping-tolerance"));
            if (graph_vertex_snapping_tolerance <= 0) {
                std::cerr << "Error: graph-vertex-snapping-tolerance must be positive" << std::endl;
                return;
            }
        } catch (const std::exception& e) {
            std::cerr << "Error: Invalid graph-vertex-snapping-tolerance: " << e.what() << std::endl;
            return;
        }
    }
    
    io::PointReaderConfig point_config;
    point_config.file_path = point_file_path;
    point_config.id_field = point_id_field;
    point_config.layer_index = point_layer_index;
    
    io::PolygonReaderConfig building_config;
    building_config.file_path = building_file_path;
    building_config.id_field = building_id_field;
    building_config.is_obstacle_only_field = is_obstacle_only_field;
    building_config.snappable_ids_field = snappable_ids_field;
    building_config.candidate_access_points_search_distance = candidate_access_points_search_distance;
    building_config.layer_index = building_layer_index;
    building_config.min_polygon_boundary_segment_length_for_nearest_road_edge_detection = min_polygon_boundary_segment_length;
    
    // Create convex path processor and process
    graph::ConvexPath convex_path;
    
    // Set the include-network-distance flag
    convex_path.setIncludeNetworkDistance(include_network_distance);
    std::cout << "Include network distance flag set to: " << (include_network_distance ? "true" : "false") << std::endl;
    
    // Set the graph vertex snapping tolerance
    convex_path.setGraphVertexSnappingTolerance(graph_vertex_snapping_tolerance);
    std::cout << "Graph vertex snapping tolerance set to: " << graph_vertex_snapping_tolerance << std::endl;
    
    bool success = false;
    if (has_road_file) {
        // Create road configuration
        io::RoadReaderConfig road_config;
        road_config.file_path = road_file_path;
        road_config.id_field = road_id_field;
        road_config.from_z_field = road_from_z_field;
        road_config.to_z_field = road_to_z_field;
        road_config.length_field = road_length_field;
        road_config.default_from_z = 0.0;  // Always default to 0.0
        road_config.default_to_z = 0.0;    // Always default to 0.0
        road_config.layer_index = road_layer_index;
        
        success = convex_path.processConvexPathMode(road_config, point_config, building_config);
    } else {
        // No road file provided, use processConvexPathModeNoRoad
        success = convex_path.processConvexPathModeNoRoad(point_config, building_config);
    }
    
    if (!success) {
        std::cerr << "Error processing structure access mode" << std::endl;
        return;
    }
    
    // Write convex path results to output files
    io::ConvexPathWriter writer;
    io::ConvexPathWriterConfig writer_config;
    writer_config.output_file_path = output_file;
    writer_config.crs_wkt = convex_path.getCoordinateSystemWKT();
    writer_config.reproject_to_epsg4326 = reproject_to_epsg4326;
    writer_config.use_road_data = has_road_file;
    writer_config.output_road_access_point = output_road_access_point;
    
    const auto& polygon_results = convex_path.getPolygonResults();
    
    if (!writer.writeConvexPathResults(writer_config, polygon_results)) {
        std::cerr << "Error writing convex path output files: " << writer.getLastError() << std::endl;
        return;
    }
    
    std::cout << "\nStructure access processing completed successfully" << std::endl;
    std::cout << "Output written to: " << output_file << std::endl;
}

void processNeighboringPointsMode(const std::unordered_map<std::string, std::string>& args) {
    // Parse required arguments
    std::string road_file_path = args.at("road-file-path");
    std::string point_file_path = args.at("point-file-path");
    
    // Parse optional arguments
    std::string road_id_field = args.count("road-id-field") ? args.at("road-id-field") : "";
    std::string road_from_z_field = args.count("road-from-z-field") ? args.at("road-from-z-field") : "";
    std::string road_to_z_field = args.count("road-to-z-field") ? args.at("road-to-z-field") : "";
    std::string road_length_field = args.count("road-length-field") ? args.at("road-length-field") : "";
    std::string point_id_field = args.count("point-id-field") ? args.at("point-id-field") : "";
    
    // Parse layer indices
    int road_layer_index = 0;
    int point_layer_index = 0;
    if (args.count("road-layer-index") > 0) {
        try {
            road_layer_index = std::stoi(args.at("road-layer-index"));
            if (road_layer_index < 0) {
                throw std::invalid_argument("Road layer index must be non-negative");
            }
        } catch (const std::exception& e) {
            std::cerr << "Error: Invalid road-layer-index: " << e.what() << std::endl;
            return;
        }
    }
    if (args.count("point-layer-index") > 0) {
        try {
            point_layer_index = std::stoi(args.at("point-layer-index"));
            if (point_layer_index < 0) {
                throw std::invalid_argument("Point layer index must be non-negative");
            }
        } catch (const std::exception& e) {
            std::cerr << "Error: Invalid point-layer-index: " << e.what() << std::endl;
            return;
        }
    }
    
    // Parse output arguments
    std::string output_file = args.count("output-file") ? args.at("output-file") : "output.geojson";
    bool reproject_to_epsg4326 = parseBooleanFlag(args, "reproject-to-epsg4326", false);
    
    // Parse neighboring points specific arguments
    double intersection_vertex_distance_threshold = 60.0; // Default value
    if (args.count("intersection-vertex-distance-threshold") > 0) {
        try {
            intersection_vertex_distance_threshold = std::stod(args.at("intersection-vertex-distance-threshold"));
            if (intersection_vertex_distance_threshold <= 0) {
                throw std::invalid_argument("Intersection vertex distance threshold must be positive");
            }
        } catch (const std::exception& e) {
            std::cerr << "Error: Invalid intersection-vertex-distance-threshold: " << e.what() << std::endl;
            return;
        }
    }
    
    std::optional<double> cutoff = std::nullopt;
    if (args.count("cutoff") > 0) {
        try {
            double cutoff_value = std::stod(args.at("cutoff"));
            if (cutoff_value <= 0) {
                throw std::invalid_argument("Cutoff must be positive");
            }
            cutoff = cutoff_value;
        } catch (const std::exception& e) {
            std::cerr << "Error: Invalid cutoff: " << e.what() << std::endl;
            return;
        }
    }
    
    // Create configurations
    io::RoadReaderConfig road_config;
    road_config.file_path = road_file_path;
    road_config.id_field = road_id_field;
    road_config.from_z_field = road_from_z_field;
    road_config.to_z_field = road_to_z_field;
    road_config.length_field = road_length_field;
    road_config.default_from_z = 0.0;  // Always default to 0.0
    road_config.default_to_z = 0.0;    // Always default to 0.0
    road_config.layer_index = road_layer_index;
    
    io::PointReaderConfig point_config;
    point_config.file_path = point_file_path;
    point_config.id_field = point_id_field;
    point_config.layer_index = point_layer_index;
    
    // Create neighboring points processor and process
    graph::NeighboringPoints neighboring_points;
    if (!neighboring_points.processNeighboringPoints(road_config, point_config, 
                                                    intersection_vertex_distance_threshold, cutoff)) {
        std::cerr << "Error processing neighboring points mode" << std::endl;
        return;
    }
    
    // Write neighboring points results to output files
    io::NeighboringPointsWriter writer;
    io::NeighboringPointsWriterConfig writer_config;
    writer_config.output_file_path = output_file;
    writer_config.crs_wkt = neighboring_points.getCoordinateSystemWKT();
    writer_config.reproject_to_epsg4326 = reproject_to_epsg4326;
    
    const auto& neighboring_points_results = neighboring_points.getNeighboringPointsResults();
    
    if (!writer.writeNeighboringPointsResults(writer_config, neighboring_points_results, neighboring_points)) {
        std::cerr << "Error writing neighboring points output files: " << writer.getLastError() << std::endl;
        return;
    }
    
    std::cout << "\nNeighboring points processing completed successfully" << std::endl;
    std::cout << "Output written to: " << output_file << std::endl;
    std::cout << "Snapped points written to: " << writer.generateSnappedPointsFilePath(output_file) << std::endl;
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
            std::cout << "AdjFind v0.2.0\n";
            std::cout << "Adjacency/Proximity Path Finding Tool\n";
            std::cout << "Copyright (c) 2026 Zifan Wang\n";
            std::cout << "MIT License\n";
            return 0;
        }
        
        // Check for required arguments
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
        
        // Check for road-file-path (required for all modes except structure-access)
        if (mode != "structure-access" && args.count("road-file-path") == 0) {
            std::cerr << "Error: --road-file-path is required for mode '" << mode << "'" << std::endl;
            printUsage(argv[0]);
            return 1;
        }
        
        if (mode == "structure-access" && args.count("building-file-path") == 0) {
            std::cerr << "Error: --building-file-path is required for structure-access mode" << std::endl;
            printUsage(argv[0]);
            return 1;
        }
        
        if (mode == "road-segmentation") {
            processRoadSegmentationMode(args);
        } else if (mode == "structure-access") {
            processStructureAccessMode(args);
        } else if (mode == "neighboring-points") {
            processNeighboringPointsMode(args);
        } else {
            std::cerr << "Error: Unknown mode '" << mode << "'" << std::endl;
            printUsage(argv[0]);
            return 1;
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
} 