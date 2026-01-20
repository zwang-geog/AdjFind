#include "graph/convex_path.hpp"
#include <iostream>
#include <limits>
#include <iomanip>
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <tuple>
#include <chrono>

namespace adjfind {
namespace graph {

namespace bg = boost::geometry;

ConvexPath::ConvexPath()
    : building_dataset_initialized_(false), useRoadData_(true), include_network_distance_(false), graph_vertex_snapping_tolerance_(1e-6), next_convex_path_vertex_id_(0), polygon_split_next_vertex_id_(0) {
}

bool ConvexPath::processConvexPathMode(const io::RoadReaderConfig& road_config, 
                                      const io::PointReaderConfig& point_config,
                                      const io::PolygonReaderConfig& building_config) {
    useRoadData_ = true;
    auto total_start_time = std::chrono::high_resolution_clock::now();
    std::cout << "Starting ConvexPath processing..." << std::endl;
    
    // Step 1: Initialize and read building dataset
    auto step1_start = std::chrono::high_resolution_clock::now();
    if (!initializeBuildingDataset(building_config)) {
        std::cerr << "Error: Failed to initialize building dataset" << std::endl;
        return false;
    }
    auto step1_end = std::chrono::high_resolution_clock::now();
    auto step1_duration = std::chrono::duration_cast<std::chrono::milliseconds>(step1_end - step1_start);
    std::cout << "Step 1 reading building dataset completed in " << step1_duration.count() << " ms" << std::endl;
    
    // Step 2: Apply AdjGraph::readAndSnapPointToRoad method
    auto step2_start = std::chrono::high_resolution_clock::now();
    if (!readAndSnapPointToRoad(road_config, point_config)) {
        std::cerr << "Error: Failed to read and snap points to roads" << std::endl;
        return false;
    }
    auto step2_end = std::chrono::high_resolution_clock::now();
    auto step2_duration = std::chrono::duration_cast<std::chrono::milliseconds>(step2_end - step2_start);
    std::cout << "Step 2 reading and snapping points to roads completed in " << step2_duration.count() << " ms" << std::endl;

    if (!building_reader_) {
        std::cerr << "Error: Building reader not initialized" << std::endl;
        return false;
    }
    
    // Coordinate system validation
    int road_epsg = getCoordinateSystemEPSG();
    int building_epsg = building_reader_->getCoordinateSystemEPSG();

    if (road_epsg != building_epsg) {
        std::cerr << "Warning: Coordinate system mismatch detected!" << std::endl;
        std::cerr << "Road and point dataset: EPSG:" << road_epsg << std::endl;
        std::cerr << "Building dataset: EPSG:" << building_epsg << std::endl;
        
        // Log WKT information for detailed debugging
        std::cout << "Road and point dataset WKT: " << getCoordinateSystemWKT() << std::endl;
        std::cout << "Building dataset WKT: " << building_reader_->getCoordinateSystemWKT() << std::endl;
        return false;
    } 
    
    // Step 3: Apply RoadSegmentation methods
    auto step3_start = std::chrono::high_resolution_clock::now();
    std::cout << "Populating distance to point vertex..." << std::endl;
    populateDistanceToPointVertex();
    
    std::cout << "Splitting linestrings at equilibrium points..." << std::endl;
    splitLinestringAtEquilibrium();
    auto step3_end = std::chrono::high_resolution_clock::now();
    auto step3_duration = std::chrono::duration_cast<std::chrono::milliseconds>(step3_end - step3_start);
    std::cout << "Step 3 splitting road linestrings completed in " << step3_duration.count() << " ms" << std::endl;
    
    // Step 4: Build edge R-tree
    auto step4_start = std::chrono::high_resolution_clock::now();
    std::cout << "Building edge R-tree..." << std::endl;
    buildEdgeRTree();
    
    // Step 5: Populate snappable road IDs for buildings
    std::cout << "Populating snappable road IDs for buildings..." << std::endl;
    building_reader_->populateSnappableRoadIds(*this);
    
    // Step 6: Clear edge R-tree to free memory (no longer needed after populating snappable road IDs)
    std::cout << "Clearing edge R-tree to free memory..." << std::endl;
    clearEdgeRTree();
    auto step6_end = std::chrono::high_resolution_clock::now();
    auto step6_duration = std::chrono::duration_cast<std::chrono::milliseconds>(step6_end - step4_start);
    std::cout << "Step 4-6 determining road edges that each building can snap to completed in " << step6_duration.count() << " ms" << std::endl;
    
    // Step 7: Process all polygons and collect results
    auto step7_start = std::chrono::high_resolution_clock::now();
    std::cout << "Computing convex paths for all non-obstacle polygons..." << std::endl;
    
    size_t total_polygons = building_reader_->getFeatureCount();
    
    // Create a local vector to collect results
    std::vector<std::tuple<std::vector<ConvexPathResult>, size_t, Point>> local_polygon_results;
    local_polygon_results.reserve(total_polygons);
    
    for (size_t i = 0; i < total_polygons; ++i) {
        const auto& polygon_feature = building_reader_->getPolygonFeature(i);
        
        // Skip obstacle-only polygons
        if (polygon_feature.is_obstacle_only) {
            continue;
        }
        
        std::cout << "Processing polygon " << (i + 1) << "/" << total_polygons 
                    << " (ID: " << polygon_feature.feature_id << ")..." << std::endl;
        
        try {
            auto result = processSinglePolygon(polygon_feature);
            
            local_polygon_results.push_back(result);
        } catch (const std::exception& e) {
            std::cerr << "Error processing polygon " << polygon_feature.feature_id 
                        << ": " << e.what() << std::endl;
            // Continue processing other polygons
        }
    }
    
    // Update the member variable with all collected results
    polygon_results_ = std::move(local_polygon_results);
    
    auto step7_end = std::chrono::high_resolution_clock::now();
    auto step7_duration = std::chrono::duration_cast<std::chrono::milliseconds>(step7_end - step7_start);
    std::cout << "Step 7 computaing convex paths for all polygons completed in " << step7_duration.count() << " ms" << std::endl;
    
    // Total processing time
    auto total_end_time = std::chrono::high_resolution_clock::now();
    auto total_duration = std::chrono::duration_cast<std::chrono::seconds>(total_end_time - total_start_time);
    
    std::cout << "ConvexPath processing completed successfully!" << std::endl;
    std::cout << "Total processing time: " << total_duration.count() << " s" << std::endl;
    return true;
}

bool ConvexPath::processConvexPathModeNoRoad(const io::PointReaderConfig& point_config,
                                             const io::PolygonReaderConfig& building_config) {
    useRoadData_ = false;
    auto total_start_time = std::chrono::high_resolution_clock::now();
    std::cout << "Starting ConvexPath processing (no road)..." << std::endl;
    
    // Step 1: Initialize and read building dataset
    if (!initializeBuildingDataset(building_config)) {
        std::cerr << "Error: Failed to initialize building dataset" << std::endl;
        return false;
    }
    
    // Step 2: Read point dataset (r-tree built in readFeatures)
    // Use local variable like readAndSnapPointToRoad does - only needed for populateSnappablePointIds
    io::PointReader point_reader(point_config);
    if (!point_reader.read()) {
        std::cerr << "Error: Failed to read point dataset" << std::endl;
        return false;
    }
    
    // Coordinate system validation between building and point
    // Note: This validation uses the readers' coordinate systems directly, not AdjGraph's
    int building_epsg = building_reader_->getCoordinateSystemEPSG();
    int point_epsg = point_reader.getCoordinateSystemEPSG();
    if (building_epsg != point_epsg) {
        std::cerr << "Error: Coordinate system mismatch. Building: EPSG:" << building_epsg
                  << ", Point: EPSG:" << point_epsg << std::endl;
        return false;
    }
    
    // Set coordinate system in AdjGraph for output writing (getCoordinateSystemWKT() called by writer)
    // This is separate from validation above, which uses readers' coordinate systems directly
    setCoordinateSystem(building_reader_->getCoordinateSystemWKT(), building_reader_->getCoordinateSystemEPSG());
    
    // Step 3: Populate snappable point IDs for buildings (uses PointReader::findPointsIntersectBoundingBox)
    std::cout << "Populating snappable point IDs for buildings..." << std::endl;
    building_reader_->populateSnappablePointIds(point_reader);
    
    // Step 4: Move points vector from reader (r-tree and dataset no longer needed)
    // After moving points, point_reader will be destroyed when it goes out of scope,
    // which will close the dataset in its destructor. Store as member variable so
    // processSinglePolygon, findConvexPath, and sectionalSearch can access it.
    access_points_ = point_reader.movePoints();
    std::cout << "Moved " << access_points_.size() << " access points from reader" << std::endl;
    
    // point_reader goes out of scope here - destructor will close dataset_ and clean up
    
    // Step 5: Process all polygons and collect results
    std::cout << "Computing convex paths for all non-obstacle polygons..." << std::endl;
    
    size_t total_polygons = building_reader_->getFeatureCount();
    
    // Create a local vector to collect results
    std::vector<std::tuple<std::vector<ConvexPathResult>, size_t, Point>> local_polygon_results;
    local_polygon_results.reserve(total_polygons);
    
    for (size_t i = 0; i < total_polygons; ++i) {
        const auto& polygon_feature = building_reader_->getPolygonFeature(i);
        
        // Skip obstacle-only polygons
        if (polygon_feature.is_obstacle_only) {
            continue;
        }
        
        std::cout << "Processing polygon " << (i + 1) << "/" << total_polygons 
                    << " (ID: " << polygon_feature.feature_id << ")..." << std::endl;
        
        try {
            auto result = processSinglePolygon(polygon_feature);
            
            local_polygon_results.push_back(result);
        } catch (const std::exception& e) {
            std::cerr << "Error processing polygon " << polygon_feature.feature_id 
                        << ": " << e.what() << std::endl;
            // Continue processing other polygons
        }
    }
    
    // Update the member variable with all collected results
    polygon_results_ = std::move(local_polygon_results);
    
    // Total processing time
    auto total_end_time = std::chrono::high_resolution_clock::now();
    auto total_duration = std::chrono::duration_cast<std::chrono::seconds>(total_end_time - total_start_time);
    
    std::cout << "ConvexPath processing (no road) completed successfully!" << std::endl;
    std::cout << "Total processing time: " << total_duration.count() << " s" << std::endl;
    return true;
}

bool ConvexPath::initializeBuildingDataset(const io::PolygonReaderConfig& building_config) {
    try {
        building_reader_ = std::make_unique<io::PolygonReader>(building_config);
        
        if (!building_reader_->read()) {
            std::cerr << "Error: Failed to read building dataset" << std::endl;
            return false;
        }
        
        std::cout << "Successfully read " << building_reader_->getFeatureCount() 
                  << " building features" << std::endl;
        
        building_dataset_initialized_ = true;
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Error initializing building dataset: " << e.what() << std::endl;
        return false;
    }
}

void ConvexPath::clear() {
    // Clear all convex path graph data structures
    convex_path_vertices_.clear();
    convex_path_edges_.clear();
    added_convex_path_edges_.clear();
    convex_path_vertex_rtree_.clear();
    next_convex_path_vertex_id_ = 0;

    // Clear polygon split graph data structures
    clearPolygonSplitGraph();
    
    // Clear polygon processing results
    polygon_results_.clear();
    
    // Clear access points
    access_points_.clear();
}

void ConvexPath::clearPolygonSplitGraph() {
    // Clear all polygon split graph data structures
    polygon_split_vertices_.clear();
    polygon_split_edges_.clear();
    added_polygon_split_edges_.clear();
    polygon_split_vertex_rtree_.clear();
    polygon_split_next_vertex_id_ = 0;
}

std::optional<PointFeature> ConvexPath::getAccessPoint(size_t index) const {
    if (index >= access_points_.size()) {
        return std::nullopt;
    }
    return access_points_[index];
}

size_t ConvexPath::findOrCreateConvexPathGraphVertex(const Point& point) {
    // Query R-tree for nearby vertices using distance-based tolerance
    std::vector<std::pair<Point, size_t>> candidates;
    convex_path_vertex_rtree_.query(
        bgi::nearest(point, 1) && 
        bgi::satisfies([&](const std::pair<Point, size_t>& v) {
            return bg::distance(point, v.first) <= graph_vertex_snapping_tolerance_;
        }),
        std::back_inserter(candidates)
    );
    
    if (!candidates.empty()) {
        return candidates[0].second;  // Return existing vertex ID
    }
    
    // Create new vertex
    size_t vertex_id = next_convex_path_vertex_id_++;
    convex_path_vertices_.emplace_back(point, vertex_id);
    convex_path_vertex_rtree_.insert(std::make_pair(point, vertex_id));
    return vertex_id;
}

std::optional<size_t> ConvexPath::findConvexPathGraphVertexForPoint(const Point& point) const {
    // Query R-tree for nearby vertices using distance-based tolerance
    std::vector<std::pair<Point, size_t>> candidates;
    convex_path_vertex_rtree_.query(
        bgi::nearest(point, 1) && 
        bgi::satisfies([&](const std::pair<Point, size_t>& v) {
            return bg::distance(point, v.first) <= graph_vertex_snapping_tolerance_;
        }),
        std::back_inserter(candidates)
    );
    
    if (!candidates.empty()) {
        return candidates[0].second;
    }
    
    return std::nullopt;
}

void ConvexPath::addSegmentToConvexPathGraph(const Point& start_point, const Point& end_point) {
    // Skip zero-length segments
    if (bg::equals(start_point, end_point)) {
        return;
    }
    
    size_t from_vertex = findOrCreateConvexPathGraphVertex(start_point);
    size_t to_vertex = findOrCreateConvexPathGraphVertex(end_point);
    
    if (from_vertex != to_vertex) {  // Additional check to avoid self-loops
        // Check if this edge pair already exists
        ConvexPathEdgePair edge_pair(from_vertex, to_vertex);
        if (added_convex_path_edges_.find(edge_pair) != added_convex_path_edges_.end()) {
            // Edge already exists, skip adding
            return;
        }
        
        // Mark this edge pair as added
        added_convex_path_edges_.insert(edge_pair);
        
        double weight = bg::distance(start_point, end_point);
        
        // Add bidirectional edges
        convex_path_edges_.emplace_back(from_vertex, to_vertex, weight);
        convex_path_edges_.emplace_back(to_vertex, from_vertex, weight);
    }
}

void ConvexPath::addSegmentToConvexPathGraph(const Segment& segment) {
    // Delegate to the point-based overload
    addSegmentToConvexPathGraph(segment.first, segment.second);
}

void ConvexPath::addSegmentToConvexPathGraph(size_t from_vertex, size_t to_vertex, bool check_duplicate) {
    // Validate vertex IDs
    if (from_vertex >= convex_path_vertices_.size() || to_vertex >= convex_path_vertices_.size()) {
        std::cerr << "Invalid vertex IDs in addSegment. From vertex: " << from_vertex 
                  << ", To vertex: " << to_vertex 
                  << ", Total vertices: " << convex_path_vertices_.size() << std::endl;
        return;
    }
    
    // Skip self-loops
    if (from_vertex == to_vertex) {
        return;
    }
    
    // Create edge pair once
    ConvexPathEdgePair edge_pair(from_vertex, to_vertex);
    
    // Check if this edge pair already exists (only if check_duplicate is true)
    if (check_duplicate) {
        if (added_convex_path_edges_.find(edge_pair) != added_convex_path_edges_.end()) {
            // Edge already exists, skip adding
            return;
        }
    }
    
    // Mark this edge pair as added
    added_convex_path_edges_.insert(edge_pair);
    
    // Calculate weight using vertex positions
    const auto& from_location = convex_path_vertices_[from_vertex].location;
    const auto& to_location = convex_path_vertices_[to_vertex].location;
    double weight = bg::distance(from_location, to_location);
    
    // Add bidirectional edges
    convex_path_edges_.emplace_back(from_vertex, to_vertex, weight);
    convex_path_edges_.emplace_back(to_vertex, from_vertex, weight);
}

size_t ConvexPath::findOrCreatePolygonSplitGraphVertex(const Point& point, bool is_boundary_vertex) {
    // Query R-tree for nearby vertices using distance-based tolerance
    std::vector<std::pair<Point, size_t>> candidates;
    polygon_split_vertex_rtree_.query(
        bgi::nearest(point, 1) && 
        bgi::satisfies([&](const std::pair<Point, size_t>& v) {
            return bg::distance(point, v.first) <= 1e-7;
        }),
        std::back_inserter(candidates)
    );
    
    if (!candidates.empty()) {
        return candidates[0].second;  // Return existing vertex ID
    }
    
    // Create new vertex
    size_t vertex_id = polygon_split_next_vertex_id_++;
    polygon_split_vertices_.emplace_back(point, vertex_id, is_boundary_vertex);
    polygon_split_vertex_rtree_.insert(std::make_pair(point, vertex_id));
    return vertex_id;
}

void ConvexPath::addSegmentToSplitPolygonGraph(const Point& start_point, const Point& end_point, bool is_boundary_edge) {
    // Skip zero-length segments
    if (bg::equals(start_point, end_point)) {
        return;
    }
    
    size_t from_vertex = findOrCreatePolygonSplitGraphVertex(start_point, is_boundary_edge);
    size_t to_vertex = findOrCreatePolygonSplitGraphVertex(end_point, is_boundary_edge);
    
    if (from_vertex != to_vertex) {  // Additional check to avoid self-loops
        // Check if this edge pair already exists
        PolygonSplitEdgePair edge_pair(from_vertex, to_vertex);
        if (added_polygon_split_edges_.find(edge_pair) != added_polygon_split_edges_.end()) {
            // Edge already exists, skip adding
            return;
        }
        
        // Mark this edge pair as added
        added_polygon_split_edges_.insert(edge_pair);
        
        // Add bidirectional edges
        polygon_split_edges_.emplace_back(from_vertex, to_vertex, is_boundary_edge);
        polygon_split_edges_.emplace_back(to_vertex, from_vertex, is_boundary_edge);
    }
}

std::vector<size_t> ConvexPath::getPolygonSplitBoundaryVertices() const {
    std::vector<size_t> boundary_vertices;
    for (const auto& vertex : polygon_split_vertices_) {
        if (vertex.is_boundary_vertex) {
            boundary_vertices.push_back(vertex.vertex_id);
        }
    }
    return boundary_vertices;
}

std::optional<Point> ConvexPath::getPolygonSplitVertexLocation(size_t vertex_id) const {
    if (vertex_id < polygon_split_vertices_.size()) {
        return polygon_split_vertices_[vertex_id].location;
    }
    return std::nullopt;
}

ConvexPathResult ConvexPath::computeShortestPathOnConvexPathGraph(size_t start_vertex_id, size_t end_vertex_id) {
    // Validate vertex IDs
    if (start_vertex_id >= convex_path_vertices_.size() || end_vertex_id >= convex_path_vertices_.size()) {
        std::cerr << "Invalid vertex IDs. Start vertex ID: " << start_vertex_id 
                  << ", End vertex ID: " << end_vertex_id 
                  << ", Total vertices: " << convex_path_vertices_.size() << std::endl;
        return ConvexPathResult();
    }
    
    // Build adjacency list - only include edges that are not deleted
    std::unordered_map<size_t, std::vector<size_t>> adjacency_list;
    for (size_t i = 0; i < convex_path_edges_.size(); ++i) {
        if (!convex_path_edges_[i].is_deleted) {  // Only include non-deleted edges
            adjacency_list[convex_path_edges_[i].from_vertex].push_back(i);
        }
    }
    
    // Initialize data structures
    std::vector<double> distances(convex_path_vertices_.size(), std::numeric_limits<double>::infinity());
    std::vector<size_t> predecessors(convex_path_vertices_.size(), std::numeric_limits<size_t>::max());  // vertex_id -> edge_index
    std::vector<bool> visited(convex_path_vertices_.size(), false);
    
    distances[start_vertex_id] = 0.0;

    double network_distance = 0.0;
    
    // Priority queue: (distance, vertex_id)
    using QueueEntry = std::pair<double, size_t>;
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> pq;
    pq.push({0.0, start_vertex_id});
    
    // Dijkstra algorithm
    while (!pq.empty()) {
        auto [current_dist, current_vertex] = pq.top();
        pq.pop();
        
        if (visited[current_vertex]) continue;
        visited[current_vertex] = true;
        
        if (current_vertex == end_vertex_id) break;  // Found shortest path to target
        
        // Process neighbors
        if (adjacency_list.find(current_vertex) != adjacency_list.end()) {
            for (size_t edge_idx : adjacency_list[current_vertex]) {
                const auto& edge = convex_path_edges_[edge_idx];
                size_t neighbor = edge.to_vertex;
                
                // Skip if neighbor is already visited
                if (visited[neighbor]) continue;

                double new_dist;
                if (neighbor != 1) {
                    new_dist = current_dist + edge.weight;
                } else if (getIncludeNetworkDistance()) {
                    new_dist = current_dist + edge.weight;
                } else {
                    new_dist = current_dist;
                    if (new_dist < distances[neighbor]) {
                        network_distance = edge.weight;
                    }
                }
                
                if (new_dist < distances[neighbor]) {
                    distances[neighbor] = new_dist;
                    predecessors[neighbor] = edge_idx;
                    pq.push({new_dist, neighbor});
                }
            }
        }
    }
    
    // Check if path was found
    if (distances[end_vertex_id] == std::numeric_limits<double>::infinity()) {
        const auto& start_vertex = convex_path_vertices_[start_vertex_id];
        const auto& end_vertex = convex_path_vertices_[end_vertex_id];
        std::cerr << "No path found between start and end points. "
                  << "Start point: (" << std::fixed << std::setprecision(2) 
                  << bg::get<0>(start_vertex.location) << ", " << bg::get<1>(start_vertex.location) 
                  << ") [vertex_id: " << start_vertex_id << "], "
                  << "End point: (" << bg::get<0>(end_vertex.location) << ", " << bg::get<1>(end_vertex.location) 
                  << ") [vertex_id: " << end_vertex_id << "]" << std::endl;
        return ConvexPathResult();
    }
    
    // Reconstruct path and build LineString simultaneously
    std::vector<size_t> path_edges;
    double objective_distance = distances[end_vertex_id];  // Objective distance from Dijkstra
    
    // Calculate total_length based on include_network_distance flag
    double total_length;
    if (getIncludeNetworkDistance()) {
        // When include_network_distance is true, total_length equals objective_distance (network distance is already included)
        total_length = objective_distance;
    } else {
        // When include_network_distance is false, total_length equals objective_distance + network_distance
        // (network distance is added separately to the objective distance)
        total_length = objective_distance + network_distance;
    }
    
    size_t current = end_vertex_id;
    while (current != start_vertex_id) {
        size_t edge_idx = predecessors[current];
        path_edges.push_back(edge_idx);
        current = convex_path_edges_[edge_idx].from_vertex;
    }
    
    // Reverse to get correct order
    std::reverse(path_edges.begin(), path_edges.end());
    
    // Build LineString from path and capture artificial edge information
    LineString path_geometry;
    size_t nearest_point_vertex_position_index = std::numeric_limits<size_t>::max();
    size_t edge_index = std::numeric_limits<size_t>::max();
    
    if (!path_edges.empty()) {
        // Add first point (start of first edge)
        const auto& first_edge = convex_path_edges_[path_edges[0]];
        const auto& first_vertex = convex_path_vertices_[first_edge.from_vertex];
        path_geometry.push_back(first_vertex.location);
        
        // Add subsequent points (end of each edge) and capture artificial edge info
        for (size_t edge_idx : path_edges) {
            const auto& edge = convex_path_edges_[edge_idx];
            const auto& to_vertex = convex_path_vertices_[edge.to_vertex];
            if (to_vertex.vertex_id != 1) {  // Skip root end vertex (assumed to be 1 and is artificial without real geometry)
                path_geometry.push_back(to_vertex.location);
            } else {
                // Capture information from artificial edge connecting to vertex 1
                if (edge.nearest_point_vertex_position_index != std::numeric_limits<size_t>::max()) {
                    nearest_point_vertex_position_index = edge.nearest_point_vertex_position_index;
                }
                if (edge.edge_index != std::numeric_limits<size_t>::max()) {
                    edge_index = edge.edge_index;
                }
            }
        }
    }
    
    return ConvexPathResult(path_geometry, path_edges, total_length, objective_distance, nearest_point_vertex_position_index, edge_index);
}

std::vector<std::vector<Point>> ConvexPath::splitPolygon(const PolygonFeature& polygon_feature, const LineString& splitting_line) {
    // Clear polygon split graph data structures to ensure clean state
    clearPolygonSplitGraph();
    
    // Logging polygon coordinates and splitting line coordinates with 9 decimal places
    // std::cout << "Splitting line coordinates: ";
    // for (const auto& point : splitting_line) {
    //     std::cout << "(" << std::fixed << std::setprecision(9) << bg::get<0>(point) << ", " << bg::get<1>(point) << ") ";
    // }
    // std::cout << std::endl;
    
    try {
//# Step 1: Get intersection points (they are typically ordered along the polygon exterior ring)
        std::vector<Point> intersection_points;
        bg::intersection(splitting_line, polygon_feature.geometry, intersection_points);
        
        // if (!intersection_points.empty()) {
        //     std::cout << "Intersection points found: " << intersection_points.size() << std::endl;
        //     for (size_t i = 0; i < intersection_points.size(); ++i) {
        //         std::cout << "  Intersection point " << i << ": (" << std::fixed << std::setprecision(9) 
        //                  << bg::get<0>(intersection_points[i]) << ", " << bg::get<1>(intersection_points[i]) << ")" << std::endl;
        //     }
        // }
        
        if (intersection_points.size() < 2) {
            std::cerr << "Warning: Splitting line does not intersect polygon with sufficient points" << std::endl;
            return {};  // Return empty vector
        }
        
//# Step 2: Sort intersection points by their projection parameter t along the splitting line
        // while preserving the first element
        std::vector<std::tuple<size_t, Point, double>> points_with_t;
        points_with_t.reserve(intersection_points.size());
        
        // Determine the reference line segment for projection
        Point ref_start, ref_end;
        if (splitting_line.size() == 4) {
            // Use middle two points (actual start and end) if splitting_line has 4 points
            ref_start = splitting_line[1];
            ref_end = splitting_line[2];
        } else {
            // Fallback to first and last point
            ref_start = splitting_line.front();
            ref_end = splitting_line.back();
        }
        
        // Calculate projection parameter t for each intersection point
        for (size_t i = 0; i < intersection_points.size(); ++i) {
            double dx = bg::get<0>(ref_end) - bg::get<0>(ref_start);
            double dy = bg::get<1>(ref_end) - bg::get<1>(ref_start);
            
            // Calculate the dot product of the vector from line_start to point
            // with the normalized direction vector of the line
            double point_dx = bg::get<0>(intersection_points[i]) - bg::get<0>(ref_start);
            double point_dy = bg::get<1>(intersection_points[i]) - bg::get<1>(ref_start);
            
            // Projection parameter t
            // This formula works for all cases:
            // - Vertical lines (dx = 0): t = point_dy / dy
            // - Horizontal lines (dy = 0): t = point_dx / dx  
            // - Diagonal lines: t = (point_dx * dx + point_dy * dy) / (dx * dx + dy * dy)
            double t = (point_dx * dx + point_dy * dy) / (dx * dx + dy * dy);
            points_with_t.emplace_back(i, intersection_points[i], t);
        }
        
        // Sort by t
        std::sort(points_with_t.begin(), points_with_t.end(), 
                    [](const auto& a, const auto& b) { return std::get<2>(a) < std::get<2>(b); });
        
        // Debug logging of sorted intersection points
        // std::cout << "Sorted intersection points by projection parameter t:" << std::endl;
        // for (size_t i = 0; i < points_with_t.size(); ++i) {
        //     const auto& [original_index, point, t] = points_with_t[i];
        //     std::cout << "  Sorted intersection point " << i << ": (" << std::fixed << std::setprecision(9) 
        //              << bg::get<0>(point) << ", " << bg::get<1>(point) 
        //              << ") t=" << t << " (original index: " << original_index << ")" << std::endl;
        // }

//# Step 3: Make linestrings between consecutive intersection points if they interior intersect with the polygon
    
        std::vector<LineString> intersection_linestrings;
        for (size_t i = 0; i < points_with_t.size() - 1; ++i) {
            size_t current_sorted_idx = i;
            size_t next_sorted_idx = i + 1;

            Point current_point = std::get<1>(points_with_t[current_sorted_idx]);
            Point next_point = std::get<1>(points_with_t[next_sorted_idx]);

            LineString segment;
            segment.push_back(current_point);
            segment.push_back(next_point);
            
            // Check if this linestring crosses or is within the polygon
            bool is_valid_intersection = false;
            if (polygon_feature.shrink_geometry.has_value()) {
                const Polygon& shrunk_polygon = *polygon_feature.shrink_geometry;
                is_valid_intersection = bg::crosses(segment, shrunk_polygon);
            } else {
                // Fallback to original geometry if no shrunk geometry available
                is_valid_intersection = bg::crosses(segment, polygon_feature.geometry) || bg::within(segment, polygon_feature.geometry);
            }

            if (is_valid_intersection) {
                intersection_linestrings.push_back(segment);
                // std::cout << "  Valid intersection linestring " << intersection_linestrings.size() - 1 << ": ";
                // for (const auto& point : segment) {
                //     std::cout << "(" << std::fixed << std::setprecision(9) 
                //              << bg::get<0>(point) << ", " << bg::get<1>(point) << ") ";
                // }
                // std::cout << std::endl;
            }
        }

//# Step 4: Determine where endpoints of intersection line segments should be snapped to on the polygon boundary edges, and add graph edges accordingly
        
        // 1. Initialize a bool array with length equals to the length of polygon_feature.outer_ring, all values initialized to false
        std::vector<bool> boundary_segment_split(polygon_feature.outer_ring.size(), false);
        
        // 2. Construct a 3-level nested loop
        // Outer-most loop: iterate through intersection_linestrings
        for (const auto& intersection_linestring : intersection_linestrings) {
            // Middle loop: iterate through the first and the second point of the linestring
            for (size_t endpoint_idx = 0; endpoint_idx < 2; ++endpoint_idx) {
                Point intersection_endpoint = intersection_linestring[endpoint_idx];
                
                // Inner-most loop: iterate through polygon_feature.outer_ring by index
                for (size_t boundary_idx = 0; boundary_idx < polygon_feature.outer_ring.size(); ++boundary_idx) {
                    // Skip if the bool array of corresponding index is true
                    if (boundary_segment_split[boundary_idx]) {
                        continue;
                    }
                    
                    // Get segment_start and segment_end point for each boundary segment
                    Point segment_start = polygon_feature.outer_ring[boundary_idx];
                    Point segment_end = polygon_feature.outer_ring[(boundary_idx + 1) % polygon_feature.outer_ring.size()];
                    
                    // 3. If the intersection linestring endpoint has bg::distance to the segment within 1e-7
                    bg::model::segment<Point> boundary_segment(segment_start, segment_end);
                    double distance_to_segment = bg::distance(intersection_endpoint, boundary_segment);
                    
                    if (distance_to_segment <= 1e-7) {
                        // (1) Check if the intersection linestring endpoint is within 1e-7 from segment_start or segment_end point
                        double dist_to_start = bg::distance(intersection_endpoint, segment_start);
                        double dist_to_end = bg::distance(intersection_endpoint, segment_end);
                        
                        if (dist_to_start <= 1e-7 || dist_to_end <= 1e-7) {
                            // No need to split polygon boundary edge, continue
                            continue;
                        }
                        
                        // (2) Otherwise, add segments and mark as split
                        // First, ensure intersection_endpoint is created as a vertex with is_boundary_vertex = false
                        findOrCreatePolygonSplitGraphVertex(intersection_endpoint, false);
                        
                        // Add segment for segment_start -> intersection linestring endpoint
                        addSegmentToSplitPolygonGraph(segment_start, intersection_endpoint, true);
                        
                        // Add segment for intersection linestring endpoint -> segment_end
                        addSegmentToSplitPolygonGraph(intersection_endpoint, segment_end, true);
                        
                        // Mark the corresponding element in bool array to true
                        boundary_segment_split[boundary_idx] = true;
                    }
                }
            }
            
            // 4. For a intersection linestring, once both its endpoints are examined, add the intersection linestring as a graph edge with is_boundary_edge set to false
            addSegmentToSplitPolygonGraph(intersection_linestring[0], intersection_linestring[1], false);
        }
        
        // 5. Once the 3-level nested loop is done, loop through polygon_feature.outer_ring, and if the corresponding bool array element is false, add the segment as graph edge
        for (size_t boundary_idx = 0; boundary_idx < polygon_feature.outer_ring.size(); ++boundary_idx) {
            if (!boundary_segment_split[boundary_idx]) {
                Point segment_start = polygon_feature.outer_ring[boundary_idx];
                Point segment_end = polygon_feature.outer_ring[(boundary_idx + 1) % polygon_feature.outer_ring.size()];
                addSegmentToSplitPolygonGraph(segment_start, segment_end, true);
            }
        }

//# Step 5: Graph traversal algorithm
        
        // 1. Initialize data structures
        std::unordered_set<size_t> used_boundary_vertices;
        std::vector<std::vector<Point>> split_polygon_results;
        
        // Build adjacency list from polygon_split_edges_
        std::unordered_map<size_t, std::vector<size_t>> adjacency_list;
        for (const auto& edge : polygon_split_edges_) {
            adjacency_list[edge.from_vertex].push_back(edge.to_vertex);
        }
        
        // 2. Loop through boundary vertices
        std::vector<size_t> boundary_vertices = getPolygonSplitBoundaryVertices();
        for (size_t origin_vertex_id : boundary_vertices) {
            // Skip if vertex already used
            if (used_boundary_vertices.find(origin_vertex_id) != used_boundary_vertices.end()) {
                continue;
            }
            
            // 3. Graph traversal algorithm
            std::queue<size_t> queue;
            std::unordered_map<size_t, size_t> predecessor; // precedence vector
            std::unordered_set<size_t> visited;
            
            // Add origin to queue
            queue.push(origin_vertex_id);
            visited.insert(origin_vertex_id);
            predecessor[origin_vertex_id] = origin_vertex_id; // self-reference for origin
            
            size_t cycle_end_vertex = std::numeric_limits<size_t>::max(); // Track where cycle is found
            size_t iteration = 0;
            
            while (!queue.empty() && cycle_end_vertex == std::numeric_limits<size_t>::max()) {
                size_t current_vertex = queue.front();
                queue.pop();
                
                // Get adjacent vertices
                auto adj_it = adjacency_list.find(current_vertex);
                if (adj_it == adjacency_list.end()) {
                    continue;
                }
                
                const std::vector<size_t>& adjacent_vertices = adj_it->second;
                
                for (size_t next_vertex : adjacent_vertices) {
                    if (visited.find(next_vertex) != visited.end()) {
                        // Check if we're back to origin (termination criteria)
                        if (next_vertex == origin_vertex_id && iteration > 1) {
                            cycle_end_vertex = current_vertex; // Set where cycle is found
                            break;
                        }
                        continue;
                    }
                    
                    // Special handling for iteration 0 (from origin)
                    if (current_vertex == origin_vertex_id && iteration == 0) {
                        // Only add the first adjacent vertex to avoid exploring both directions
                        queue.push(next_vertex);
                        visited.insert(next_vertex);
                        predecessor[next_vertex] = current_vertex;
                        break; // Only process first adjacent vertex
                    }
                    
                    // Normal traversal for other iterations
                    if (iteration > 0) {
                        queue.push(next_vertex);
                        visited.insert(next_vertex);
                        predecessor[next_vertex] = current_vertex;
                    }
                }
                
                iteration++;
            }
            
            // 4. If cycle found, collect polygon boundary
            if (cycle_end_vertex != std::numeric_limits<size_t>::max()) {
                std::vector<Point> polygon_boundary;
                
                // Collect points by back-tracing from cycle_end_vertex to origin
                size_t current = cycle_end_vertex;
                while (current != origin_vertex_id) {
                    auto location_opt = getPolygonSplitVertexLocation(current);
                    if (location_opt.has_value()) {
                        polygon_boundary.push_back(location_opt.value());
                    }
                    
                    // Mark this vertex as used (it's part of the polygon boundary)
                    used_boundary_vertices.insert(current);
                    
                    // Move to predecessor
                    auto pred_it = predecessor.find(current);
                    if (pred_it != predecessor.end() && pred_it->second != current) {
                        current = pred_it->second;
                    } else {
                        break;
                    }
                }
                
                // Add the origin vertex point (appears once, not closed)
                auto origin_location_opt = getPolygonSplitVertexLocation(origin_vertex_id);
                if (origin_location_opt.has_value()) {
                    polygon_boundary.push_back(origin_location_opt.value());
                }
                
                // Mark origin vertex as used
                used_boundary_vertices.insert(origin_vertex_id);
                
                // Add to results
                if (polygon_boundary.size() >= 2) { 
                    split_polygon_results.push_back(polygon_boundary);
                }
            }
        }
        
        return split_polygon_results;
        
    } catch (const std::exception& e) {
        std::cerr << "Error during polygon splitting: " << e.what() << std::endl;
        return {};
    }
}

ConvexHullResult ConvexPath::generateConvexHullSegments(const PolygonFeature& polygon_feature, 
                                                        const Point& start_point, 
                                                        const Point& end_point,
                                                        bool add_direct_connection,
                                                        bool end_point_is_last_path_point) {
    ConvexHullResult result;
    
    // Distance threshold for filtering segments close to start-end line
    const double DISTANCE_THRESHOLD = 1e-7;
    
    // Set to track unique vertex IDs created during hull generation (for deduplication)
    // This avoids creating duplicate connection edges and enables efficient interior intersection checking
    std::unordered_set<size_t> hull_vertex_ids;
    
    // Step 1: Create MultiPoint containing polygon outer ring points + start and end points
    MultiPoint all_points;
    
    // Add all polygon outer ring points
    for (const auto& point : polygon_feature.outer_ring) {
        all_points.push_back(point);
    }
    
    // Add start and end points
    all_points.push_back(start_point);
    all_points.push_back(end_point);
    
    // Step 2: Compute convex hull
    Polygon convex_hull;
    bg::convex_hull(all_points, convex_hull);
    
    // Step 3: Check if start_point or end_point is hull vertices
    size_t start_hull_position_idx = SIZE_MAX;
    size_t end_hull_position_idx = SIZE_MAX;
    
    const auto& hull_outer_ring = bg::exterior_ring(convex_hull);
    for (size_t i = 0; i < hull_outer_ring.size() - 1; ++i) {  // -1 because the first and last elements are the same point (polygon closure)
        const auto& vertex = hull_outer_ring[i];
        if (start_hull_position_idx != SIZE_MAX && end_hull_position_idx != SIZE_MAX) {
            break;
        }
        if (start_hull_position_idx == SIZE_MAX && bg::equals(vertex, start_point)) {
            start_hull_position_idx = i;
        }
        if (end_hull_position_idx == SIZE_MAX && bg::equals(vertex, end_point)) {
            end_hull_position_idx = i;
        }
    }
    
    // Step 4: If either point is within the convex hull, apply splitPolygon
    if (start_hull_position_idx == SIZE_MAX || end_hull_position_idx == SIZE_MAX) {
        result.is_concave_case = true;
        
        // Create the splitting line from start to end point (without extending)
        LineString splitting_line;

        // Calculate direction vector and extend the line
        double dx = bg::get<0>(end_point) - bg::get<0>(start_point);
        double dy = bg::get<1>(end_point) - bg::get<1>(start_point);
        double segment_length = std::sqrt(dx * dx + dy * dy);
        
        // Normalize direction vector
        if (segment_length > 0) {
            dx /= segment_length;
            dy /= segment_length;
        }

        double extension_distance = 10000;

        // Calculate extended points
        Point extended_start(
            bg::get<0>(start_point) - dx * extension_distance,
            bg::get<1>(start_point) - dy * extension_distance
        );
        Point extended_end(
            bg::get<0>(end_point) + dx * extension_distance,
            bg::get<1>(end_point) + dy * extension_distance
        );
        
        // Create the extended line with four points
        splitting_line.push_back(extended_start);
        splitting_line.push_back(start_point);
        splitting_line.push_back(end_point);
        splitting_line.push_back(extended_end);

        auto split_results = splitPolygon(polygon_feature, splitting_line);
        
        // Create a segment from start point to end point for efficient distance checking
        bg::model::segment<Point> start_end_segment(start_point, end_point);

        // For each split "polygon", generate a convex hull using start point and another convex hull using end point
        for (const auto& point_group : split_results) {
            // Construct convex hull using split polygon + start_point
            bg::model::multi_point<Point> start_group_points;
            for (const auto& pt : point_group) {
                start_group_points.push_back(pt);
            }
            start_group_points.push_back(start_point);

            Polygon start_convex_hull;
            bg::convex_hull(start_group_points, start_convex_hull);

            // Process start convex hull segments
            const auto& start_hull_outer_ring = bg::exterior_ring(start_convex_hull);
            if (start_hull_outer_ring.size() > 2) {
                size_t prev_vertex_id = findOrCreateConvexPathGraphVertex(start_hull_outer_ring[0]);
                hull_vertex_ids.insert(prev_vertex_id);

                for (size_t i = 0; i < start_hull_outer_ring.size() - 1; ++i) {
                    Point pt1 = start_hull_outer_ring[i];
                    Point pt2 = start_hull_outer_ring[i + 1];

                    // If both pt1 and pt2 are close to the splitting line, check if the segment interior intersects with the polygon
                    double dist_pt1_to_line = bg::distance(pt1, splitting_line);
                    double dist_pt2_to_line = bg::distance(pt2, splitting_line);

                    if (dist_pt1_to_line <= DISTANCE_THRESHOLD && dist_pt2_to_line <= DISTANCE_THRESHOLD) {
                        LineString segment_for_intersection_check;
                        segment_for_intersection_check.push_back(pt1);
                        segment_for_intersection_check.push_back(pt2);
                        // Check if the segment interior intersects with the polygon
                        if (segmentIntersectsPolygonInterior(segment_for_intersection_check, polygon_feature)) {
                            prev_vertex_id = findOrCreateConvexPathGraphVertex(pt2); // Update prev_vertex_id for next iteration even when skipping
                            hull_vertex_ids.insert(prev_vertex_id);
                            continue;
                        }
                    }

                    // Find or create vertex for pt2 and add candidate edge
                    size_t curr_vertex_id = findOrCreateConvexPathGraphVertex(pt2);
                    hull_vertex_ids.insert(curr_vertex_id);
                    result.candidate_edges.emplace_back(prev_vertex_id, curr_vertex_id, EdgeType::HULL_EDGE);
                    prev_vertex_id = curr_vertex_id;
                }
            }

            // Construct convex hull using split polygon + end_point
            bg::model::multi_point<Point> end_group_points;
            for (const auto& pt : point_group) {
                end_group_points.push_back(pt);
            }
            end_group_points.push_back(end_point);
            
            Polygon end_convex_hull;
            bg::convex_hull(end_group_points, end_convex_hull);

            const auto& end_hull_outer_ring = bg::exterior_ring(end_convex_hull);

            // Determine end point hull position index
            end_hull_position_idx = SIZE_MAX;
            for (size_t i = 0; i < end_hull_outer_ring.size() - 1; ++i) {
                if (bg::equals(end_hull_outer_ring[i], end_point)) {
                    end_hull_position_idx = i;
                    break;
                }
            }

            if (end_hull_position_idx == SIZE_MAX || end_hull_outer_ring.size() < 3) {
                continue;
            }

            size_t end_hull_prev_point_idx = end_hull_position_idx == 0 ? end_hull_outer_ring.size() - 2 : end_hull_position_idx - 1;
            Point end_hull_prev_point = end_hull_outer_ring[end_hull_prev_point_idx];

            size_t end_hull_next_point_idx = end_hull_position_idx == end_hull_outer_ring.size() - 2 ? 0 : end_hull_position_idx + 1;
            Point end_hull_next_point = end_hull_outer_ring[end_hull_next_point_idx];

            size_t loop_start_vertex_id = findOrCreateConvexPathGraphVertex(end_hull_next_point);
            size_t loop_end_vertex_id = findOrCreateConvexPathGraphVertex(end_hull_prev_point);

            bool end_hull_prev_to_be_added = false;
            bool end_hull_next_to_be_added = false;

            size_t end_point_vertex_id = findOrCreateConvexPathGraphVertex(end_point);

            double dist_end_hull_prev_point_to_line = bg::distance(end_hull_prev_point, splitting_line);
            double dist_end_hull_next_point_to_line = bg::distance(end_hull_next_point, splitting_line);
            double dist_end_point_to_line = bg::distance(end_point, splitting_line);

            if (dist_end_hull_prev_point_to_line <= DISTANCE_THRESHOLD && dist_end_point_to_line <= DISTANCE_THRESHOLD) {
                LineString end_hull_prev_end_point_segment;
                end_hull_prev_end_point_segment.push_back(end_hull_prev_point);
                end_hull_prev_end_point_segment.push_back(end_point);
                // Check if the segment interior intersects with the polygon
                if (!segmentIntersectsPolygonInterior(end_hull_prev_end_point_segment, polygon_feature)) {
                    end_hull_prev_to_be_added = true;
                }
            }
            else {
                end_hull_prev_to_be_added = true;
            }

            if (dist_end_hull_next_point_to_line <= DISTANCE_THRESHOLD && dist_end_point_to_line <= DISTANCE_THRESHOLD) {
                LineString end_hull_next_end_point_segment;
                end_hull_next_end_point_segment.push_back(end_hull_next_point);
                end_hull_next_end_point_segment.push_back(end_point);
                // Check if the segment interior intersects with the polygon
                if (!segmentIntersectsPolygonInterior(end_hull_next_end_point_segment, polygon_feature)) {
                    end_hull_next_to_be_added = true;
                }
            }
            else {
                end_hull_next_to_be_added = true;
            }

            if (end_hull_prev_to_be_added) {
                if (!end_point_is_last_path_point || !useRoadData_) {
                    result.candidate_edges.emplace_back(loop_end_vertex_id, end_point_vertex_id, EdgeType::CONNECTION_TO_END);
                }
                else {
                    bool new_snapped_point_added = false;
                    std::vector<std::tuple<Point, double, double, size_t, size_t>> candidate_snapped_points_outside_segment;
                    for (size_t snappable_id : polygon_feature.snappable_ids) {
                        auto [snapped_point, dist_to_nearest_point, dist_to_closest_edge, nearest_point_vertex_position_index, edge_index, is_outside_segment] = computeSnappedPointOnEdge(snappable_id, end_hull_prev_point);
                        if (bg::is_empty(snapped_point)) {
                            continue;
                        }
                        if (is_outside_segment) {
                            candidate_snapped_points_outside_segment.emplace_back(snapped_point, dist_to_nearest_point, dist_to_closest_edge, nearest_point_vertex_position_index, edge_index);
                            continue;
                        }
                        new_snapped_point_added = true;
                        size_t snapped_point_vertex_id = findOrCreateConvexPathGraphVertex(snapped_point);
                        result.candidate_edges.emplace_back(loop_end_vertex_id, snapped_point_vertex_id, EdgeType::CONNECTION_TO_END);
    
                        // Add an artificial edge from snapped_point_vertex_id to root end vertex (assumed to be 1)
                        ConvexPathEdgePair edge_pair(snapped_point_vertex_id, 1);
                        if (added_convex_path_edges_.find(edge_pair) == added_convex_path_edges_.end()) {
                            added_convex_path_edges_.insert(edge_pair);
                            convex_path_edges_.emplace_back(snapped_point_vertex_id, 1, dist_to_nearest_point, false, EdgeType::CONNECTION_TO_END, nearest_point_vertex_position_index, edge_index);
                        }
                    }
                    if (!new_snapped_point_added && candidate_snapped_points_outside_segment.size() > 0) {
                        for (const auto& [snapped_point, dist_to_nearest_point, dist_to_closest_edge, nearest_point_vertex_position_index, edge_index] : candidate_snapped_points_outside_segment) {
                            size_t snapped_point_vertex_id = findOrCreateConvexPathGraphVertex(snapped_point);
                            result.candidate_edges.emplace_back(loop_end_vertex_id, snapped_point_vertex_id, EdgeType::CONNECTION_TO_END);

                            // Add an artificial edge from snapped_point_vertex_id to root end vertex (assumed to be 1)
                            ConvexPathEdgePair edge_pair(snapped_point_vertex_id, 1);
                            if (added_convex_path_edges_.find(edge_pair) == added_convex_path_edges_.end()) {
                                added_convex_path_edges_.insert(edge_pair);
                                convex_path_edges_.emplace_back(snapped_point_vertex_id, 1, dist_to_nearest_point, false, EdgeType::CONNECTION_TO_END, nearest_point_vertex_position_index, edge_index);
                            }
                        }
                    }
                }
            }

            if (end_hull_next_to_be_added) {
                if (!end_point_is_last_path_point || !useRoadData_) {
                    result.candidate_edges.emplace_back(loop_start_vertex_id, end_point_vertex_id, EdgeType::CONNECTION_TO_END);
                }
                else {
                    bool new_snapped_point_added2 = false;
                    std::vector<std::tuple<Point, double, double, size_t, size_t>> candidate_snapped_points_outside_segment2;
                    for (size_t snappable_id : polygon_feature.snappable_ids) {
                        auto [snapped_point2, dist_to_nearest_point2, dist_to_closest_edge2, nearest_point_vertex_position_index2, edge_index2, is_outside_segment2] = computeSnappedPointOnEdge(snappable_id, end_hull_next_point);
                        if (bg::is_empty(snapped_point2)) {
                            continue;
                        }
                        if (is_outside_segment2) {
                            candidate_snapped_points_outside_segment2.emplace_back(snapped_point2, dist_to_nearest_point2, dist_to_closest_edge2, nearest_point_vertex_position_index2, edge_index2);
                            continue;
                        }
                        new_snapped_point_added2 = true;
                        size_t snapped_point2_vertex_id = findOrCreateConvexPathGraphVertex(snapped_point2);
                        result.candidate_edges.emplace_back(loop_start_vertex_id, snapped_point2_vertex_id, EdgeType::CONNECTION_TO_END);

                        // Add an artificial edge from snapped_point2_vertex_id to root end vertex (assumed to be 1)
                        ConvexPathEdgePair edge_pair(snapped_point2_vertex_id, 1);
                        if (added_convex_path_edges_.find(edge_pair) == added_convex_path_edges_.end()) {
                            added_convex_path_edges_.insert(edge_pair);
                            convex_path_edges_.emplace_back(snapped_point2_vertex_id, 1, dist_to_nearest_point2, false, EdgeType::CONNECTION_TO_END, nearest_point_vertex_position_index2, edge_index2);
                        }
                    }
                    if (!new_snapped_point_added2 && candidate_snapped_points_outside_segment2.size() > 0) {
                        for (const auto& [snapped_point2, dist_to_nearest_point2, dist_to_closest_edge2, nearest_point_vertex_position_index2, edge_index2] : candidate_snapped_points_outside_segment2) {
                            size_t snapped_point2_vertex_id = findOrCreateConvexPathGraphVertex(snapped_point2);
                            result.candidate_edges.emplace_back(loop_start_vertex_id, snapped_point2_vertex_id, EdgeType::CONNECTION_TO_END);

                            // Add an artificial edge from snapped_point2_vertex_id to root end vertex (assumed to be 1)
                            ConvexPathEdgePair edge_pair(snapped_point2_vertex_id, 1);
                            if (added_convex_path_edges_.find(edge_pair) == added_convex_path_edges_.end()) {
                                added_convex_path_edges_.insert(edge_pair);
                                convex_path_edges_.emplace_back(snapped_point2_vertex_id, 1, dist_to_nearest_point2, false, EdgeType::CONNECTION_TO_END, nearest_point_vertex_position_index2, edge_index2);
                            }
                        }
                    }
                }
            }
        }
    }
    else {
        result.is_concave_case = false;
        
        if (hull_outer_ring.size() > 2) {
            if (!end_point_is_last_path_point || !useRoadData_) {
                // Find or create vertex for the first point outside the loop
                size_t prev_vertex_id = findOrCreateConvexPathGraphVertex(hull_outer_ring[0]);
                hull_vertex_ids.insert(prev_vertex_id);

                for (size_t i = 0; i < hull_outer_ring.size() - 1; ++i) {
                    Point pt2 = hull_outer_ring[i + 1];
                    
                    // Find or create vertex for pt2 and add candidate edge
                    size_t curr_vertex_id = findOrCreateConvexPathGraphVertex(pt2);
                    hull_vertex_ids.insert(curr_vertex_id);
                    result.candidate_edges.emplace_back(prev_vertex_id, curr_vertex_id, EdgeType::HULL_EDGE);
                    prev_vertex_id = curr_vertex_id;
                }
            }
            else {
                size_t end_hull_prev_point_idx = end_hull_position_idx == 0 ? hull_outer_ring.size() - 2 : end_hull_position_idx - 1;
                Point end_hull_prev_point = hull_outer_ring[end_hull_prev_point_idx];
                size_t loop_end_vertex_id = findOrCreateConvexPathGraphVertex(end_hull_prev_point);
                
                size_t end_hull_next_point_idx = end_hull_position_idx == hull_outer_ring.size() - 2 ? 0 : end_hull_position_idx + 1;
                Point end_hull_next_point = hull_outer_ring[end_hull_next_point_idx];
                size_t loop_start_vertex_id = findOrCreateConvexPathGraphVertex(end_hull_next_point);

                bool new_snapped_point_added = false;
                std::vector<std::tuple<Point, double, double, size_t, size_t>> candidate_snapped_points_outside_segment;
                bool new_snapped_point_added2 = false;
                std::vector<std::tuple<Point, double, double, size_t, size_t>> candidate_snapped_points_outside_segment2;
                for (size_t snappable_id : polygon_feature.snappable_ids) {
                    auto [snapped_point, dist_to_nearest_point, dist_to_closest_edge, nearest_point_vertex_position_index, edge_index, is_outside_segment] = computeSnappedPointOnEdge(snappable_id, end_hull_prev_point);
                    if (!bg::is_empty(snapped_point)) {
                        if (is_outside_segment) {
                            candidate_snapped_points_outside_segment.emplace_back(snapped_point, dist_to_nearest_point, dist_to_closest_edge, nearest_point_vertex_position_index, edge_index);
                        }
                        else {
                            new_snapped_point_added = true;
                            size_t snapped_point_vertex_id = findOrCreateConvexPathGraphVertex(snapped_point);
                            result.candidate_edges.emplace_back(loop_end_vertex_id, snapped_point_vertex_id, EdgeType::CONNECTION_TO_END);

                            // Add an artificial edge from snapped_point_vertex_id to root end vertex (assumed to be 1)
                            ConvexPathEdgePair edge_pair(snapped_point_vertex_id, 1);
                            if (added_convex_path_edges_.find(edge_pair) == added_convex_path_edges_.end()) {
                                added_convex_path_edges_.insert(edge_pair);
                                convex_path_edges_.emplace_back(snapped_point_vertex_id, 1, dist_to_nearest_point, false, EdgeType::CONNECTION_TO_END, nearest_point_vertex_position_index, edge_index);
                            }
                        }
                    }

                    auto [snapped_point2, dist_to_nearest_point2, dist_to_closest_edge2, nearest_point_vertex_position_index2, edge_index2, is_outside_segment2] = computeSnappedPointOnEdge(snappable_id, end_hull_next_point);
                    if (!bg::is_empty(snapped_point2)) {
                        if (is_outside_segment2) {
                            candidate_snapped_points_outside_segment2.emplace_back(snapped_point2, dist_to_nearest_point2, dist_to_closest_edge2, nearest_point_vertex_position_index2, edge_index2);
                        }
                        else {
                            new_snapped_point_added2 = true;
                            size_t snapped_point2_vertex_id = findOrCreateConvexPathGraphVertex(snapped_point2);
                            result.candidate_edges.emplace_back(loop_start_vertex_id, snapped_point2_vertex_id, EdgeType::CONNECTION_TO_END);

                            // Add an artificial edge from snapped_point2_vertex_id to root end vertex (assumed to be 1)
                            ConvexPathEdgePair edge_pair(snapped_point2_vertex_id, 1);
                            if (added_convex_path_edges_.find(edge_pair) == added_convex_path_edges_.end()) {
                                added_convex_path_edges_.insert(edge_pair);
                                convex_path_edges_.emplace_back(snapped_point2_vertex_id, 1, dist_to_nearest_point2, false, EdgeType::CONNECTION_TO_END, nearest_point_vertex_position_index2, edge_index2);
                            }
                        }
                    }
                }
                if (!new_snapped_point_added && candidate_snapped_points_outside_segment.size() > 0) {
                    for (const auto& [snapped_point, dist_to_nearest_point, dist_to_closest_edge, nearest_point_vertex_position_index, edge_index] : candidate_snapped_points_outside_segment) {
                        size_t snapped_point_vertex_id = findOrCreateConvexPathGraphVertex(snapped_point);
                        result.candidate_edges.emplace_back(loop_end_vertex_id, snapped_point_vertex_id, EdgeType::CONNECTION_TO_END);

                        // Add an artificial edge from snapped_point_vertex_id to root end vertex (assumed to be 1)
                        ConvexPathEdgePair edge_pair(snapped_point_vertex_id, 1);
                        if (added_convex_path_edges_.find(edge_pair) == added_convex_path_edges_.end()) {
                            added_convex_path_edges_.insert(edge_pair);
                            convex_path_edges_.emplace_back(snapped_point_vertex_id, 1, dist_to_nearest_point, false, EdgeType::CONNECTION_TO_END, nearest_point_vertex_position_index, edge_index);
                        }
                    }
                }
                if (!new_snapped_point_added2 && candidate_snapped_points_outside_segment2.size() > 0) {
                    for (const auto& [snapped_point2, dist_to_nearest_point2, dist_to_closest_edge2, nearest_point_vertex_position_index2, edge_index2] : candidate_snapped_points_outside_segment2) {
                        size_t snapped_point2_vertex_id = findOrCreateConvexPathGraphVertex(snapped_point2);
                        result.candidate_edges.emplace_back(loop_start_vertex_id, snapped_point2_vertex_id, EdgeType::CONNECTION_TO_END);

                        // Add an artificial edge from snapped_point2_vertex_id to root end vertex (assumed to be 1)
                        ConvexPathEdgePair edge_pair(snapped_point2_vertex_id, 1);
                        if (added_convex_path_edges_.find(edge_pair) == added_convex_path_edges_.end()) {
                            added_convex_path_edges_.insert(edge_pair);
                            convex_path_edges_.emplace_back(snapped_point2_vertex_id, 1, dist_to_nearest_point2, false, EdgeType::CONNECTION_TO_END, nearest_point_vertex_position_index2, edge_index2);
                        }
                    }
                }

                // Loop through hull_outer_ring with end_hull_next_point_idx and end_hull_prev_point_idx as start and end points
                // We need to create hull edges that connect the points in sequence, but exclude the two hull segments
                // that would connect to end_hull_position_idx (end_point_is_last_path_point case)
                
                // Start from end_hull_next_point_idx and go clockwise to end_hull_prev_point_idx
                // Note: hull_outer_ring has first and last elements as the same point (polygon closure)
                size_t current_idx = end_hull_next_point_idx;
                size_t prev_vertex_id = loop_start_vertex_id;
                
                // Loop until we reach end_hull_prev_point_idx, creating hull edges along the way
                while (current_idx != end_hull_prev_point_idx) {
                    // Get the next point in the clockwise direction
                    size_t next_idx;
                    if (current_idx == hull_outer_ring.size() - 2) {
                        // We're at the second-to-last point, next should be 0 (first point, not the duplicate closing point)
                        next_idx = 0;
                    } else {
                        next_idx = current_idx + 1;
                    }
                    
                    Point current_point = hull_outer_ring[current_idx];
                    Point next_point = hull_outer_ring[next_idx];
                    
                    // Find or create vertex for the next point
                    size_t next_vertex_id = loop_end_vertex_id;
                    if (next_idx != end_hull_prev_point_idx) {
                        next_vertex_id = findOrCreateConvexPathGraphVertex(next_point);
                        hull_vertex_ids.insert(next_vertex_id);
                    }
                    
                    // Add hull edge from current to next point
                    result.candidate_edges.emplace_back(prev_vertex_id, next_vertex_id, EdgeType::HULL_EDGE);
                    
                    // Update for next iteration
                    prev_vertex_id = next_vertex_id;
                    current_idx = next_idx;
                    
                    // Break if we've reached end_hull_prev_point_idx
                    if (current_idx == end_hull_prev_point_idx) {
                        break;
                    }
                }
            }
        }
    }
    
    // Add connection edges to root start and end vertices if requested
    // These connections allow the path to reach the hull vertices from the root start/end points
    if (add_direct_connection && !hull_vertex_ids.empty()) {
        const size_t ROOT_START_VERTEX = 0;  // Root start vertex (index 0)
        const size_t ROOT_END_VERTEX = 1;    // Root end vertex (index 1)
        
        for (size_t hull_vertex_id : hull_vertex_ids) {
            // Skip if this is already one of the root vertices
            if (hull_vertex_id == ROOT_START_VERTEX || hull_vertex_id == ROOT_END_VERTEX) {
                continue;
            }
            
            // Add connection to root start vertex
            result.candidate_edges.emplace_back(hull_vertex_id, ROOT_START_VERTEX, EdgeType::CONNECTION_TO_START);
            
            // Add connection to root end vertex
            // result.candidate_edges.emplace_back(hull_vertex_id, ROOT_END_VERTEX, EdgeType::CONNECTION_TO_END);
        }
    }
    
    return result;
}

ConvexPathResult ConvexPath::findConvexPath(const Point& start_point, const std::vector<size_t>& snappable_ids) {
    // Step 1: Try to snap the point to road edge
    clear();

    size_t root_start_vertex_id = findOrCreateConvexPathGraphVertex(start_point);

    // Manually add an artificial end vertex
    size_t root_end_vertex_id = next_convex_path_vertex_id_++;
    convex_path_vertices_.emplace_back(start_point, root_end_vertex_id);  // Technically, the geometry of this vertex is dummy and use start point as a placeholder
    // Do not insert into r-tree as this vertex to avoid conflict with initial_snapped_point_vertex_id

    if (useRoadData_) {
        // For each possible snappable road edge, perform sanpping and add a convex path edge to account all possible network paths
        // Prioritizing adding edges that have snapped points on the road segments
        // Only add edges that have snapped points outside the segment if no other edges are added
        std::vector<std::tuple<Point, double, double, size_t, size_t>> candidate_snapped_points_outside_segment;
        for (size_t snappable_id : snappable_ids) {
            auto [initial_snapped_point, initial_dist_to_nearest_point, initial_dist_to_closest_edge, initial_nearest_point_vertex_position_index, initial_edge_index, initial_is_outside_segment] = computeSnappedPointOnEdge(snappable_id, start_point);
            if (bg::is_empty(initial_snapped_point)) {
                continue;
            }
            if (initial_is_outside_segment) {
                candidate_snapped_points_outside_segment.emplace_back(initial_snapped_point, initial_dist_to_nearest_point, initial_dist_to_closest_edge, initial_nearest_point_vertex_position_index, initial_edge_index);
                continue;
            }

            size_t initial_snapped_point_vertex_id = next_convex_path_vertex_id_++;
            convex_path_vertices_.emplace_back(initial_snapped_point, initial_snapped_point_vertex_id);
            convex_path_vertex_rtree_.insert(std::make_pair(initial_snapped_point, initial_snapped_point_vertex_id));

            // Add the initial direct connection between start and snapped points
            addSegmentToConvexPathGraph(root_start_vertex_id, initial_snapped_point_vertex_id, false); // First edge, no need to check duplicate

            // Manually add the edge between snapped point and artificial vertex
            ConvexPathEdgePair second_edge_pair(initial_snapped_point_vertex_id, root_end_vertex_id);
            added_convex_path_edges_.insert(second_edge_pair);
            convex_path_edges_.emplace_back(initial_snapped_point_vertex_id, root_end_vertex_id, initial_dist_to_nearest_point, false, EdgeType::CONNECTION_TO_END, initial_nearest_point_vertex_position_index, initial_edge_index);
            
        }

        // If no snapped point that is on the segment is found, we need to add edges that have snapped points outside the segment
        if (next_convex_path_vertex_id_ == 2) {
            if (candidate_snapped_points_outside_segment.size() > 0) {
                for (const auto& [initial_snapped_point, initial_dist_to_nearest_point, initial_dist_to_closest_edge, initial_nearest_point_vertex_position_index, initial_edge_index] : candidate_snapped_points_outside_segment) {
                    size_t initial_snapped_point_vertex_id = next_convex_path_vertex_id_++;
                    convex_path_vertices_.emplace_back(initial_snapped_point, initial_snapped_point_vertex_id);
                    convex_path_vertex_rtree_.insert(std::make_pair(initial_snapped_point, initial_snapped_point_vertex_id));

                    // Add the initial direct connection between start and snapped points
                    addSegmentToConvexPathGraph(root_start_vertex_id, initial_snapped_point_vertex_id, false); // First edge, no need to check duplicate

                    // Manually add the edge between snapped point and artificial vertex
                    ConvexPathEdgePair second_edge_pair(initial_snapped_point_vertex_id, root_end_vertex_id);
                    added_convex_path_edges_.insert(second_edge_pair);
                    convex_path_edges_.emplace_back(initial_snapped_point_vertex_id, root_end_vertex_id, initial_dist_to_nearest_point, false, EdgeType::CONNECTION_TO_END, initial_nearest_point_vertex_position_index, initial_edge_index);
                    
                }
            }
            else {
                // Return empty result when snapping fails
                return ConvexPathResult();
            }
        }
    } else {
        // Process access points (no road data)
        if (snappable_ids.empty()) {
            // Return empty result when no snappable points
            return ConvexPathResult();
        }

        for (size_t snappable_id : snappable_ids) {
            auto access_point_opt = getAccessPoint(snappable_id);
            if (!access_point_opt.has_value()) {
                std::cerr << "Warning: Access point with index " << snappable_id << " not found" << std::endl;
                continue;
            }
            
            const auto& access_point = access_point_opt.value();
            const Point& access_point_location = access_point.geometry;
            
            // Create vertex for access point
            size_t access_point_vertex_id = next_convex_path_vertex_id_++;
            convex_path_vertices_.emplace_back(access_point_location, access_point_vertex_id);
            convex_path_vertex_rtree_.insert(std::make_pair(access_point_location, access_point_vertex_id));

            // Add the initial direct connection between start and access point
            addSegmentToConvexPathGraph(root_start_vertex_id, access_point_vertex_id, false); // First edge, no need to check duplicate

            // Manually add the edge between access point and artificial vertex
            ConvexPathEdgePair second_edge_pair(access_point_vertex_id, root_end_vertex_id);
            added_convex_path_edges_.insert(second_edge_pair);
            convex_path_edges_.emplace_back(access_point_vertex_id, root_end_vertex_id, 0.0, false, EdgeType::CONNECTION_TO_END, snappable_id, snappable_id);
        }
    }

    // Initialize iteration state
    IterationState iteration_state;
    
    // Step 2: Run first iteration to identify direct obstacles
    // Direct obstacles are those that intersect with the initial straight-line path
    auto [path_result, obstacle_count] = identifyAndResolveCrossingObstaclesOnShortestPath(iteration_state, false);

    // Reset iteration counts for direct obstacles (they are considered as direct obstacles)
    // Subsequent obstacles found will be considered indirect obstacles
    iteration_state.current_obstacle_count = 0;
    iteration_state.previous_obstacle_count = 0;
    
    // Step 3: If no obstacles found, return the direct path
    if (obstacle_count == 0) {
        return path_result;
    }
    
    // Step 4: Continue iterating until no more obstacles are found (indirect obstacles)
    // Indirect obstacles are those that intersect with the path after adding convex hull segments
    while (true) {
        auto [current_path_result, current_obstacle_count] = identifyAndResolveCrossingObstaclesOnShortestPath(iteration_state, true);
        
        // If no more obstacles found, we have our final path
        if (current_obstacle_count == 0) {
            // Debug logging of number of edges in the graph
            // std::cout << "Number of edges in the graph: " << convex_path_edges_.size() << std::endl;
            return current_path_result;
        }
        
        // Update path result for next iteration
        path_result = current_path_result;
    }
    
    // Return the best path found (just a syntax fallback, should never be reached)
    return path_result;
}

void ConvexPath::updateGlobalObstacleSets(const std::vector<PolygonFeature>& obstacles,
                                         IterationState& iteration_state) {
    for (const auto& obstacle : obstacles) {
        // If building obstacle is never seen in previous iteration, add it
        if (iteration_state.seen_building_ids.find(obstacle.feature_id) == iteration_state.seen_building_ids.end()) {
            iteration_state.seen_building_ids.insert(obstacle.feature_id);  // prevent duplicate add to global_obstacles
            iteration_state.global_obstacles.push_back(obstacle);
        }
    }
}

void ConvexPath::decideEdgesToAdd(const std::vector<ConvexHullResult>& hull_results,
                                 const IterationState& iteration_state,
                                 bool is_indirect_obstacle) {
    // For direct obstacles, add all candidate edges without filtering
    // Direct obstacles are the primary obstacles that must be avoided, so we add all hull and connection edges
    if (!is_indirect_obstacle) {
        for (const auto& hull_result : hull_results) {
            for (const auto& candidate : hull_result.candidate_edges) {
                addSegmentToConvexPathGraph(candidate.from_vertex_id, candidate.to_vertex_id, true);
            }
        }
        return;
    }
    
    // For indirect obstacles, apply complex filtering logic
    // Indirect obstacles may require more careful edge selection to avoid creating new conflicts
    // Process each hull result individually
    for (const auto& hull_result : hull_results) {
        if (!hull_result.is_concave_case) {
            // Convex case logic for this specific result
            for (const auto& candidate : hull_result.candidate_edges) {
                if (candidate.type == EdgeType::HULL_EDGE || candidate.type == EdgeType::CONNECTION_TO_END) {
                    addSegmentToConvexPathGraph(candidate.from_vertex_id, candidate.to_vertex_id, true);
                } else { // CONNECTION_TO_START
                    if (!checkInteriorIntersectWithVectorOfObstacles(candidate, iteration_state.global_obstacles)) {
                        addSegmentToConvexPathGraph(candidate.from_vertex_id, candidate.to_vertex_id, false);
                    }
                }
            }
        } else {
            // Concave case logic for this specific result
            // Use line-polygon overlay with current iteration obstacles (psi_check)
            for (const auto& candidate : hull_result.candidate_edges) {
                if (candidate.type == EdgeType::HULL_EDGE || candidate.type == EdgeType::CONNECTION_TO_END) {
                    addSegmentToConvexPathGraph(candidate.from_vertex_id, candidate.to_vertex_id, true);
                } else { // CONNECTION_TO_START
                    if (!checkInteriorIntersectWithVectorOfObstacles(candidate, iteration_state.current_obstacles)) {
                        addSegmentToConvexPathGraph(candidate.from_vertex_id, candidate.to_vertex_id, false);
                    }
                }
            }
        }
    }
}

bool ConvexPath::checkInteriorIntersectWithVectorOfObstacles(const CandidateEdge& edge,
                                                            const std::vector<PolygonFeature>& obstacles) {
    // Check if this edge already exists in added_edges_ - if so, return true to avoid adding it again
    ConvexPathEdgePair edge_pair(edge.from_vertex_id, edge.to_vertex_id);
    if (added_convex_path_edges_.find(edge_pair) != added_convex_path_edges_.end()) {
        return true;  // Edge already exists, treat as intersecting to avoid re-adding
    }
    
    // Create segment and check against the provided obstacles
    const auto& from_vertex = convex_path_vertices_[edge.from_vertex_id];
    const auto& to_vertex = convex_path_vertices_[edge.to_vertex_id];
    
    LineString edge_segment;
    edge_segment.push_back(from_vertex.location);
    edge_segment.push_back(to_vertex.location);
    
    for (const auto& obstacle : obstacles) {
        if (segmentIntersectsPolygonInterior(edge_segment, obstacle)) {
            return true;
        }
    }
    
    return false;
}

bool ConvexPath::segmentIntersectsPolygonInterior(const LineString& segment, 
                                                 const PolygonFeature& polygon_feature) {
    // Use pre-computed shrink geometry for performance optimization
    if (polygon_feature.shrink_geometry.has_value()) {
        // Use pre-computed shrunk geometry - no runtime buffering needed!
        const Polygon& shrunk_polygon = *polygon_feature.shrink_geometry;
        
        // For my application, an interior intersect between a segment and a polygon means following:
        // 1. If disjoint (segment does not share interior nor exterior point with polygon), false
        // 2. If segment only touches a vertex of the polygon, false
        // 3. If segment only overlaps an edge of the polygon (sharing exterior boundary of the polygon), false
        // 4. If segment crosses the exterior boundary of the polygon once (shares only one exterior point and a small number of interior points), and this is just one endpoint of the segment is slightly within the polygon due to precision issue, false
        // 5. If segment is within polygon (e.g., one or two shared exterior points, and a lot of interior points), this typically is like a long linestring clipped to the polygon boundary, true
        // 6. If segment crosses the polygon (two shared exterior points, and a lot of interior points), true
        if (bg::intersects(segment, shrunk_polygon)) {
            return true;  // Genuine interior intersection
        }
    } else {
        // Fallback to original logic if shrink geometry computation failed
        if (bg::crosses(segment, polygon_feature.geometry) || 
            bg::within(segment, polygon_feature.geometry)) {
            return true;
        }
    }
    
    return false;
}

std::pair<ConvexPathResult, size_t> ConvexPath::identifyAndResolveCrossingObstaclesOnShortestPath(
    IterationState& iteration_state,
    bool is_indirect_obstacle) {
    
    // Initialize result vector and sets for uniqueness tracking
    std::vector<PolygonFeature> all_crossing_obstacles;
    std::unordered_set<size_t> seen_building_ids;        // Track unique building feature IDs
    
    // Run computeShortestPath with default parameters (0, 1)
    ConvexPathResult path_result = computeShortestPathOnConvexPathGraph(0,1);
    
    // Check if path was found
    if (!path_result.path_found || path_result.path_geometry.empty()) {
        return std::make_pair(path_result, 0);  // Return path result with 0 obstacles if no path found
    }

    // Store convex hull results that will be used to decide edges to add
    std::vector<ConvexHullResult> hull_results;
    
    // Loop through the path geometry to create segments
    // when to_vertex.vertex_id is 1, it is supposed to be along road network and is artificial edge; it is not added to path geometry in path finding method
    const auto& path_points = path_result.path_geometry;
    bool end_point_is_last_path_point = false;
    for (size_t i = 0; i < path_points.size() - 1; ++i) {
        // Create segment from current point to next point
        LineString segment;
        segment.push_back(path_points[i]);
        segment.push_back(path_points[i + 1]);
        
        // Find crossing obstacles for this segment
        auto obstacles = building_reader_ -> queryPolygonRTree(segment);
        
        // If obstacles were found, process them
        if (!obstacles.empty()) {
            // Add unique obstacles to result collection
            for (const auto& obstacle : obstacles) {
                // Check building set
                if (seen_building_ids.find(obstacle.feature_id) == seen_building_ids.end()) {
                    seen_building_ids.insert(obstacle.feature_id);
                    all_crossing_obstacles.push_back(obstacle);
                }
            }

            // Mark the segment as deleted
            // The segment corresponds to edge at index i in the path
            if (i < path_result.edge_indices.size()) {
                size_t edge_idx = path_result.edge_indices[i];
                if (edge_idx < convex_path_edges_.size()) {
                    convex_path_edges_[edge_idx].is_deleted = true;
                }
            }

            // Loop through all obstacles and apply generateConvexHullSegments to each obstacle
            for (const auto& obstacle : obstacles) {
                end_point_is_last_path_point = (path_points.size() - 2 == i);
                ConvexHullResult hull_result = generateConvexHullSegments(obstacle, path_points[i], path_points[i + 1], is_indirect_obstacle, end_point_is_last_path_point);
                hull_results.push_back(hull_result);
            }
        }
    }
    
    // Update global obstacle sets and iteration state
    updateGlobalObstacleSets(all_crossing_obstacles, iteration_state);
    iteration_state.current_obstacles = all_crossing_obstacles;  // Store current iteration obstacles
    iteration_state.updateIteration(all_crossing_obstacles.size());
    
    // Decide which edges to add based on the hull results
    if (!hull_results.empty()) {
        decideEdgesToAdd(hull_results, iteration_state, is_indirect_obstacle);
    }
    
    return std::make_pair(path_result, all_crossing_obstacles.size());
}

std::tuple<std::vector<ConvexPathResult>, size_t, Point> ConvexPath::processSinglePolygon(const PolygonFeature& polygon_feature) {
    std::vector<ConvexPathResult> vertex_paths;
    vertex_paths.reserve(polygon_feature.outer_ring.size() + 1);

    // Step 1: Compute convex paths to each vertex of the polygon
    double max_distance = 0.0;
    double max_total_length = 0.0;
    LineString longest_path;
    size_t nearest_point_vertex_position_index_with_longest_path = SIZE_MAX;
    size_t edge_index_with_longest_path = SIZE_MAX;
    MultiPoint all_path_points; // For computing a convex hull around all the path points

    for (size_t i = 0; i < polygon_feature.outer_ring.size(); ++i) { // polygon_feature.outer_ring already has last vertex dropped if it is the same as the first vertex
        const Point& vertex = polygon_feature.outer_ring[i];
        
        auto path_result = findConvexPath(vertex, polygon_feature.snappable_ids);

        if (path_result.path_found) {
            path_result.start_point_type = ConvexPathResultType::BUILDING_CORNER;

            // Track the vertex that leads to the longest path to avoid unnecessary least accessible point finding
            if (path_result.objective_distance > max_distance) {
                max_distance = path_result.objective_distance;
                max_total_length = path_result.total_length;
                longest_path = path_result.path_geometry;
                nearest_point_vertex_position_index_with_longest_path = path_result.nearest_point_vertex_position_index;
                edge_index_with_longest_path = path_result.edge_index;
            }

            if (!path_result.path_geometry.empty()) {
                for (const auto& point : path_result.path_geometry) {
                    all_path_points.push_back(point);
                }
            }
        }
        
        vertex_paths.push_back(path_result); // add regardless if a path if found or not
    }
    
    // Step 2: Compute convex hull polygon using all the points in vertex_paths
    Polygon convex_hull;
    if (!all_path_points.empty()) {
        bg::convex_hull(all_path_points, convex_hull);
    } else {
        std::cout << "  Warning: No valid path points found for convex hull" << std::endl;
        clear();
        // Return empty result when no valid path points found
        return std::make_tuple(std::vector<ConvexPathResult>(), polygon_feature.feature_id, Point());
    }
    
    // Step 3: For each edge of the polygon, find the least accessible point
    for (size_t i = 0; i < polygon_feature.outer_ring.size(); ++i) {
        size_t next_i = (i + 1) % polygon_feature.outer_ring.size();
        
        const Point& point_i = polygon_feature.outer_ring[i];
        const Point& point_next = polygon_feature.outer_ring[next_i];
        
        // Skip if either path is not found
        if (!vertex_paths[i].path_found || !vertex_paths[next_i].path_found) {
            std::cout << "    Warning: one or both vertex paths not found for edge " << i << " -> " << next_i << std::endl;
            if (!vertex_paths[i].path_found) {
                std::cout << "      Vertex " << i << " path not found. Coordinates: (" 
                          << std::fixed << std::setprecision(9) 
                          << bg::get<0>(point_i) << ", " << bg::get<1>(point_i) << ")" << std::endl;
            }
            if (!vertex_paths[next_i].path_found) {
                std::cout << "      Vertex " << next_i << " path not found. Coordinates: (" 
                          << std::fixed << std::setprecision(9) 
                          << bg::get<0>(point_next) << ", " << bg::get<1>(point_next) << ")" << std::endl;
            }
            continue;
        }
        
        // Building edge is traversed by a convex path, the overall least accessible point will not be on this segment
        // if the second last point in vertex_path[i].path_geometry is the same point as next_i, OR
        // if the second last point in vertex_path[next_i].path_geometry is the same point as i, then skip this edge
        if (vertex_paths[i].path_geometry.size() > 1 && 
            bg::equals(vertex_paths[i].path_geometry[vertex_paths[i].path_geometry.size() - 2], point_next)) {
            continue;
        }
        if (vertex_paths[next_i].path_geometry.size() > 1 && 
            bg::equals(vertex_paths[next_i].path_geometry[vertex_paths[next_i].path_geometry.size() - 2], point_i)) {
            continue;
        }

        // Check potential improvement condition
        double min_endpoint_distance = std::min(vertex_paths[i].objective_distance, vertex_paths[next_i].objective_distance);
        double segment_length = bg::distance(point_i, point_next);
        if (segment_length + min_endpoint_distance <= max_distance) {
            continue;
        }
        
        // Check if segment has interior intersection with convex hull
        LineString segment_line;
        segment_line.push_back(point_i);
        segment_line.push_back(point_next);
        bool segment_within_hull = bg::within(segment_line, convex_hull);
        
        if (polygon_feature.interpolated_snappable_edge_indices_map.find(i) != polygon_feature.interpolated_snappable_edge_indices_map.end()) {
            // Use sectional search for boundary segment with nearest road edge not detected with building corner
            std::vector<size_t> merged_snappable_ids = polygon_feature.snappable_ids;
            const auto& interpolated_edges = polygon_feature.interpolated_snappable_edge_indices_map.at(i);
            merged_snappable_ids.insert(merged_snappable_ids.end(),
                                           interpolated_edges.begin(),
                                           interpolated_edges.end());
            auto search_result = sectionalSearch(point_i, point_next, 
                                               vertex_paths[i], vertex_paths[next_i],
                                               max_distance, merged_snappable_ids);
            if (search_result.path_found && search_result.objective_distance > max_distance) {
                max_distance = search_result.objective_distance;
                max_total_length = search_result.total_length;
                longest_path = search_result.path_geometry;
                nearest_point_vertex_position_index_with_longest_path = search_result.nearest_point_vertex_position_index;
                edge_index_with_longest_path = search_result.edge_index;
            }
        }
        else if (!segment_within_hull) {
            // Simple arithmetic solution when segment doesn't interior intersect convex hull
            double x = (vertex_paths[next_i].objective_distance + segment_length - vertex_paths[i].objective_distance) / 2.0;
            double total_distance = vertex_paths[i].objective_distance + x;
            
            if (total_distance > max_distance) {
                // x is measured from i to the optimal point
                LineString path_geometry = vertex_paths[i].path_geometry;
                 
                // Use boost geometry line_interpolate with absolute distance
                LineString segment_line;
                segment_line.push_back(point_i);
                segment_line.push_back(point_next);
                Point optimal_point;
                 
                bg::line_interpolate(segment_line, x, optimal_point);
                
                // Add the optimal point to the path
                LineString new_path;
                new_path.push_back(optimal_point);
                new_path.insert(new_path.end(), path_geometry.begin(), path_geometry.end());
                
                max_distance = total_distance;
                max_total_length = vertex_paths[i].total_length + x;
                longest_path = new_path;
                nearest_point_vertex_position_index_with_longest_path = vertex_paths[i].nearest_point_vertex_position_index;
                edge_index_with_longest_path = vertex_paths[i].edge_index;
            }
        } else {
            // Use sectional search for segments that intersect with convex hull
            auto search_result = sectionalSearch(point_i, point_next, 
                                               vertex_paths[i], vertex_paths[next_i],
                                               max_distance, polygon_feature.snappable_ids);
            
            if (search_result.path_found && search_result.objective_distance > max_distance) {
                max_distance = search_result.objective_distance;
                max_total_length = search_result.total_length;
                longest_path = search_result.path_geometry;
                nearest_point_vertex_position_index_with_longest_path = search_result.nearest_point_vertex_position_index;
                edge_index_with_longest_path = search_result.edge_index;
            }
        }
    }
    
    // Clear the graph before returning output
    clear();
    
    // Extract the least accessible point from the path (first point in the geometry)
    Point least_accessible_point;
    if (!longest_path.empty()) {
        least_accessible_point = longest_path.front();
    } else {
        std::cerr << "WARNING: Empty path geometry for polygon " << polygon_feature.feature_id << std::endl;
        // Return empty result when no valid path found
        return std::make_tuple(std::vector<ConvexPathResult>(), polygon_feature.feature_id, Point());
    }
    
    ConvexPathResult result(longest_path, {}, max_total_length, max_distance, nearest_point_vertex_position_index_with_longest_path, edge_index_with_longest_path);
    result.start_point_type = ConvexPathResultType::LEAST_ACCESSIBLE_POINT;
    vertex_paths.push_back(result);

    // Loop through all vertex paths, if path found, convert nearest_point_vertex_position_index and edge_index to feature_id
    for (auto& path : vertex_paths) {
        if (path.path_found) {
            if (useRoadData_) {
                // Road data case: convert vertex/edge indices to feature IDs
                path.nearest_point_vertex_position_index = getVertex(path.nearest_point_vertex_position_index).feature_id;
                path.edge_index = getEdge(path.edge_index).feature_id;
            } else {
                // No road data case: nearest_point_vertex_position_index and edge_index are both set to snappable_id (vector index)
                // Convert to actual feature ID from access_points_
                size_t snappable_id = path.nearest_point_vertex_position_index;
                auto access_point_opt = getAccessPoint(snappable_id);
                if (access_point_opt.has_value()) {
                    path.nearest_point_vertex_position_index = access_point_opt.value().id;
                    path.edge_index = access_point_opt.value().id;  // Same as nearest_point for access points
                }
            }
        }
    }

    return std::make_tuple(vertex_paths, polygon_feature.feature_id, least_accessible_point);
}

ConvexPathResult ConvexPath::sectionalSearch(const Point& p1, const Point& p2,
                                            const ConvexPathResult& p1_path,
                                            const ConvexPathResult& p2_path,
                                            double max_distance,
                                            const std::vector<size_t>& snappable_ids
                                        ) {
    /*
    By default, we will use 7 intervals to split the segment, which means 8 points (two endpoints and 6 intermediate points).
    Imagine a coordinate with 1 demical place precision, the minimum noticeable length of the segment will be sqrt(0.1 * 0.1 + 0.1 * 0.1) which is approximately 0.1414 (round to 0.15 for simplicity).
    Interpolating along the segment with a too small interval distance will be sensitive to coordinate precision and its rounding error.
    When the segment length is too small (0.15 * 7 = 1.05), we shall just use a constant interval distance of 0.15.
    */
    const int NUM_INTERVALS = 7;  // 8 points = 7 intervals
    const double CRITICAL_SEGMENT_LENGTH = 1.05;
    
    Point left_bound = p1;
    Point right_bound = p2;
    auto left_path = p1_path;
    auto right_path = p2_path;
    double current_best = max_distance;
    
    // Initialize best result with better endpoint
    ConvexPathResult best_result;
    if (left_path.path_found && right_path.path_found) {
        best_result = left_path.objective_distance >= right_path.objective_distance ? left_path : right_path;
    } else if (left_path.path_found) {
        best_result = left_path;
    } else if (right_path.path_found) {
        best_result = right_path;
    } else {
        return ConvexPathResult(); // No valid paths found
    }
    
    int iteration = 0;
    bool is_terminated = false;
    while (!is_terminated) {
        iteration++;
        
        double segment_length = bg::distance(left_bound, right_bound);
        
        // Check potential improvement condition
        double min_endpoint_distance = std::min(
            left_path.path_found ? left_path.objective_distance : 0.0,
            right_path.path_found ? right_path.objective_distance : 0.0
        );
        if (segment_length + min_endpoint_distance <= current_best) {
            //std::cout << "  Terminating: insufficient potential improvement" << std::endl;
            is_terminated = true;
            break;
        }

        // If segment length is less than CRITICAL_SEGMENT_LENGTH, we will use a constant interval distance of 0.15; otherwise, we will use the default number of intervals
        double interval_distance;
        if (segment_length <= CRITICAL_SEGMENT_LENGTH) {
            interval_distance = 0.15;
            is_terminated = true;  // terminate after this iteration
        }
        else {
            interval_distance = std::floor((segment_length / NUM_INTERVALS) * 100) / 100;
        }
        
        // Create sample points using line_interpolate with MultiPoint output
        std::vector<Point> sample_points;
        std::vector<ConvexPathResult> sample_paths;
        
        // Add left endpoint
        sample_points.push_back(left_bound);
        
        // Generate intermediate points using line_interpolate
        LineString segment_line;
        segment_line.push_back(left_bound);
        segment_line.push_back(right_bound);
        
        MultiPoint interpolated_points;
        bg::line_interpolate(segment_line, interval_distance, interpolated_points);
        
        // Extract the first NUM_INTERVALS - 1 points (excluding both endpoints, but total points is NUM_INTERVALS + 1)
        size_t points_to_use = std::min(static_cast<size_t>(NUM_INTERVALS - 1), interpolated_points.size());
        for (size_t i = 0; i < points_to_use; ++i) {
            sample_points.push_back(interpolated_points[i]);
        }
        
        // Add right endpoint
        sample_points.push_back(right_bound);
        int sample_points_size = sample_points.size();
        
        // Now compute paths for all points
        sample_paths.push_back(left_path);
        
        for (size_t i = 1; i < sample_points_size - 1; ++i) {
            // Use the current ConvexPath instance directly
            auto path = findConvexPath(sample_points[i], snappable_ids);
            
            sample_paths.push_back(path);
        }
        
        sample_paths.push_back(right_path);
        
        // Find maximum distance
        double max_distance = 0.0;
        int max_index = -1;
        
        for (int i = 0; i <= sample_points_size - 1; ++i) { // NUM_INTERVALS will be sample_points_size - 1 if interpolation is correct
            if (sample_paths[i].path_found && sample_paths[i].objective_distance > max_distance) {
                max_distance = sample_paths[i].objective_distance;
                max_index = i;
            }
        }

        if (max_index == -1) {
            break;
        }

        // Update best result if we found a better one
        if (max_distance > best_result.objective_distance) {
            best_result.path_geometry = sample_paths[max_index].path_geometry;
            best_result.objective_distance = sample_paths[max_index].objective_distance;
            best_result.total_length = sample_paths[max_index].total_length;
            best_result.path_found = sample_paths[max_index].path_found;
            best_result.nearest_point_vertex_position_index = sample_paths[max_index].nearest_point_vertex_position_index;
            best_result.edge_index = sample_paths[max_index].edge_index;
            current_best = max_distance;
        }
        
        // Update bounds for next iteration
        if (max_index == 0) {
            if (sample_points.size() > 1) {
                right_bound = sample_points[1];
                right_path = sample_paths[1];
            } else {
                break;
            }
        } else if (max_index == (sample_points_size - 1)) {
            left_bound = sample_points[sample_points_size - 2];
            left_path = sample_paths[sample_points_size - 2];
        } else {
            left_bound = sample_points[max_index - 1];
            left_path = sample_paths[max_index - 1];
            right_bound = sample_points[max_index + 1];
            right_path = sample_paths[max_index + 1];
        }
    }
    
    return best_result;
}

} // namespace graph
} // namespace adjfind
