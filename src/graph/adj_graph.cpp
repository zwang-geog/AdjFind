#include "graph/adj_graph.hpp"
#include <stdexcept>
#include <iostream>
#include <algorithm>
#include <queue>

namespace adjfind {
namespace graph {

AdjGraph::AdjGraph()
    : next_vertex_id_(0), next_edge_id_(0) {
}

void AdjGraph::setCoordinateSystem(const std::string& wkt, int epsg) {
    coordinate_system_wkt_ = wkt;
    coordinate_system_epsg_ = epsg;
}

size_t AdjGraph::addVertex(const Point3D& geometry, VertexType type, size_t feature_id) {
    // For point vertices (snapped points), we want to create unique vertices
    // For intersection vertices, we use spatial tolerance
    if (type == VertexType::POINT || type == VertexType::SPLIT) {
        size_t vertex_id = next_vertex_id_++;
        vertices_.emplace_back(vertex_id, feature_id, geometry, type);
        vertex_rtree_.insert(VertexRTreeValue(geometry, vertex_id));
        return vertex_id;
    } else {
        // For intersection vertices, use spatial tolerance
        return findOrCreateVertex(geometry);
    }
}

size_t AdjGraph::findOrCreateVertex(const Point3D& point) {
    // Query R-tree for nearby vertices using distance-based tolerance
    std::vector<VertexRTreeValue> candidates;
    vertex_rtree_.query(
        bgi::nearest(point, 1) && 
        bgi::satisfies([&](const VertexRTreeValue& v) {
            return bg::distance(point, v.location) <= ROAD_VERTEX_SNAPPING_TOLERANCE;
        }),
        std::back_inserter(candidates)
    );
    
    if (!candidates.empty()) {
        return candidates[0].vertex_id;  // Return existing vertex ID
    }
    
    // Create new vertex
    size_t vertex_id = next_vertex_id_++;
    vertices_.emplace_back(vertex_id, vertex_id, point, VertexType::INTERSECTION); // feature_id same as vertex_id for intersection vertices
    vertex_rtree_.insert(VertexRTreeValue(point, vertex_id));
    return vertex_id;
}

std::optional<size_t> AdjGraph::findVertexForPoint(const Point3D& point) const {
    // Query R-tree for nearby vertices using distance-based tolerance
    std::vector<VertexRTreeValue> candidates;
    vertex_rtree_.query(
        bgi::nearest(point, 1) && 
        bgi::satisfies([&](const VertexRTreeValue& v) {
            return bg::distance(point, v.location) <= ROAD_VERTEX_SNAPPING_TOLERANCE;
        }),
        std::back_inserter(candidates)
    );
    
    if (!candidates.empty()) {
        return candidates[0].vertex_id;
    }
    
    return std::nullopt;
}

size_t AdjGraph::addEdge(size_t from_vertex, size_t to_vertex, double weight, size_t feature_id, const LineString& geometry) {
    // Skip self-loops
    if (from_vertex == to_vertex) {
        return std::numeric_limits<size_t>::max();
    }
    
    // Always create new edge (allow multiple edges between same vertices)
    size_t edge_id = next_edge_id_++;
    edges_.emplace_back(edge_id, feature_id, from_vertex, to_vertex, weight, geometry);
    
    // Add adjacency relationships with edge information
    adjacency_list_[from_vertex].emplace_back(to_vertex, edge_id, weight);
    adjacency_list_[to_vertex].emplace_back(from_vertex, edge_id, weight);
    
    return edge_id;
}

void AdjGraph::addRoadSegment(const LineString& geometry, double from_z, double to_z, double length, size_t feature_id) {
    // Skip empty or single-point linestrings
    if (geometry.size() < 2) {
        return;
    }
    
    const auto& start_point = geometry.front();
    const auto& end_point = geometry.back();
    
    // Skip zero-length segments
    if (bg::equals(start_point, end_point)) {
        return;
    }
    
    // Create 3D vertices with z-values from the road segment
    // For start point, use from_z; for end point, use to_z
    Point3D start_point_3d(bg::get<0>(start_point), bg::get<1>(start_point), from_z);
    Point3D end_point_3d(bg::get<0>(end_point), bg::get<1>(end_point), to_z);
    
    size_t from_vertex = findOrCreateVertex(start_point_3d);
    size_t to_vertex = findOrCreateVertex(end_point_3d);
    
    addEdge(from_vertex, to_vertex, length, feature_id, geometry);
    
}

const Vertex& AdjGraph::getVertex(size_t vertex_id) const {
    if (vertex_id >= vertices_.size()) {
        std::stringstream error_msg;
        error_msg << "Vertex ID out of range: vertex_id=" << vertex_id 
                 << ", vertices_.size()=" << vertices_.size();
        throw std::out_of_range(error_msg.str());
    }
    return vertices_[vertex_id];
}

const Edge& AdjGraph::getEdge(size_t edge_id) const {
    if (edge_id >= edges_.size()) {
        std::stringstream error_msg;
        error_msg << "Edge ID out of range: edge_id=" << edge_id 
                 << ", edges_.size()=" << edges_.size();
        throw std::out_of_range(error_msg.str());
    }
    return edges_[edge_id];
}



const std::vector<std::tuple<size_t, size_t, double>>& AdjGraph::getConnectedVertices(size_t vertex_id) const {
    auto it = adjacency_list_.find(vertex_id);
    if (it != adjacency_list_.end()) {
        return it->second;
    }
    static const std::vector<std::tuple<size_t, size_t, double>> empty_vector;
    return empty_vector;
}

std::vector<size_t> AdjGraph::getPointVertices() const {
    std::vector<size_t> point_vertices;
    for (const auto& vertex : vertices_) {
        if (vertex.type == VertexType::POINT) {
            point_vertices.push_back(vertex.vertex_index);
        }
    }
    return point_vertices;
}

std::vector<size_t> AdjGraph::getIntersectionVertices() const {
    std::vector<size_t> intersection_vertices;
    for (const auto& vertex : vertices_) {
        if (vertex.type == VertexType::INTERSECTION) {
            intersection_vertices.push_back(vertex.vertex_index);
        }
    }
    return intersection_vertices;
}

void AdjGraph::updateVertexNearestPointVertex(size_t vertex_id, size_t nearest_point_vertex_position_index, double distance_to_point_vertex) {
    if (vertex_id >= vertices_.size()) {
        throw std::out_of_range("Vertex ID out of range");
    }
    vertices_[vertex_id].nearest_point_vertex_position_index = nearest_point_vertex_position_index;
    vertices_[vertex_id].distance_to_point_vertex = distance_to_point_vertex;
}

bool AdjGraph::readAndSnapPointToRoad(const io::RoadReaderConfig& road_config, const io::PointReaderConfig& point_config) {
    // Step 1: Read road and point data
    std::cout << "Step 1: Reading road and point data..." << std::endl;
    
    io::RoadReader road_reader(road_config);
    if (!road_reader.read()) {
        std::cerr << "Failed to read road data" << std::endl;
        return false;
    }
    
    io::PointReader point_reader(point_config);
    if (!point_reader.read()) {
        std::cerr << "Failed to read point data" << std::endl;
        return false;
    }
    
    // Check coordinate system compatibility
    int road_epsg = road_reader.getCoordinateSystemEPSG();
    int point_epsg = point_reader.getCoordinateSystemEPSG();
    
    if (road_epsg != point_epsg) {
        std::cerr << "ERROR: Coordinate system mismatch!" << std::endl;
        std::cerr << "  Road dataset: EPSG:" << road_epsg << std::endl;
        std::cerr << "  Point dataset: EPSG:" << point_epsg << std::endl;
        std::cerr << "  Both datasets must have the same coordinate system." << std::endl;
        return false;
    }
    
    std::cout << "  Coordinate systems match: EPSG:" << road_epsg << std::endl;
    
    // Store coordinate system information from road reader
    coordinate_system_wkt_ = road_reader.getCoordinateSystemWKT();
    coordinate_system_epsg_ = road_reader.getCoordinateSystemEPSG();
    
    std::cout << "  Read " << road_reader.getFeatureCount() << " roads and " 
              << point_reader.getFeatureCount() << " points" << std::endl;
    
    // Step 2: Snap points to roads and create snapped points dictionary
    std::cout << "Step 2: Snapping points to roads..." << std::endl;
    
    std::unordered_map<size_t, std::vector<graph::SnappedPointInfo>> snapped_points_dict;
    
    for (size_t i = 0; i < point_reader.getFeatureCount(); ++i) {
        const auto& point_feature = point_reader.getPointFeature(i);
        
        auto nearest_result = road_reader.findNearestRoadSegment(point_feature.geometry);
        if (!nearest_result) {
            std::cout << "  Warning: No nearest road found for point " << i << std::endl;
            continue;
        }
        
        auto [road_index, segment_index, snapped_point, distance_to_first] = *nearest_result;
        
        graph::SnappedPointInfo snapped_info(point_feature.id, snapped_point, distance_to_first, segment_index);
        snapped_points_dict[road_index].push_back(snapped_info);
    }
    
    std::cout << "  Snapped " << point_reader.getFeatureCount() << " points to " 
              << snapped_points_dict.size() << " roads" << std::endl;
    
    // Clear road reader and point reader R-trees - no longer needed after snapping
    road_reader.clearSpatialIndex();
    point_reader.clearSpatialIndex();

    // Clear point reader data (points) - no longer needed after snapping
    point_reader.clearPoints();
    
    // Step 3: Split roads at snapped points
    std::cout << "Step 3: Splitting roads at snapped points..." << std::endl;
    
    // Process each road that has snapped points
    for (auto& [road_index, snapped_points] : snapped_points_dict) {
        // Get the original road feature directly using vector index
        auto& original_road = road_reader.getRoadFeature(road_index);
        
        // Soft delete the original road
        original_road.is_deleted = true;
        
        // Sort snapped points by segment_index first, then by distance_to_first_point
        std::sort(snapped_points.begin(), snapped_points.end(), 
                 [](const graph::SnappedPointInfo& a, const graph::SnappedPointInfo& b) {
                     return std::tie(a.segment_index, a.distance_to_first_point) < 
                            std::tie(b.segment_index, b.distance_to_first_point);
                 });
        
        // Create new linestrings by splitting between snapped points
        const auto& original_linestring = original_road.geometry;
        
        if (original_linestring.size() < 2) {
            std::cout << "Warning: Road index " << road_index << " has insufficient points for splitting" << std::endl;
            continue;
        }

        // Add the first and last point of the original linestring to graph vertices with type INTERSECTION
        auto linestring_first_vertex_id = addVertex(Point3D(bg::get<0>(original_linestring[0]), bg::get<1>(original_linestring[0]), original_road.from_z), graph::VertexType::INTERSECTION, 0);
        auto linestring_last_vertex_id = addVertex(Point3D(bg::get<0>(original_linestring[original_linestring.size() - 1]), bg::get<1>(original_linestring[original_linestring.size() - 1]), original_road.to_z), graph::VertexType::INTERSECTION, 0);
        
        // Loop through snapped_points.size() + 1 times
        for (size_t i = 0; i <= snapped_points.size(); ++i) {
            graph::LineString new_linestring;
            size_t snapped_point_vertex_id;
            size_t previous_vertex_id;
            
            if (i == 0) {
                // First iteration: from first point to first snapped point
                const auto& first_snapped = snapped_points[0];
                
                // Add points from start to segment_index
                for (size_t j = 0; j <= first_snapped.segment_index; ++j) {
                    new_linestring.push_back(original_linestring[j]);
                }
                // Add the snapped point itself
                new_linestring.push_back(first_snapped.snapped_geometry);
                previous_vertex_id = linestring_first_vertex_id;
                // Convert 2D snapped point to 3D
                Point3D snapped_point_3d(bg::get<0>(first_snapped.snapped_geometry), bg::get<1>(first_snapped.snapped_geometry), original_road.from_z);
                snapped_point_vertex_id = addVertex(snapped_point_3d, graph::VertexType::POINT, first_snapped.original_point_index);

            } else if (i == snapped_points.size()) {
                // Last iteration: from last snapped point to end
                const auto& last_snapped = snapped_points[snapped_points.size() - 1];
                
                // Add the snapped point itself
                new_linestring.push_back(last_snapped.snapped_geometry);
                // Add points from segment_index + 1 to end
                for (size_t j = last_snapped.segment_index + 1; j < original_linestring.size(); ++j) {
                    new_linestring.push_back(original_linestring[j]);
                }
                
            } else {
                // Middle iterations: between consecutive snapped points
                const auto& prev_snapped = snapped_points[i - 1];
                const auto& curr_snapped = snapped_points[i];
                
                // Add the previous snapped point
                new_linestring.push_back(prev_snapped.snapped_geometry);
                // Add points from prev_segment_index + 1 to curr_segment_index
                for (size_t j = prev_snapped.segment_index + 1; j <= curr_snapped.segment_index; ++j) {
                    new_linestring.push_back(original_linestring[j]);
                }
                // Add the current snapped point
                new_linestring.push_back(curr_snapped.snapped_geometry);
                previous_vertex_id = snapped_point_vertex_id;
                // Convert 2D snapped point to 3D
                Point3D snapped_point_3d(bg::get<0>(curr_snapped.snapped_geometry), bg::get<1>(curr_snapped.snapped_geometry), original_road.to_z);
                snapped_point_vertex_id = addVertex(snapped_point_3d, graph::VertexType::POINT, curr_snapped.original_point_index);
                
            }
            
            // Calculate length and add to graph if valid
            if (new_linestring.size() >= 2) {
                double new_length = bg::length(new_linestring);
                
                // Add edges to the graph
                if (i == 0) {
                    // First segment: from linestring_first_vertex_id to snapped_point_vertex_id
                    addEdge(linestring_first_vertex_id, snapped_point_vertex_id, new_length, original_road.id, new_linestring);
                } else if (i == snapped_points.size()) {
                    // Last segment: from last snapped point to linestring_last_vertex_id
                    addEdge(snapped_point_vertex_id, linestring_last_vertex_id, new_length, original_road.id, new_linestring);
                } else { 
                    // Middle segments: from previous snapped point to current snapped point
                    addEdge(previous_vertex_id, snapped_point_vertex_id, new_length, original_road.id, new_linestring);
                }
            }
        }
    }
    
    // Memory cleanup after Step 3: Clear snapped points dictionary since we've processed all roads
    snapped_points_dict.clear();
    
    // Step 4: Add non-deleted roads directly to the graph
    std::cout << "Step 4: Adding non-deleted roads to graph..." << std::endl;
    
    for (size_t i = 0; i < road_reader.getFeatureCount(); ++i) {
        const auto& road_feature = road_reader.getRoadFeature(i);
        if (!road_feature.is_deleted) {
            const auto& linestring = road_feature.geometry;
            
            if (linestring.size() < 2) {
                continue;
            }
            
            // Create one edge for the entire linestring
            addRoadSegment(linestring, road_feature.from_z, road_feature.to_z, road_feature.length, road_feature.id);
        }
    }
    
    // Memory cleanup after Step 4: Clear road data since we've processed all roads
    road_reader.clearRoads();
    
    return true;
}

void AdjGraph::buildEdgeRTree() {
    // Clear existing R-tree
    edge_rtree_.clear();
    
    // Iterate through all edges
    for (const auto& edge : edges_) {
        // Check if edge's endpoint_vertex_to_nearest_point is max (disconnected edge)
        if (edge.endpoint_vertex_to_nearest_point == std::numeric_limits<size_t>::max()) {
            std::cout << "WARNING: Skipping edge " << edge.feature_id << " because endpoint_vertex_to_nearest_point is not set" << std::endl;
            continue;
        }
        const auto& geometry = edge.geometry;
        
        // Skip edges with insufficient points
        if (geometry.size() < 2) {
            continue;
        }
        
        // Create segments from the linestring
        for (size_t i = 0; i < geometry.size() - 1; ++i) {
            Segment segment(geometry[i], geometry[i + 1]);
            edge_rtree_.insert(EdgeRTreeValue(segment, edge.edge_index));
        }
    }
}

std::optional<size_t> AdjGraph::findNearestEdge(const Point& query_point) const {
    if (edge_rtree_.empty()) {
        return std::nullopt;
    }
    
    // Query for the nearest segment
    std::vector<EdgeRTreeValue> nearest_results;
    edge_rtree_.query(
        bgi::nearest(query_point, 1),
        std::back_inserter(nearest_results)
    );
    
    if (nearest_results.empty()) {
        return std::nullopt;
    }
    
    return nearest_results[0].edge_index;
}

void AdjGraph::clearEdgeRTree() {
    edge_rtree_.clear();
}

void AdjGraph::buildFeatureIdMapping() {
    // Clear existing mapping
    feature_id_to_edge_indices_.clear();
    
    // Loop through all edges and populate the mapping
    for (const auto& edge : edges_) {
        feature_id_to_edge_indices_[edge.feature_id].push_back(edge.edge_index);
    }
}

std::vector<size_t> AdjGraph::getEdgeIndicesForFeatureId(size_t feature_id) const {
    auto it = feature_id_to_edge_indices_.find(feature_id);
    if (it != feature_id_to_edge_indices_.end()) {
        return it->second;
    }
    return std::vector<size_t>(); // Return empty vector if feature_id not found
}

void AdjGraph::clearFeatureIdMapping() {
    feature_id_to_edge_indices_.clear();
}

std::tuple<Point, double, double, size_t, size_t, bool> AdjGraph::computeSnappedPointOnEdge(size_t snappable_road_edge_index, 
                                                                                             const Point& point) const {
    // Default return values
    Point projected_point;  // Empty point as default
    double distance_to_nearest_point = 0.0;  // snapped point to nearest point vertex
    double dist_to_closest_edge = std::numeric_limits<double>::max(); // unsnapped point to snapped point
    size_t nearest_point_vertex_position_index = SIZE_MAX;  // index of nearest point vertex on the edge
    size_t edge_index = SIZE_MAX;
    bool is_outside_segment = false;  // indicates if t is outside [0,1] range
    
    // Validate edge index
    if (snappable_road_edge_index >= edges_.size()) {
        return std::make_tuple(projected_point, distance_to_nearest_point, dist_to_closest_edge, nearest_point_vertex_position_index, edge_index, is_outside_segment);
    }
    
    const auto& edge = edges_[snappable_road_edge_index];
    
    // Get the linestring geometry for this edge
    const auto& edge_geometry = edge.geometry;
    
    // Initialize variables for finding closest segment
    size_t segment_idx = SIZE_MAX;
    edge_index = snappable_road_edge_index;
    
    // Loop through each segment constituting the linestring
    for (size_t i = 0; i < edge_geometry.size() - 1; ++i) {
        Point segment_start = edge_geometry[i];
        Point segment_end = edge_geometry[i + 1];
        
        // Create segment for distance calculation
        bg::model::segment<Point> segment(segment_start, segment_end);
        
        // Calculate distance from point to this segment
        double dist_to_segment = bg::distance(point, segment);
        
        // Update closest segment if this segment is closer
        if (dist_to_segment < dist_to_closest_edge) {
            dist_to_closest_edge = dist_to_segment;
            segment_idx = i;
        }
    }

    // With the closest edge index, compute the projected point on the edge
    if (segment_idx != SIZE_MAX) {
        // Calculate the closest point on the segment using projection
        const Point& segment_start = edge_geometry[segment_idx];
        const Point& segment_end = edge_geometry[segment_idx + 1];
        
        // Vector from segment_start to segment_end
        double abx = bg::get<0>(segment_end) - bg::get<0>(segment_start);
        double aby = bg::get<1>(segment_end) - bg::get<1>(segment_start);
        
        // Vector from segment_start to point
        double apx = bg::get<0>(point) - bg::get<0>(segment_start);
        double apy = bg::get<1>(point) - bg::get<1>(segment_start);
        
        // Length of segment squared
        double ab_squared = abx * abx + aby * aby;
        
        // If the line segment has zero length, return one endpoint
        if (ab_squared == 0) {
            projected_point = segment_start;
        } else {
            // Calculate projection of ap onto ab, normalized by length of ab
            double t = (apx * abx + apy * aby) / ab_squared;
            
            // Check if t is outside the [0,1] range before constraining
            is_outside_segment = (t < 0.0 || t > 1.0);
            
            // Constrain t to lie within the line segment
            t = std::max(0.0, std::min(1.0, t));
            
            // Calculate closest point
            bg::set<0>(projected_point, bg::get<0>(segment_start) + t * abx);
            bg::set<1>(projected_point, bg::get<1>(segment_start) + t * aby);
        }

        // Loop through vertices of the edge until segment_idx
        Point prev_vertex = edge_geometry[0];
        for (size_t i = 1; i <= segment_idx; ++i) {
            Point current_vertex = edge_geometry[i];
            double dist_to_current_vertex = bg::distance(prev_vertex, current_vertex);
            distance_to_nearest_point += dist_to_current_vertex;
            prev_vertex = current_vertex;
        }

        distance_to_nearest_point += bg::distance(projected_point, prev_vertex);

        if (!edge.endpoint_vertex_to_nearest_point_is_from_vertex) {
            distance_to_nearest_point = edge.weight - distance_to_nearest_point + getVertex(edge.endpoint_vertex_to_nearest_point).distance_to_point_vertex;
        }
        else {
            distance_to_nearest_point += getVertex(edge.endpoint_vertex_to_nearest_point).distance_to_point_vertex;
        }

        nearest_point_vertex_position_index = edge.nearest_point_vertex_position_index;
    }
    
    return std::make_tuple(projected_point, distance_to_nearest_point, dist_to_closest_edge, nearest_point_vertex_position_index, edge_index, is_outside_segment);
}

} // namespace graph
} // namespace adjfind 