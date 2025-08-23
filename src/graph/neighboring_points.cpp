#include "graph/neighboring_points.hpp"
#include <algorithm>
#include <limits>
#include <unordered_map>

namespace adjfind {
namespace graph {

std::tuple<size_t, double, MultiLineString> NeighboringPoints::findPathToClosestPointVertex(
    size_t source,
    std::optional<double> cutoff,
    const std::vector<size_t>& edges_to_exclude) const {

    const Vertex& source_vertex = getVertex(source);

    // Initialize data structures
    std::vector<double> distances(vertices_.size(), std::numeric_limits<double>::max());
    std::vector<size_t> predecessors_vertex(vertices_.size(), std::numeric_limits<size_t>::max());
    std::vector<size_t> predecessors_edge(vertices_.size(), std::numeric_limits<size_t>::max());
    std::vector<bool> finalized(vertices_.size(), false);

    // Priority queue for Dijkstra (distance, vertex_id)
    std::priority_queue<std::pair<double, size_t>, std::vector<std::pair<double, size_t>>, std::greater<std::pair<double, size_t>>> pq;

    // Initialize source
    distances[source] = 0.0;
    pq.push({0.0, source});

    // Process vertices
    while (!pq.empty()) {
        auto [current_distance, current_vertex] = pq.top();
        pq.pop();

        // Skip if we've already finalized this vertex or exceeded cutoff
        if (finalized[current_vertex] || (cutoff && current_distance > *cutoff)) {
            continue;
        }

        // Mark as finalized
        finalized[current_vertex] = true;

        const Vertex& cur_vertex = getVertex(current_vertex);
        // Check if the current node is a point vertex; if it is, we add it to the results
        if (current_vertex != source && cur_vertex.type == VertexType::POINT) {
            // Add the current node to the results
            auto path = buildPath(current_vertex, source, predecessors_vertex, predecessors_edge);
            return std::make_tuple(current_vertex, current_distance, path);  // currently vertex internal index instead of feature_id
        }

        // Process outgoing edges
        for (const auto& [neighbor_vertex, edge_id, edge_weight] : getConnectedVertices(current_vertex)) {
            // Skip this edge if it's in the excluded edges list
            if (std::find(edges_to_exclude.begin(), edges_to_exclude.end(), edge_id) != edges_to_exclude.end()) {
                continue;
            }
            
            if (finalized[neighbor_vertex]) {
                continue;
            }
                    
            // Calculate new distance
            double new_dist = current_distance + edge_weight;
            
            // Skip if exceeds cutoff
            if (cutoff && new_dist > *cutoff) {
                continue;
            }

            // Update if we found a shorter path
            if (new_dist < distances[neighbor_vertex]) {
                distances[neighbor_vertex] = new_dist;
                predecessors_vertex[neighbor_vertex] = current_vertex;
                predecessors_edge[neighbor_vertex] = edge_id;
                pq.push({new_dist, neighbor_vertex});
            }
        }
    }

    return std::make_tuple(std::numeric_limits<size_t>::max(), std::numeric_limits<double>::max(), MultiLineString());
}

MultiLineString NeighboringPoints::buildPath(
    size_t end,
    size_t source,
    const std::vector<size_t>& predecessors_vertex,
    const std::vector<size_t>& predecessors_edge) const {
    
    // Collect all edge geometries in reverse order (from end to source)
    std::vector<LineString> edge_geometries;
    
    size_t current = end;
    while (current != source) {
        const Vertex& current_vertex = getVertex(current);
        
        size_t current_edge_index = predecessors_edge[current];
        const Edge& current_edge = getEdge(current_edge_index);
        
        // Add the edge geometry to our collection
        edge_geometries.push_back(current_edge.geometry);
        
        current = predecessors_vertex[current];
    }
    
    // Reverse the order so we go from source to end
    std::reverse(edge_geometries.begin(), edge_geometries.end());
    
    // If we have no edges, return empty multilinestring
    if (edge_geometries.empty()) {
        return MultiLineString();
    }
    
    // Start with the first edge geometry
    MultiLineString path_geometry;
    path_geometry.push_back(edge_geometries[0]);
    
    // Union all remaining edge geometries to create a continuous path
    for (size_t i = 1; i < edge_geometries.size(); ++i) {
        MultiLineString union_result;
        bg::union_(path_geometry, edge_geometries[i], union_result);
        
        // Keep all elements from the union result
        // This handles cases where edges might not connect perfectly
        path_geometry = union_result;
    }
    
    return path_geometry;
}

std::pair<size_t, std::vector<std::tuple<size_t, double, MultiLineString>>> NeighboringPoints::findNeighboringPointsForAGivenVertex(
    size_t source_vertex_index,
    double intersection_vertex_distance_threshold, 
    std::optional<double> cutoff) const {

    std::vector<std::tuple<size_t, double, MultiLineString>> results;

    const std::vector<std::tuple<size_t, size_t, double>>& source_vertex_adjacency = getConnectedVertices(source_vertex_index);

    for (const auto& [neighbor_vertex_index, neighbor_edge_index, edge_weight] : source_vertex_adjacency) {
        bool treat_as_intersection_vertex = false;

        const Vertex& neighbor_vertex = getVertex(neighbor_vertex_index);

        // Condition 1: Is the connected vertex a road intersection vertex?
        if (neighbor_vertex.type == VertexType::INTERSECTION) {
            // Condition 2: Is the source vertex very close to the connected intersection vertex?
            if (edge_weight <= intersection_vertex_distance_threshold) {
                // Condition 3: Is the source vertex the only point vertex that is very close to the connected intersection vertex?
                bool neighbor_vertex_has_two_nearby_point_vertices = false;
                for (const auto& [neighbor2_vertex_index, neighbor2_edge_index, edge2_weight] : getConnectedVertices(neighbor_vertex_index)) {
                    if (neighbor2_vertex_index != source_vertex_index && edge2_weight <= intersection_vertex_distance_threshold) {
                        const Vertex& neighbor2_vertex = getVertex(neighbor2_vertex_index);
                        if (neighbor2_vertex.type == VertexType::POINT) {
                            neighbor_vertex_has_two_nearby_point_vertices = true;
                            break;
                        }
                    }
                }

                if (!neighbor_vertex_has_two_nearby_point_vertices) {
                    treat_as_intersection_vertex = true;
                }
            }
        }

        if (!treat_as_intersection_vertex) {
            std::vector<size_t> edges_to_exclude;
            for (const auto& [neighbor_vertex_index_2, neighbor_edge_index_2, edge_weight_2] : source_vertex_adjacency) {
                if (neighbor_edge_index_2 != neighbor_edge_index) {
                    edges_to_exclude.push_back(neighbor_edge_index_2);
                }
            }
            auto path_result = findPathToClosestPointVertex(source_vertex_index, cutoff, edges_to_exclude);
            if (std::get<0>(path_result) != std::numeric_limits<size_t>::max()) {
                // Check if we already have an entry with the same feature_id
                bool found_existing = false;
                for (auto& existing_result : results) {
                    if (std::get<0>(existing_result) == std::get<0>(path_result)) {
                        // Update existing entry if new path is shorter
                        if (std::get<1>(path_result) < std::get<1>(existing_result)) {
                            existing_result = path_result;
                        }
                        found_existing = true;
                        break;
                    }
                }
                // Add new entry if no existing entry was found
                if (!found_existing) {
                    results.push_back(path_result);
                }
            }
        }
        else {
            const std::vector<std::tuple<size_t, size_t, double>>& neighbor_vertex_adjacency = getConnectedVertices(neighbor_vertex_index);
            for (size_t i = 0; i < neighbor_vertex_adjacency.size(); ++i) {
                size_t vertex_index_1 = std::get<0>(neighbor_vertex_adjacency[i]);
                if (vertex_index_1 != source_vertex_index) {
                    std::vector<size_t> edges_to_exclude;
                    for (const auto& [neighbor_vertex_index_2, neighbor_edge_index_2, edge_weight_2] : source_vertex_adjacency) {
                        if (neighbor_edge_index_2 != neighbor_edge_index) {
                            edges_to_exclude.push_back(neighbor_edge_index_2);
                        }
                    }

                    for (size_t j = 0; j < neighbor_vertex_adjacency.size(); ++j) {
                        size_t vertex_index_2 = std::get<0>(neighbor_vertex_adjacency[j]);
                        if (i != j && vertex_index_2 != source_vertex_index) {
                            size_t edge_index_2 = std::get<1>(neighbor_vertex_adjacency[j]);
                            edges_to_exclude.push_back(edge_index_2);
                        }
                    }

                    auto path_result = findPathToClosestPointVertex(source_vertex_index, cutoff, edges_to_exclude);
                    if (std::get<0>(path_result) != std::numeric_limits<size_t>::max()) {
                        // Check if we already have an entry with the same feature_id
                        bool found_existing = false;
                        for (auto& existing_result : results) {
                            if (std::get<0>(existing_result) == std::get<0>(path_result)) {
                                // Update existing entry if new path is shorter
                                if (std::get<1>(path_result) < std::get<1>(existing_result)) {
                                    existing_result = path_result;
                                }
                                found_existing = true;
                                break;
                            }
                        }
                        // Add new entry if no existing entry was found
                        if (!found_existing) {
                            results.push_back(path_result);
                        }
                    }
                }
            }
        }
    }

    return std::make_pair(source_vertex_index, results);  // currently vertex internal index instead of feature_id
}

bool NeighboringPoints::processNeighboringPoints(const io::RoadReaderConfig& road_config, 
                                                const io::PointReaderConfig& point_config,
                                                double intersection_vertex_distance_threshold,
                                                std::optional<double> cutoff) {
    std::cout << "=== Neighboring Points Mode ===" << std::endl;
    
    // Read and snap points to roads (Steps 1-4) - call base class method
    if (!AdjGraph::readAndSnapPointToRoad(road_config, point_config)) {
        std::cerr << "ERROR: Failed to read and snap points to roads. Aborting neighboring points processing." << std::endl;
        return false;
    }
    
    // Get the total number of point vertices to reserve the results vector
    std::vector<size_t> point_vertices = getPointVertices();
    size_t total_points = point_vertices.size();
    
    std::cout << "Found " << total_points << " point vertices. Processing neighboring points..." << std::endl;
    
    // Loop through point vertices directly (no need to check type since getPointVertices() already filters)
    for (size_t vertex_index : point_vertices) {
        auto result = findNeighboringPointsForAGivenVertex(vertex_index, 
                                                          intersection_vertex_distance_threshold, 
                                                          cutoff);
        
        // Process the result directly for deduplication
        size_t source_vertex_index = result.first;
        
        for (const auto& path_result : result.second) {
            size_t target_vertex_index = std::get<0>(path_result);
            double distance = std::get<1>(path_result);
            
            // Create normalized key with smaller vertex ID first
            auto key = (source_vertex_index <= target_vertex_index) ?
                std::make_pair(source_vertex_index, target_vertex_index) :
                std::make_pair(target_vertex_index, source_vertex_index);
            
            auto it = neighboring_points_results_.find(key);
            
            if (it == neighboring_points_results_.end() || distance < std::get<1>(it->second)) {
                neighboring_points_results_[key] = path_result;
            }
        }
    }
    
    std::cout << "Completed processing " << neighboring_points_results_.size() << " unique vertex pairs." << std::endl;
    return true;
}

} // namespace graph
} // namespace adjfind


