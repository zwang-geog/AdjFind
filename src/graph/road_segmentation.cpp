#include "graph/road_segmentation.hpp"
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <queue>
#include <unordered_map>
#include <sstream>
#include <stdexcept>

namespace adjfind {
namespace graph {

RoadSegmentation::RoadSegmentation() : AdjGraph() {}

std::vector<double> RoadSegmentation::parseDistanceVector(const std::string& distance_str) {
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

std::vector<RoadSplitByDistanceBracketsOutput> RoadSegmentation::processRoadSegmentationMode(const io::RoadReaderConfig& road_config, const io::PointReaderConfig& point_config, const std::string& distance_str) {
    std::cout << "=== Road Segmentation Mode ===" << std::endl;
    
    // Read and snap points to roads (Steps 1-4) - call base class method
    if (!AdjGraph::readAndSnapPointToRoad(road_config, point_config)) {
        std::cerr << "ERROR: Failed to read and snap points to roads. Aborting road segmentation." << std::endl;
        return std::vector<RoadSplitByDistanceBracketsOutput>();
    }
    
    // Step 5: Populate distances to point vertices
    std::cout << "Step 5: Populating distances to point vertices..." << std::endl;
    populateDistanceToPointVertex();
    
    // Step 6: Split linestrings at equilibrium points
    std::cout << "Step 6: Splitting linestrings at equilibrium points..." << std::endl;
    splitLinestringAtEquilibrium();
    
    // Step 7: Parse distance string and process based on distance breakpoints
    std::vector<double> distance_breakpoints = parseDistanceVector(distance_str);
    
    if (!distance_breakpoints.empty()) {
        std::cout << "Step 7: Splitting road linestrings by distance breakpoints..." << std::endl;
        return splitRoadLinestringsByDistanceVector(distance_breakpoints);
    } else {
        std::cout << "Step 7: Extracting edge information without distance splitting..." << std::endl;
        return extractEdgeInformation();
    }
}

void RoadSegmentation::populateDistanceToPointVertex() {
    // For each intersection vertex, find the nearest point vertex
    for (auto& vertex : vertices_) {
        if (vertex.type == VertexType::INTERSECTION) {
            // Skip if already populated
            if (vertex.nearest_point_vertex_position_index != std::numeric_limits<size_t>::max()) {
                continue;
            }
            
            // findNearestPointVertex already updates the vertex internally
            findNearestPointVertex(vertex.vertex_index);
        }
    }
}

std::pair<double, size_t> RoadSegmentation::findNearestPointVertex(size_t start_vertex) {
    // Use vectors for better performance
    std::vector<double> distances(vertices_.size(), std::numeric_limits<double>::max());
    std::vector<size_t> predecessors(vertices_.size(), std::numeric_limits<size_t>::max());
    std::vector<bool> finalized(vertices_.size(), false);
    
    // Priority queue for Dijkstra (distance, vertex_id)
    std::priority_queue<std::pair<double, size_t>, std::vector<std::pair<double, size_t>>, std::greater<std::pair<double, size_t>>> pq;
    
    // Initialize source
    distances[start_vertex] = 0.0;
    pq.push({0.0, start_vertex});
    
    size_t nearest_point_vertex_id = std::numeric_limits<size_t>::max();
    double nearest_distance = std::numeric_limits<double>::max();
    
    while (!pq.empty()) {
        auto [current_distance, current_vertex] = pq.top();
        pq.pop();
        
        // Skip if already finalized
        if (finalized[current_vertex]) {
            continue;
        }
        
        finalized[current_vertex] = true;
        
        // Check if this is a point vertex
        const Vertex& vertex = getVertex(current_vertex);
        if (vertex.type == VertexType::POINT) {
            nearest_point_vertex_id = current_vertex;
            nearest_distance = current_distance;
            break;
        }
        
        // Explore adjacent vertices
        for (const auto& [neighbor_vertex, edge_id, edge_weight] : getConnectedVertices(current_vertex)) {
            // Skip if already finalized
            if (finalized[neighbor_vertex]) {
                continue;
            }
            
            double new_distance = current_distance + edge_weight;
            
            if (new_distance < distances[neighbor_vertex]) {
                distances[neighbor_vertex] = new_distance;
                predecessors[neighbor_vertex] = current_vertex;
                pq.push({new_distance, neighbor_vertex});
            }
        }
    }
    
    // If we found a nearest point vertex, backtrace to set intermediate distances
    if (nearest_point_vertex_id != std::numeric_limits<size_t>::max()) {
        // Backtrace and update vertices directly (no path construction needed)
        size_t current_vertex = nearest_point_vertex_id;
        
        while (current_vertex != start_vertex) {
            const Vertex& vertex = getVertex(current_vertex);
            
            // The intermediate vertex was not set before
            if (vertex.nearest_point_vertex_position_index == std::numeric_limits<size_t>::max()) {
                // distance_to_point_vertex should be nearest_distance - distances[current_vertex]
                double distance_to_point_vertex = nearest_distance - distances[current_vertex];
                updateVertexNearestPointVertex(current_vertex, nearest_point_vertex_id, distance_to_point_vertex);
            }
            
            if (predecessors[current_vertex] == std::numeric_limits<size_t>::max()) {
                break; // No path found
            }
            current_vertex = predecessors[current_vertex];
        }
        
        // Update the start vertex if not already set
        const Vertex& start_vertex_obj = getVertex(start_vertex);
        if (start_vertex_obj.nearest_point_vertex_position_index == std::numeric_limits<size_t>::max()) {
            double distance_to_point_vertex = nearest_distance - distances[start_vertex];
            updateVertexNearestPointVertex(start_vertex, nearest_point_vertex_id, distance_to_point_vertex);
        }
    }
    
    return {nearest_distance, nearest_point_vertex_id};
}

void RoadSegmentation::splitLinestringAtEquilibrium() {
    // Get the original number of edges before we start adding new ones
    size_t original_edge_count = edges_.size();
    
    for (size_t i = 0; i < original_edge_count; ++i) {
        // Deep copy the Edge element to avoid issues when modifying the vector
        Edge edge = edges_[i];
        // Make deep copies to avoid reference invalidation when vertices_ vector is modified
        Vertex from_vertex = getVertex(edge.from_vertex);
        Vertex to_vertex = getVertex(edge.to_vertex);
        
        // Step 1: Check if vertices have the same nearest point vertex or any is unpopulated
        if (from_vertex.nearest_point_vertex_position_index == std::numeric_limits<size_t>::max() &&
        to_vertex.nearest_point_vertex_position_index == std::numeric_limits<size_t>::max()) {
            std::cout << "WARNING: Skipping edge " << edge.feature_id << " because both vertices have no nearest point vertex" << std::endl;
            continue;
        }
        else if (from_vertex.nearest_point_vertex_position_index == to_vertex.nearest_point_vertex_position_index && 
                 std::abs(std::abs(to_vertex.distance_to_point_vertex - from_vertex.distance_to_point_vertex) - edge.weight) < 1) {
            // Case 1: Both vertices have the same nearest point vertex AND the distance difference constraint (split point is near linestring end point) is satisfied
            // Compare distances and use the vertex with smaller distance
            if (from_vertex.distance_to_point_vertex <= to_vertex.distance_to_point_vertex) {
                edges_[i].endpoint_vertex_to_nearest_point = from_vertex.vertex_index;
                edges_[i].nearest_point_vertex_position_index = from_vertex.nearest_point_vertex_position_index;
                edges_[i].endpoint_vertex_to_nearest_point_is_from_vertex = true;
            } else {
                edges_[i].endpoint_vertex_to_nearest_point = to_vertex.vertex_index;
                edges_[i].nearest_point_vertex_position_index = to_vertex.nearest_point_vertex_position_index;
                edges_[i].endpoint_vertex_to_nearest_point_is_from_vertex = false;
            }
            continue; // Skip to next iteration
        }
        else if (from_vertex.nearest_point_vertex_position_index == std::numeric_limits<size_t>::max()) {
            // Case 2: Only from_vertex is unpopulated, use to_vertex
            edges_[i].endpoint_vertex_to_nearest_point = to_vertex.vertex_index;
            edges_[i].nearest_point_vertex_position_index = to_vertex.nearest_point_vertex_position_index;
            edges_[i].endpoint_vertex_to_nearest_point_is_from_vertex = false;
            continue; // Skip to next iteration
        }
        else if (to_vertex.nearest_point_vertex_position_index == std::numeric_limits<size_t>::max()) {
            // Case 3: Only to_vertex is unpopulated, use from_vertex
            edges_[i].endpoint_vertex_to_nearest_point = from_vertex.vertex_index;
            edges_[i].nearest_point_vertex_position_index = from_vertex.nearest_point_vertex_position_index;
            edges_[i].endpoint_vertex_to_nearest_point_is_from_vertex = true;
            continue; // Skip to next iteration
        }
        
        // Step 2: Calculate distance from first vertex
        double distance_from_first_vertex = 0.5 * (to_vertex.distance_to_point_vertex - from_vertex.distance_to_point_vertex + edge.weight);
        
        // Step 3: Check if equilibrium point is within valid range
        if (distance_from_first_vertex <= 0) {
            edges_[i].endpoint_vertex_to_nearest_point = from_vertex.vertex_index;
            edges_[i].nearest_point_vertex_position_index = from_vertex.nearest_point_vertex_position_index;
            edges_[i].endpoint_vertex_to_nearest_point_is_from_vertex = true;
            continue; // Skip to next iteration
        }
        else if (distance_from_first_vertex >= edge.weight) {
            edges_[i].endpoint_vertex_to_nearest_point = to_vertex.vertex_index;
            edges_[i].nearest_point_vertex_position_index = to_vertex.nearest_point_vertex_position_index;
            edges_[i].endpoint_vertex_to_nearest_point_is_from_vertex = false;
            continue; // Skip to next iteration
        }
        
        // Step 4: Loop through current edge linestring geometry to find equilibrium point
        const auto& linestring = edge.geometry;
        double cumulated_length = 0.0;
        graph::LineString new_linestring;
        size_t split_point_index = 0;
        Point interpolated_point;
        
        for (size_t i = 1; i < linestring.size(); ++i) {
            const auto& point1 = linestring[i - 1];
            const auto& point2 = linestring[i];
            
            double segment_length = bg::distance(point1, point2);
            
            if (cumulated_length + segment_length >= distance_from_first_vertex) {
                // Step 5(1): Use bg::line_interpolate to find equilibrium point
                graph::LineString segment_linestring;
                segment_linestring.push_back(point1);
                segment_linestring.push_back(point2);
                
                double interpolation_distance = distance_from_first_vertex - cumulated_length;
                
                // Use boost geometry line_interpolate with absolute distance
                bg::line_interpolate(segment_linestring, interpolation_distance, interpolated_point);
                
                new_linestring.push_back(point1);
                new_linestring.push_back(interpolated_point);
                split_point_index = i;
                cumulated_length += interpolation_distance;
                break;
            } else {
                // Step 5(2): Add current point and accumulate length
                new_linestring.push_back(point1);
                cumulated_length += segment_length;
            }
        }

        if (split_point_index == 0) {
            std::cout << "WARNING: Skipping edge " << edge.feature_id << " because no equilibrium point found" << std::endl;
            continue;
        }
        
        // Step 5: Add new vertex with interpolated point
        Point3D interpolated_point_3d(bg::get<0>(interpolated_point), bg::get<1>(interpolated_point), bg::get<2>(to_vertex.geometry));
        
        size_t new_vertex_id = addVertex(interpolated_point_3d, VertexType::SPLIT, 0);
        
        // Update the new vertex with calculated properties
        updateVertexNearestPointVertex(new_vertex_id, from_vertex.nearest_point_vertex_position_index, (distance_from_first_vertex + from_vertex.distance_to_point_vertex));
        
        // Step 6: Update current edge (first linestring) with new linestring and length
        edges_[edge.edge_index].geometry = new_linestring;
        edges_[edge.edge_index].weight = cumulated_length;
        edges_[edge.edge_index].to_vertex = new_vertex_id;
        edges_[edge.edge_index].endpoint_vertex_to_nearest_point = from_vertex.vertex_index;
        edges_[edge.edge_index].nearest_point_vertex_position_index = from_vertex.nearest_point_vertex_position_index;
        edges_[edge.edge_index].endpoint_vertex_to_nearest_point_is_from_vertex = true;
        
        // Update adjacency list for the modified edge
        // Remove old adjacency entries
        auto& from_adjacencies = adjacency_list_[edge.from_vertex];
        auto& to_adjacencies = adjacency_list_[edge.to_vertex];
        
        // Remove old entries
        from_adjacencies.erase(
            std::remove_if(from_adjacencies.begin(), from_adjacencies.end(),
                [edge](const auto& adj) { return std::get<1>(adj) == edge.edge_index; }),
            from_adjacencies.end()
        );
        to_adjacencies.erase(
            std::remove_if(to_adjacencies.begin(), to_adjacencies.end(),
                [edge](const auto& adj) { return std::get<1>(adj) == edge.edge_index; }),
            to_adjacencies.end()
        );
        
        // Add new adjacency entries
        from_adjacencies.emplace_back(new_vertex_id, edge.edge_index, cumulated_length);
        adjacency_list_[new_vertex_id].emplace_back(edge.from_vertex, edge.edge_index, cumulated_length);
        
        // Step 7: Create second linestring
        graph::LineString second_linestring;
        second_linestring.push_back(interpolated_point);
        
        // Add remaining points from original linestring
        for (size_t i = split_point_index; i < linestring.size(); ++i) {
            second_linestring.push_back(linestring[i]);
        }
        
        double second_length = bg::length(second_linestring);
        
        // Step 8: Add new edge with the second linestring
        size_t new_edge_id = next_edge_id_++;
        edges_.emplace_back(new_edge_id, edge.feature_id, new_vertex_id, edge.to_vertex, second_length, second_linestring);
        
        // Set the new edge's attributes for the second linestring
        edges_.back().endpoint_vertex_to_nearest_point = to_vertex.vertex_index;
        edges_.back().nearest_point_vertex_position_index = to_vertex.nearest_point_vertex_position_index;
        edges_.back().endpoint_vertex_to_nearest_point_is_from_vertex = false;
        
        // Add adjacency entries for the new edge
        adjacency_list_[new_vertex_id].emplace_back(edge.to_vertex, new_edge_id, second_length);
        to_adjacencies.emplace_back(new_vertex_id, new_edge_id, second_length);
    }
}

std::vector<RoadSplitByDistanceBracketsOutput> RoadSegmentation::splitRoadLinestringsByDistanceVector(const std::vector<double>& distance_breakpoints) {
    // Step 1: Initialize empty output vector and sort input vector
    std::vector<RoadSplitByDistanceBracketsOutput> output;
    
    // Reserve memory: worst case is each edge produces distance_breakpoints.size() segments
    size_t estimated_capacity = edges_.size() * distance_breakpoints.size();
    output.reserve(estimated_capacity);
    
    std::vector<double> sorted_distances = distance_breakpoints;
    std::sort(sorted_distances.begin(), sorted_distances.end());
    
    // Step 2: Loop through each edge in edges_ vector
    for (const auto& edge : edges_) {
        // Step 3: Check if edge's endpoint_vertex_to_nearest_point is max
        if (edge.endpoint_vertex_to_nearest_point == std::numeric_limits<size_t>::max()) {
            std::cout << "WARNING: Skipping edge " << edge.feature_id << " because endpoint_vertex_to_nearest_point is not set" << std::endl;
            continue;
        }
        
        // Step 4: Get the vertex corresponding to endpoint_vertex_to_nearest_point
        const Vertex& endpoint_vertex = getVertex(edge.endpoint_vertex_to_nearest_point);
        double distance_to_point_vertex = endpoint_vertex.distance_to_point_vertex;
        std::vector<std::pair<double, size_t>> distances_to_be_interpolated;
        
        // Step 5: Handle special cases before processing distance vector
        double largest_bracket_distance = sorted_distances.back() - distance_to_point_vertex;
        double smallest_bracket_distance = sorted_distances.front() - distance_to_point_vertex;

        const Vertex& nearest_point_vertex = getVertex(edge.nearest_point_vertex_position_index);
        const Vertex& from_vertex = getVertex(edge.from_vertex);
        const Vertex& to_vertex = getVertex(edge.to_vertex);

        bool endpoint_vertex_is_from_vertex = edge.endpoint_vertex_to_nearest_point_is_from_vertex;
        
        // Check if largest bracket is too small (<= 1)
        if (largest_bracket_distance <= 1) {
            // Use entire geometry with largest bracket
            std::string distance_category = ">=" + std::to_string(static_cast<int>(sorted_distances.back()));
            output.emplace_back(edge.geometry, edge.weight, distance_category, from_vertex.distance_to_point_vertex, to_vertex.distance_to_point_vertex, 
                              edge.feature_id, nearest_point_vertex.feature_id);
            continue;
        }
        
        // Check if smallest bracket is too large (>= edge.weight - 1)
        if (smallest_bracket_distance >= (edge.weight - 1)) {
            // Use entire geometry with smallest bracket
            std::string distance_category = "0-" + std::to_string(static_cast<int>(sorted_distances.front()));
            output.emplace_back(edge.geometry, edge.weight, distance_category, from_vertex.distance_to_point_vertex, to_vertex.distance_to_point_vertex, 
                              edge.feature_id, nearest_point_vertex.feature_id);
            continue;
        }
        
        // Loop through each element in the input vector
        for (size_t i = 0; i < sorted_distances.size(); ++i) {
            double distance = sorted_distances[i];
            double subtracted_value = distance - distance_to_point_vertex;
            if (subtracted_value >= 0 && subtracted_value <= edge.weight) {  // avoid small segments
                distances_to_be_interpolated.push_back(std::make_pair(subtracted_value, i));
            }
        }
        
        // If no distances to interpolate after filtering, it means the linestring in fact can be split into two parts with one part that is very close to 0 length
        if (distances_to_be_interpolated.empty()) {
            std::cout << "WARNING: No valid interpolation points found for edge " << edge.feature_id 
                      << " (distance_to_point_vertex: " << distance_to_point_vertex 
                      << ", edge_weight: " << edge.weight << ")" << std::endl;
            continue;
        }
        
        // Step 6: Initialize vector of tuples (Point, double, size_t) and cumulative length
        std::vector<std::tuple<Point, double, size_t>> point_tuples;
        double cumulative_length = 0.0;
        
        // Step 7: Loop through each point in edge_linestring
        auto edge_linestring = edge.geometry;  // Create a copy
        if (!endpoint_vertex_is_from_vertex) {
            std::reverse(edge_linestring.begin(), edge_linestring.end());
        }
        
        // Add original vertices to the vector
        for (size_t i = 0; i < edge_linestring.size(); ++i) {
            const auto& point = edge_linestring[i];
            if (i > 0) {
                // Calculate distance from previous point
                const auto& prev_point = edge_linestring[i - 1];
                cumulative_length += bg::distance(prev_point, point);
            }
            point_tuples.emplace_back(point, cumulative_length, std::numeric_limits<size_t>::max());
        }
        
        // Step 8: Loop through distances_to_be_interpolated and add interpolated points
        for (const auto& distance_pair : distances_to_be_interpolated) {
            double interpolation_distance = distance_pair.first;
            size_t sorted_index = distance_pair.second;
                    
            // Interpolate point
            Point interpolated_point;
            bg::line_interpolate(edge_linestring, interpolation_distance, interpolated_point);
            
            if (bg::is_empty(interpolated_point)) {
                std::cout << "WARNING: Failed to interpolate point for edge " << edge.feature_id 
                          << " at distance " << interpolation_distance << std::endl;
                continue;
            }
            
            // Add interpolated point to vector
            point_tuples.emplace_back(interpolated_point, interpolation_distance, sorted_index);
        }
        
        // Step 9: Sort the vector by cumulative length (second element)
        std::sort(point_tuples.begin(), point_tuples.end(), 
                  [](const auto& a, const auto& b) {
                      return std::get<1>(a) < std::get<1>(b);
                  });
        
        // Step 10: Loop through sorted tuples and construct linestrings
        LineString current_linestring;
        size_t current_interpolation_idx = 0;
        size_t last_valid_sorted_index = std::numeric_limits<size_t>::max();
        
        for (size_t i = 0; i < point_tuples.size(); ++i) {
            const auto& point_tuple = point_tuples[i];
            const Point& point = std::get<0>(point_tuple);
            double distance = std::get<1>(point_tuple);
            size_t sorted_index = std::get<2>(point_tuple);
            
            // Add point to current linestring
            current_linestring.push_back(point);
            
            // Check if we need to output a segment (current point has non-max sorted_index)
            bool should_output = false;
            if (sorted_index != std::numeric_limits<size_t>::max()) {
                // Current point is an interpolated point, output the segment
                should_output = true;
            } else if (i == point_tuples.size() - 1) {
                // Last point (original vertex), always output
                should_output = true;
            }
            
            if (should_output) {
                if (current_linestring.size() < 2) {
                    std::cout << "WARNING: Skipping edge " << edge.feature_id << " because current_linestring has less than 2 points" << std::endl;
                    continue;
                }

                // Calculate output parameters
                double length = bg::length(current_linestring);
                double from_distance = (current_interpolation_idx == 0) ? distance_to_point_vertex : sorted_distances[last_valid_sorted_index];
                double to_distance = (sorted_index == std::numeric_limits<size_t>::max()) ? (distance_to_point_vertex + edge.weight) : sorted_distances[sorted_index];
                
                std::string distance_category;
                if (sorted_index == std::numeric_limits<size_t>::max()) {
                    if (last_valid_sorted_index >= sorted_distances.size() - 1) {
                        // This is the largest bracket possible
                        distance_category = ">=" + std::to_string(static_cast<int>(sorted_distances.back()));
                    } else {
                        // Getting the next bracket
                        distance_category = std::to_string(static_cast<int>(sorted_distances[last_valid_sorted_index])) + "-" + std::to_string(static_cast<int>(sorted_distances[last_valid_sorted_index + 1]));
                    }
                } 
                // if (sorted_index != std::numeric_limits<size_t>::max())
                else {
                    if (last_valid_sorted_index != std::numeric_limits<size_t>::max()) {
                        // Regular bracket
                        distance_category = std::to_string(static_cast<int>(sorted_distances[last_valid_sorted_index])) + "-" + std::to_string(static_cast<int>(sorted_distances[sorted_index]));
                    } else if (sorted_index == 0) {
                        // This is the smallest bracket possible
                        distance_category = "0-" + std::to_string(static_cast<int>(sorted_distances.front()));
                    } else {
                        // Getting the previous bracket
                        distance_category = std::to_string(static_cast<int>(sorted_distances[sorted_index-1])) + "-" + std::to_string(static_cast<int>(sorted_distances[sorted_index]));
                    }
                } 
                
                // Add to output
                output.emplace_back(current_linestring, length, distance_category, from_distance, to_distance,
                                  edge.feature_id, nearest_point_vertex.feature_id);
                
                // Reset for next segment
                current_linestring.clear();
                current_linestring.push_back(point); // Start new segment with current point
                current_interpolation_idx++;
            }

            if (sorted_index != std::numeric_limits<size_t>::max()) {
                last_valid_sorted_index = sorted_index;
            }
            
        }
    }
    
    return output;
}

std::vector<RoadSplitByDistanceBracketsOutput> RoadSegmentation::extractEdgeInformation() {
    std::vector<RoadSplitByDistanceBracketsOutput> output;
    output.reserve(edges_.size());
    
    // Helper function to convert VertexType to string
    auto vertexTypeToString = [](VertexType type) -> std::string {
        switch (type) {
            case VertexType::POINT:
                return "POINT";
            case VertexType::INTERSECTION:
                return "INTERSECTION";
            case VertexType::SPLIT:
                return "SPLIT";
            default:
                return "UNKNOWN";
        }
    };
    
    // Loop through all edges
    for (const auto& edge : edges_) {
        if (edge.nearest_point_vertex_position_index == std::numeric_limits<size_t>::max()) {
            std::cout << "WARNING: Skipping edge " << edge.feature_id << " because nearest_point_vertex_position_index is not set" << std::endl;
            continue;
        }

        // Get the from and to vertices
        const Vertex& from_vertex = getVertex(edge.from_vertex);
        const Vertex& to_vertex = getVertex(edge.to_vertex);
        
        // Create distance category string: [FROM-VERTEX-TYPE]-[TO-VERTEX-TYPE]
        std::string distance_category = vertexTypeToString(from_vertex.type) + "-" + vertexTypeToString(to_vertex.type);
        
        // Get nearest point information
        const Vertex& nearest_point_vertex = getVertex(edge.nearest_point_vertex_position_index);
        size_t nearest_point_id = nearest_point_vertex.feature_id;
        
        // Create output structure
        output.emplace_back(
            edge.geometry,           // geometry
            edge.weight,             // length
            distance_category,       // distance_category
            from_vertex.distance_to_point_vertex,  // from_distance
            to_vertex.distance_to_point_vertex,    // to_distance
            edge.feature_id,         // raw_feature_id
            nearest_point_id         // nearest_point_id
        );
    }
    
    return output;
}

} // namespace graph
} // namespace adjfind 