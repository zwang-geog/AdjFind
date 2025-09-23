#include "io/polygon_reader.hpp"
#include "io/geojson_reader.hpp"
#include "graph/common.hpp"
#include "graph/adj_graph.hpp"
#include <iostream>
#include <sstream>
#include <algorithm>
#include <unordered_set>
#include <boost/geometry.hpp>
#include <boost/geometry/strategies/buffer.hpp>
#include <boost/geometry/geometries/box.hpp>

namespace adjfind {
namespace io {

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

PolygonReader::PolygonReader(const PolygonReaderConfig& config)
    : config_(config) {
}

PolygonReader::~PolygonReader() = default;

bool PolygonReader::read() {
    // Clear any existing features
    polygons_.clear();
    
    try {
        // Read GeoJSON dataset
        graph::GeospatialDataset dataset = GeoJSONReader::readFromFile(config_.file_path);
        
        // Store coordinate system information
        coordinate_system_crs_ = dataset.crs;
        
        std::cout << "Polygon dataset coordinate system: " << coordinate_system_crs_ << std::endl;
        std::cout << "Polygon feature count: " << dataset.features.size() << std::endl;
        
        return readFeatures(dataset);
        
    } catch (const std::exception& e) {
        std::cerr << "Failed to read polygon file: " << config_.file_path << std::endl;
        std::cerr << "Error: " << e.what() << std::endl;
        return false;
    }
}

const graph::PolygonFeature& PolygonReader::getPolygonFeature(size_t index) const {
    if (index >= polygons_.size()) {
        throw std::out_of_range("Polygon index out of range");
    }
    return polygons_[index];
}

void PolygonReader::populateSnappableRoadIds(graph::AdjGraph& adj_graph) {
    // Spatial index is already built in readFeatures()
    // Just populate snappable road IDs for polygons that don't have them
    // If snappable road IDs are already populated, map them to edge indices

    if (!config_.road_ids_snappable_field.empty()) {
        adj_graph.buildFeatureIdMapping();
    }

    for (auto& polygon : polygons_) {
        // Even if obstacle-only, we still need to populate snappable road IDs because query will be made for obstacle as well
        
        if (polygon.snappable_road_ids.empty()) {
            std::vector<std::pair<std::optional<size_t>, graph::Point>> road_edge_index_vector; // Expand snappable_road_ids for later sectional search
            // Find nearest road edge for each point in the polygon's outer ring
            std::unordered_set<size_t> unique_road_ids;
            for (const auto& point : polygon.outer_ring) {
                auto nearest_edge = adj_graph.findNearestEdge(point);
                road_edge_index_vector.push_back(std::make_pair(nearest_edge, point));
                if (nearest_edge.has_value()) {
                    unique_road_ids.insert(nearest_edge.value());
                }
            }
            
            // Convert to vector and assign
            polygon.snappable_road_ids.assign(unique_road_ids.begin(), unique_road_ids.end());

            // Loop through polygon.snappable_road_ids to obtain a set of unique nearest_point_vertex_position_index
            std::unordered_set<size_t> unique_nearest_point_vertex_position_indices;
            for (size_t edge_index : polygon.snappable_road_ids) {
                // Get the edge directly using the edge index
                graph::Edge edge = adj_graph.getEdge(edge_index);
                if (edge.nearest_point_vertex_position_index != std::numeric_limits<size_t>::max()) {
                    unique_nearest_point_vertex_position_indices.insert(edge.nearest_point_vertex_position_index);
                }
            }

            // Loop through the road_edge_index_vector starting from 0
            for (size_t i = 0; i < road_edge_index_vector.size(); ++i) {
                if (road_edge_index_vector[i].first.has_value() && road_edge_index_vector[(i + 1) % road_edge_index_vector.size()].first.has_value()) {
                    size_t edge_index_1 = road_edge_index_vector[i].first.value();
                    size_t edge_index_2 = road_edge_index_vector[(i + 1) % road_edge_index_vector.size()].first.value();

                    // Criteria 1: Adjacent sanpped edge must be different
                    if (edge_index_1 != edge_index_2) {
                        graph::Edge edge_1 = adj_graph.getEdge(edge_index_1);
                        graph::Edge edge_2 = adj_graph.getEdge(edge_index_2);

                        // Criteria 2: Closest point vertex must be different
                        // if (edge_1.nearest_point_vertex_position_index == edge_2.nearest_point_vertex_position_index) {
                        //     continue;
                        // }

                        // Criteria 3: The snapped edges must not share the same endpoint vertex
                        if (edge_1.from_vertex == edge_2.to_vertex || edge_1.to_vertex == edge_2.from_vertex) {
                            continue;
                        }

                        // Criteria 4: The polygon boundary segment length must be greater than the minimum polygon boundary segment length for nearest road edge detection
                        if (bg::distance(road_edge_index_vector[i].second, road_edge_index_vector[(i + 1) % road_edge_index_vector.size()].second) <= (getMinPolygonBoundarySegmentLengthForNearestRoadEdgeDetection() + 1.0)) {
                            continue;
                        }

                        graph::MultiPoint polygon_boundary_segment_interpolated_points;
                        graph::LineString polygon_boundary_segment_line;
                        polygon_boundary_segment_line.push_back(road_edge_index_vector[i].second);
                        polygon_boundary_segment_line.push_back(road_edge_index_vector[(i + 1) % road_edge_index_vector.size()].second);
                        double interpolation_distance = std::round(getMinPolygonBoundarySegmentLengthForNearestRoadEdgeDetection());

                        bg::line_interpolate(polygon_boundary_segment_line, interpolation_distance, polygon_boundary_segment_interpolated_points);
                        
                        std::unordered_set<size_t> snappable_edge_indices_for_interpolated_points;
                        for (const auto& point : polygon_boundary_segment_interpolated_points) {
                            auto nearest_edge = adj_graph.findNearestEdge(point);
                            if (nearest_edge.has_value()) {
                                // Criteria 5: If the edge_index is already in polygon.snappable_road_ids, continue
                                if (std::find(polygon.snappable_road_ids.begin(), polygon.snappable_road_ids.end(), nearest_edge.value()) != polygon.snappable_road_ids.end()) {
                                    continue;
                                }

                                // compute snapped point on edge
                                auto [projected_point, distance_to_nearest_point, dist_to_closest_edge, nearest_point_vertex_position_index, edge_index, is_outside_segment] = adj_graph.computeSnappedPointOnEdge(nearest_edge.value(), point);
                                
                                // Criteria 6: If the segment from the interpolated point to the projected point intersects polygon.shrink_geometry (or crosses polygon.geometry if shrink_geometry is not available), continue
                                graph::LineString segment_to_projected_point;
                                segment_to_projected_point.push_back(point);
                                segment_to_projected_point.push_back(projected_point);
                                
                                bool polygon_boundary_at_back = false;
                                if (polygon.shrink_geometry.has_value()) {
                                    if (bg::intersects(segment_to_projected_point, polygon.shrink_geometry.value())) {
                                        polygon_boundary_at_back = true;
                                    }
                                } else {
                                    if (bg::crosses(segment_to_projected_point, polygon.geometry)) {
                                        polygon_boundary_at_back = true;
                                    }
                                }
                                
                                if (polygon_boundary_at_back) {
                                    continue;
                                }

                                snappable_edge_indices_for_interpolated_points.insert(nearest_edge.value());
                            }
                        }

                        // Only add to map if we found snappable edges
                        if (!snappable_edge_indices_for_interpolated_points.empty()) {
                            // Add the snappable edge indices to the interpolated_snappable_edge_indices_map
                            // Map the current boundary segment index i to the vector of edge indices
                            std::vector<size_t> edge_indices_vector(snappable_edge_indices_for_interpolated_points.begin(),
                                                                    snappable_edge_indices_for_interpolated_points.end());

                            polygon.interpolated_snappable_edge_indices_map[i] = edge_indices_vector;
                        }
                    }
                }
            }
        }
        else {
            // Map snappable road IDs to edge indices
            // Collect all edge indices for the feature IDs
            std::unordered_set<size_t> unique_edge_indices;
            for (size_t feature_id : polygon.snappable_road_ids) {
                std::vector<size_t> edge_indices = adj_graph.getEdgeIndicesForFeatureId(feature_id);
                unique_edge_indices.insert(edge_indices.begin(), edge_indices.end());
            }
            
            // Update the polygon's snappable_road_ids with the mapped edge indices
            polygon.snappable_road_ids.assign(unique_edge_indices.begin(), unique_edge_indices.end());
        }
    }
    
    // Clear feature ID mapping if road_ids_snappable_field is configured
    if (!config_.road_ids_snappable_field.empty()) {
        adj_graph.clearFeatureIdMapping();
    }
}

void PolygonReader::clearSpatialIndex() {
    rtree_.clear();
}

void PolygonReader::clearPolygons() {
    polygons_.clear();
    rtree_.clear();
}

bool PolygonReader::readFeatures(const graph::GeospatialDataset& dataset) {
    // Check if the specified ID field exists in the first feature
    bool use_feature_id = false;
    if (!config_.id_field.empty() && !dataset.features.empty()) {
        const auto& first_feature = dataset.features[0];
        if (!first_feature.properties.contains(config_.id_field)) {
            std::cout << "Warning: Specified ID field '" << config_.id_field 
                     << "' not found. Falling back to feature index." << std::endl;
            use_feature_id = true;
        }
    } else {
        use_feature_id = true;
    }

    // Process all features
    for (const auto& feature : dataset.features) {
        // Get feature ID
        size_t feature_id;
        if (use_feature_id) {
            feature_id = feature.id;
        } else {
            auto id_opt = graph::getFieldValueAsSizeT(feature.properties, config_.id_field);
            if (!id_opt) {
                std::cerr << "Warning: Could not get ID for feature " << feature.id 
                         << ". Using feature index instead." << std::endl;
                feature_id = feature.id;
            } else {
                feature_id = *id_opt;
            }
        }
        
        // Convert geometry to Polygon
        graph::Polygon polygon = graph::geoJSONPolygonToBoost(feature.geometry);
        
        // Skip empty polygons
        if (graph::bg::is_empty(polygon)) {
            continue;
        }
        
        // Get obstacle-only flag
        bool is_obstacle_only = false;
        if (!config_.is_obstacle_only_field.empty()) {
            is_obstacle_only = graph::getFieldValueAsBool(feature.properties, config_.is_obstacle_only_field, false);
        }
        
        // Get snappable road IDs
        std::vector<size_t> snappable_road_ids;
        if (!config_.road_ids_snappable_field.empty()) {
            std::string road_ids_str = graph::getFieldValueAsString(feature.properties, config_.road_ids_snappable_field);
            if (!road_ids_str.empty()) {
                snappable_road_ids = parseRoadIds(road_ids_str);
            }
        }
        
        // Process the polygon
        processSinglePolygon(polygon, feature_id, is_obstacle_only, snappable_road_ids);
    }
    
    // Check if we have any valid polygons
    if (polygons_.empty()) {
        std::cerr << "Warning: No valid polygon features found in dataset" << std::endl;
        return false;
    }
    
    std::cout << "Successfully read " << polygons_.size() << " polygon features" << std::endl;
    
    // Build spatial index
    buildSpatialIndex();
    
    return true;
}

void PolygonReader::buildSpatialIndex() {
    std::vector<graph::PolygonRTreeValue> rtree_values;
    rtree_values.reserve(polygons_.size());
    
    for (const auto& polygon : polygons_) {
        // Use shrink geometry if available, otherwise use original
        graph::Box bounding_box;
        if (polygon.shrink_geometry.has_value()) {
            bg::envelope(polygon.shrink_geometry.value(), bounding_box);
        } else {
            graph::Polygon temp_polygon = polygon.geometry;
            bg::envelope(temp_polygon, bounding_box);
        }
        
        rtree_values.emplace_back(bounding_box, polygon.index);
    }
    
    // Build R-tree
    rtree_ = PolygonRTree(rtree_values.begin(), rtree_values.end());
    
    std::cout << "Built spatial index for " << rtree_values.size() << " polygons" << std::endl;
}

std::vector<size_t> PolygonReader::parseRoadIds(const std::string& road_ids_str) const {
    std::vector<size_t> road_ids;
    std::istringstream iss(road_ids_str);
    std::string token;
    
    while (std::getline(iss, token, ',')) {
        // Trim whitespace
        token.erase(0, token.find_first_not_of(" \t\r\n"));
        token.erase(token.find_last_not_of(" \t\r\n") + 1);
        
        if (!token.empty()) {
            try {
                size_t road_id = std::stoull(token);
                road_ids.push_back(road_id);
            } catch (const std::exception& e) {
                std::cerr << "Warning: Failed to parse road ID '" << token << "': " << e.what() << std::endl;
            }
        }
    }
    
    return road_ids;
}

std::vector<graph::PolygonFeature> PolygonReader::queryPolygonRTree(const graph::LineString& segment) const {
    std::vector<graph::PolygonFeature> crossed_obstacles;
    std::vector<graph::PolygonRTreeValue> candidates;
    
    rtree_.query(
        bgi::intersects(segment) && 
        bgi::satisfies([this, &segment](const graph::PolygonRTreeValue& candidate) {
            const auto& polygon = polygons_[candidate.polygon_index];

            // Use pre-computed shrink geometry for performance optimization
            if (polygon.shrink_geometry.has_value()) {
                const graph::Polygon& shrunk_polygon = *polygon.shrink_geometry;
                return bg::intersects(segment, shrunk_polygon);
            } else {
                // Interior intersects
                return bg::crosses(segment, polygon.geometry) || 
                       bg::within(segment, polygon.geometry);
            }
        }),
        std::back_inserter(candidates)
    );
    
    for (const auto& candidate : candidates) {
        crossed_obstacles.push_back(polygons_[candidate.polygon_index]);
    }
    
    return crossed_obstacles;
}

std::optional<graph::Polygon> PolygonReader::computeShrinkGeometry(const graph::Polygon& geometry, size_t feature_id) const {
    // Static buffer strategies - declared once and reused for performance  
    static const boost::geometry::strategy::buffer::distance_symmetric<double> distance_strategy(-1e-2);
    static const boost::geometry::strategy::buffer::end_flat end_strategy;
    static const boost::geometry::strategy::buffer::side_straight side_strategy;
    static const boost::geometry::strategy::buffer::join_miter join_strategy;
    static const boost::geometry::strategy::buffer::point_circle point_strategy(8);

    try {
        // Buffer operation outputs to multi_polygon, not single polygon
        bg::model::multi_polygon<graph::Polygon> buffered_multi_polygon;
        
        bg::buffer(geometry, buffered_multi_polygon, 
                  distance_strategy, side_strategy, join_strategy, end_strategy, point_strategy);
        
        // Check if buffering produced any result
        if (!graph::bg::is_empty(buffered_multi_polygon)) {
            // Use the first (largest) polygon from the result
            return buffered_multi_polygon[0];
        }
        // If buffering failed or produced no result, shrink_geometry remains nullopt
        
    } catch (const std::exception& e) {
        // If buffering fails, leave shrink_geometry as nullopt to fall back to original intersection checking
        std::cerr << "Warning: Failed to compute shrink geometry for polygon " << feature_id 
                  << ". Will fall back to original intersection checking. Error: " << e.what() << std::endl;
    }
    
    return std::nullopt;
}

void PolygonReader::processSinglePolygon(const graph::Polygon& geometry, size_t feature_id, bool is_obstacle_only, 
                                        const std::vector<size_t>& snappable_road_ids) {
    
    // Use the provided Boost geometry polygon directly
    const graph::Polygon& boost_polygon = geometry;
    
    // Extract outer ring
    const auto& outer = bg::exterior_ring(boost_polygon);
    std::vector<graph::Point> outer_ring;
    
    // Copy all points from exterior ring
    outer_ring.assign(outer.begin(), outer.end());
    
    // Remove duplicate closing point if present
    if (outer_ring.size() > 1 && 
        bg::get<0>(outer_ring.front()) == bg::get<0>(outer_ring.back()) &&
        bg::get<1>(outer_ring.front()) == bg::get<1>(outer_ring.back())) {
        outer_ring.pop_back();
    }
    
    // Ensure we have at least 3 points for a valid polygon
    if (outer_ring.size() < 3) {
        std::cerr << "Warning: Invalid polygon feature " << feature_id 
                 << " with insufficient points (" << outer_ring.size() << "). Skipping." << std::endl;
        return;
    }
    
    // Compute shrink geometry
    auto shrink_geometry = computeShrinkGeometry(boost_polygon, feature_id);
    
    // Create polygon feature
    size_t index = polygons_.size();
    polygons_.emplace_back(index, feature_id, outer_ring, boost_polygon, shrink_geometry, 
                          snappable_road_ids, is_obstacle_only);
}



} // namespace io
} // namespace adjfind
