#include "io/road_reader.hpp"
#include "io/geojson_reader.hpp"
#include "graph/common.hpp"
#include <stdexcept>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <limits>

namespace adjfind {
namespace io {

RoadReader::RoadReader(const RoadReaderConfig& config)
    : config_(config) {
}

RoadReader::~RoadReader() = default;

bool RoadReader::read() {
    // Clear any existing features
    roads_.clear();
    
    try {
        // Read GeoJSON dataset
        graph::GeospatialDataset dataset = GeoJSONReader::readFromFile(config_.file_path);
        
        // Store coordinate system information
        coordinate_system_crs_ = dataset.crs;
        
        std::cout << "Road dataset coordinate system: " << coordinate_system_crs_ << std::endl;
        std::cout << "Road feature count: " << dataset.features.size() << std::endl;
        
        return readFeatures(dataset);
        
    } catch (const std::exception& e) {
        std::cerr << "Failed to read road file: " << config_.file_path << std::endl;
        std::cerr << "Error: " << e.what() << std::endl;
        return false;
    }
}

bool RoadReader::readFeatures(const graph::GeospatialDataset& dataset) {
    roads_.clear();
    
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

    // Check if z-level fields exist
    bool has_from_z = !config_.from_z_field.empty();
    bool has_to_z = !config_.to_z_field.empty();
    
    if (has_from_z && !has_to_z) {
        std::cout << "Warning: from-z-field specified but to-z-field not specified. Using default." << std::endl;
    }
    if (!has_from_z && has_to_z) {
        std::cout << "Warning: to-z-field specified but from-z-field not specified. Using default." << std::endl;
    }

    // Check if length field exists
    bool has_length_field = !config_.length_field.empty();
    if (has_length_field && !dataset.features.empty()) {
        const auto& first_feature = dataset.features[0];
        if (!first_feature.properties.contains(config_.length_field)) {
            std::cout << "Warning: Specified length field '" << config_.length_field 
                     << "' not found. Will compute length from geometry." << std::endl;
            has_length_field = false;
        }
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
        
        // Convert geometry to LineString
        graph::LineString linestring = graph::geoJSONLineStringToBoost(feature.geometry);
        
        // Skip empty linestrings
        if (graph::bg::is_empty(linestring)) {
            continue;
        }
        
        // Get length
        double length;
        if (has_length_field) {
            length = graph::getFieldValueAsDouble(feature.properties, config_.length_field, 0.0);
            if (length <= 0.0) {
                length = graph::bg::length(linestring);
            }
        } else {
            length = graph::bg::length(linestring);
        }
        
        // Skip features with non-positive length
        if (length <= 0.0) {
            continue;
        }
        
        // Get z-levels
        double from_z, to_z;
        if (has_from_z && has_to_z) {
            from_z = graph::getFieldValueAsDouble(feature.properties, config_.from_z_field, config_.default_from_z);
            to_z = graph::getFieldValueAsDouble(feature.properties, config_.to_z_field, config_.default_to_z);
        } else {
            from_z = config_.default_from_z;
            to_z = config_.default_to_z;
        }
        
        // Create road feature
        roads_.emplace_back(feature_id, linestring, length, from_z, to_z);
    }
    
    // Check if we have any valid roads
    if (roads_.empty()) {
        std::cerr << "Warning: No valid road features found in dataset" << std::endl;
        return false;
    }
    
    std::cout << "Successfully read " << roads_.size() << " road features" << std::endl;
    
    // Build spatial index
    buildSpatialIndex();
    
    return true;
}

void RoadReader::buildSpatialIndex() {
    std::vector<graph::RoadRTreeValue> rtree_values;
    rtree_values.reserve(roads_.size() * 10); // Estimate segments per road
    
    for (size_t i = 0; i < roads_.size(); ++i) {
        const auto& road = roads_[i];
        const auto& linestring = road.geometry;
        
        // Create segments from linestring
        for (size_t j = 0; j < linestring.size() - 1; ++j) {
            graph::Segment segment(linestring[j], linestring[j + 1]);
            rtree_values.emplace_back(segment, i, j);  // Store vector index 'i' instead of road.id
        }
    }
    
    // Build R-tree
    rtree_ = RoadRTree(rtree_values.begin(), rtree_values.end());
    
    std::cout << "Built spatial index for " << rtree_values.size() << " road segments" << std::endl;
}

graph::RoadFeature& RoadReader::getRoadFeature(size_t index) {
    if (index >= roads_.size()) {
        throw std::out_of_range("Road feature index out of range");
    }
    return roads_[index];
}

std::optional<std::tuple<size_t, size_t, graph::Point, double>> RoadReader::findNearestRoadSegment(const graph::Point& query_point) const {
    if (roads_.empty()) {
        return std::nullopt;
    }
    
    // Query R-tree for nearest segment
    std::vector<graph::RoadRTreeValue> results;
    rtree_.query(graph::bgi::nearest(query_point, 1), std::back_inserter(results));
    
    if (results.empty()) {
        return std::nullopt;
    }
    
    const auto& nearest = results[0];
    size_t road_index = nearest.road_index;  // Direct vector index, no lookup needed
    size_t segment_index = nearest.segment_index;
    
    // Calculate snapped point on the segment
    graph::Point snapped_point = calculateClosestPointOnSegment(query_point, nearest.segment.first, nearest.segment.second);
    
    // Calculate distance from segment's first point to the snapped point using bg::distance
    double distance_to_first = graph::bg::distance(nearest.segment.first, snapped_point);
    
    return std::make_tuple(road_index, segment_index, snapped_point, distance_to_first);
}

graph::Point RoadReader::calculateClosestPointOnSegment(const graph::Point& query_point, const graph::Point& segment_start_point, const graph::Point& segment_end_point) const {
    double x1 = graph::bg::get<0>(segment_start_point);
    double y1 = graph::bg::get<1>(segment_start_point);
    double x2 = graph::bg::get<0>(segment_end_point);
    double y2 = graph::bg::get<1>(segment_end_point);
    double px = graph::bg::get<0>(query_point);
    double py = graph::bg::get<1>(query_point);
    
    // Calculate the vector from p1 to p2
    double dx = x2 - x1;
    double dy = y2 - y1;
    
    // Calculate the vector from p1 to query point
    double qx = px - x1;
    double qy = py - y1;
    
    // Calculate the dot product
    double dot = qx * dx + qy * dy;
    
    // Calculate the squared length of the segment
    double len_sq = dx * dx + dy * dy;
    
    // If segment has zero length, return p1
    if (len_sq == 0) {
        return segment_start_point;
    }
    
    // Calculate the parameter t (0 <= t <= 1)
    double t = dot / len_sq;
    
    // Clamp t to [0, 1]
    t = std::max(0.0, std::min(1.0, t));
    
    // Calculate the closest point
    double closest_x = x1 + t * dx;
    double closest_y = y1 + t * dy;
    
    return graph::Point(closest_x, closest_y);
}

void RoadReader::clearSpatialIndex() {
    rtree_.clear();
    std::cout << "Cleared road spatial index (R-tree)" << std::endl;
}

void RoadReader::clearRoads() {
    roads_.clear();
    std::cout << "Cleared road data "  << std::endl;
}

} // namespace io
} // namespace adjfind 