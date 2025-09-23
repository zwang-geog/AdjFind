#include "io/point_reader.hpp"
#include "io/geojson_reader.hpp"
#include "graph/common.hpp"
#include <stdexcept>
#include <iostream>
#include <sstream>
#include <algorithm>

namespace adjfind {
namespace io {

PointReader::PointReader(const PointReaderConfig& config)
    : config_(config) {
}

PointReader::~PointReader() = default;

bool PointReader::read() {
    // Clear any existing features
    points_.clear();
    
    try {
        // Read GeoJSON dataset
        graph::GeospatialDataset dataset = GeoJSONReader::readFromFile(config_.file_path);
        
        // Store coordinate system information
        coordinate_system_crs_ = dataset.crs;
        
        std::cout << "Point dataset coordinate system: " << coordinate_system_crs_ << std::endl;
        std::cout << "Point feature count: " << dataset.features.size() << std::endl;
        
        return readFeatures(dataset);
        
    } catch (const std::exception& e) {
        std::cerr << "Failed to read point file: " << config_.file_path << std::endl;
        std::cerr << "Error: " << e.what() << std::endl;
        return false;
    }
}

bool PointReader::readFeatures(const graph::GeospatialDataset& dataset) {
    points_.clear();
    
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
        
        // Convert geometry to Point
        graph::Point point = graph::geoJSONPointToBoost(feature.geometry);
        
        // Skip empty points
        if (graph::bg::is_empty(point)) {
            continue;
        }
        
        // Create point feature
        points_.emplace_back(feature_id, point);
    }
    
    // Check if we have any valid points
    if (points_.empty()) {
        std::cerr << "Warning: No valid point features found in dataset" << std::endl;
        return false;
    }
    
    std::cout << "Successfully read " << points_.size() << " point features" << std::endl;
    
    // Build spatial index
    buildSpatialIndex();
    
    return true;
}

void PointReader::buildSpatialIndex() {
    std::vector<graph::PointRTreeValue> rtree_values;
    rtree_values.reserve(points_.size());
    
    for (size_t i = 0; i < points_.size(); ++i) {
        const auto& point = points_[i];
        rtree_values.emplace_back(point.geometry, point.id, i);
    }
    
    // Build R-tree
    rtree_ = PointRTree(rtree_values.begin(), rtree_values.end());
    
    std::cout << "Built spatial index for " << rtree_values.size() << " points" << std::endl;
}

const graph::PointFeature& PointReader::getPointFeature(size_t index) const {
    if (index >= points_.size()) {
        throw std::out_of_range("Point feature index out of range");
    }
    return points_[index];
}


void PointReader::clearSpatialIndex() {
    rtree_.clear();
    std::cout << "Cleared point spatial index (R-tree)" << std::endl;
}

void PointReader::clearPoints() {
    points_.clear();
    std::cout << "Cleared point data " << std::endl;
}

} // namespace io
} // namespace adjfind 