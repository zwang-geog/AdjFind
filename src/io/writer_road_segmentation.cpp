#include "io/writer_road_segmentation.hpp"
#include "io/geojson_writer.hpp"
#include "graph/common.hpp"
#include <iostream>
#include <filesystem>
#include <algorithm>

namespace adjfind {
namespace io {

// Constructor and destructor are now defaulted in header

graph::GeospatialFeature RoadSegmentationWriter::resultToGeoJSONFeature(const graph::RoadSplitByDistanceBracketsOutput& result, 
                                                                       size_t feature_id) {
    // Convert geometry to GeoJSON
    nlohmann::json geometry = graph::lineStringToGeoJSON(result.geometry);
    
    // Create properties (ID will be added automatically by GeoJSONWriter)
    nlohmann::json properties = nlohmann::json::object();
    properties["road_id"] = result.raw_feature_id;
    properties["point_id"] = result.nearest_point_id;
    properties["length"] = result.length;
    properties["distance_category"] = result.distance_category;
    properties["from_distance"] = result.from_distance;
    properties["to_distance"] = result.to_distance;
    
    // Create feature
    graph::GeospatialFeature feature;
    feature.id = feature_id;
    feature.geometry = geometry;
    feature.properties = properties;
    
    return feature;
}

bool RoadSegmentationWriter::writeRoadSegmentationResults(const RoadSegmentationWriterConfig& config, 
                                                        const std::vector<graph::RoadSplitByDistanceBracketsOutput>& results) {
    clearError();
    
    if (results.empty()) {
        last_error_ = "No results to write";
        return false;
    }
    
    try {
        // Create GeoJSON dataset
        graph::GeospatialDataset dataset;
        dataset.crs = config.crs;
        
        // Convert results to GeoJSON features
        dataset.features.reserve(results.size());
        for (size_t i = 0; i < results.size(); ++i) {
            dataset.features.push_back(resultToGeoJSONFeature(results[i], i));
        }
        
        // Write to file
        GeoJSONWriter::writeToFile(dataset, config.output_file_path);
        
        std::cout << "Successfully wrote " << results.size() << " road segments to: " << config.output_file_path << std::endl;
        return true;
        
    } catch (const std::exception& e) {
        last_error_ = "Failed to write road segmentation results: " + std::string(e.what());
        std::cerr << last_error_ << std::endl;
        return false;
    }
}

} // namespace io
} // namespace adjfind