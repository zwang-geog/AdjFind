#include "io/geojson_writer.hpp"
#include <fstream>
#include <stdexcept>

namespace adjfind {
namespace io {

std::string GeoJSONWriter::last_error_ = "";

bool GeoJSONWriter::writeToFile(const graph::GeospatialDataset& dataset, const std::string& filepath) {
    try {
        std::string geojson_string = writeToString(dataset);
        
        std::ofstream file(filepath);
        if (!file.is_open()) {
            setError("Failed to open file for writing: " + filepath);
            return false;
        }
        
        file << geojson_string;
        file.close();
        
        return true;
        
    } catch (const std::exception& e) {
        setError("Error writing file " + filepath + ": " + e.what());
        return false;
    }
}

std::string GeoJSONWriter::writeToString(const graph::GeospatialDataset& dataset) {
    try {
        nlohmann::json geojson;
        
        // Set the type
        geojson["type"] = "FeatureCollection";
        
        // Set CRS if provided
        if (!dataset.crs.empty()) {
            setCRS(geojson, dataset.crs);
        }
        
        // Convert features
        nlohmann::json features = nlohmann::json::array();
        for (const auto& feature : dataset.features) {
            features.push_back(featureToGeoJSON(feature));
        }
        geojson["features"] = features;
        
        return geojson.dump(2); // Pretty print with 2-space indentation
        
    } catch (const std::exception& e) {
        setError("Error converting dataset to GeoJSON: " + std::string(e.what()));
        throw std::runtime_error(last_error_);
    }
}

void GeoJSONWriter::setCRS(nlohmann::json& geojson, const std::string& crs) {
    try {
        if (crs.empty()) {
            return;
        }
        
        nlohmann::json crs_obj;
        
        // Always use the standard GeoJSON CRS format: {"type": "name", "properties": {"name": "EPSG:32633"}}
        // This is the recommended format per GeoJSON specification
        crs_obj["type"] = "name";
        crs_obj["properties"]["name"] = crs;
        
        geojson["crs"] = crs_obj;
        
    } catch (const std::exception& e) {
        setError("Error setting CRS: " + std::string(e.what()));
    }
}

nlohmann::json GeoJSONWriter::featureToGeoJSON(const graph::GeospatialFeature& feature) {
    try {
        nlohmann::json feature_json;
        
        // Set the type
        feature_json["type"] = "Feature";
        
        // Set geometry
        feature_json["geometry"] = feature.geometry;
        
        // Set properties (including ID)
        nlohmann::json properties = feature.properties;
        properties["id"] = feature.id;  // Add ID to properties
        feature_json["properties"] = properties;
        
        return feature_json;
        
    } catch (const std::exception& e) {
        throw std::runtime_error("Error converting feature to GeoJSON: " + std::string(e.what()));
    }
}

} // namespace io
} // namespace adjfind
