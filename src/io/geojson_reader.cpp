#include "io/geojson_reader.hpp"
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <optional>

namespace adjfind {
namespace io {

std::string GeoJSONReader::last_error_ = "";

graph::GeospatialDataset GeoJSONReader::readFromFile(const std::string& filepath) {
    try {
        std::ifstream file(filepath);
        if (!file.is_open()) {
            setError("Failed to open file: " + filepath);
            throw std::runtime_error(last_error_);
        }
        
        std::stringstream buffer;
        buffer << file.rdbuf();
        file.close();
        
        return readFromString(buffer.str());
        
    } catch (const std::exception& e) {
        setError("Error reading file " + filepath + ": " + e.what());
        throw std::runtime_error(last_error_);
    }
}

graph::GeospatialDataset GeoJSONReader::readFromString(const std::string& geojson_string) {
    try {
        nlohmann::json geojson = nlohmann::json::parse(geojson_string);
        
        // Validate that this is a FeatureCollection
        if (!geojson.contains("type") || geojson["type"] != "FeatureCollection") {
            setError("Invalid GeoJSON: Expected FeatureCollection type");
            throw std::runtime_error(last_error_);
        }
        
        // Parse CRS information
        std::string crs = parseCRS(geojson);
        
        // Validate CRS - cannot be WGS84 (EPSG:4326)
        if (crs.find("EPSG::4326") != std::string::npos || crs.find("EPSG:4326") != std::string::npos) {
            setError("Coordinate system cannot be WGS84 (EPSG:4326) and must be in projected coordinate system in feet or meter");
            throw std::runtime_error(last_error_);
        }
        
        // Parse features
        std::vector<graph::GeospatialFeature> features;
        if (geojson.contains("features") && geojson["features"].is_array()) {
            size_t featureIndex = 0;
            for (const auto& feature_json : geojson["features"]) {
                try {
                    auto feature = parseFeature(feature_json, featureIndex);
                    if (feature.has_value()) {
                        features.push_back(feature.value());
                    }
                    featureIndex++;
                } catch (const std::exception& e) {
                    setError("Error parsing feature: " + std::string(e.what()));
                    throw std::runtime_error(last_error_);
                }
            }
        }
        
        // Validate that we have at least one feature
        if (features.empty()) {
            setError("No valid features found in GeoJSON");
            throw std::runtime_error(last_error_);
        }
        
        return graph::GeospatialDataset(crs, features);
        
    } catch (const nlohmann::json::parse_error& e) {
        setError("JSON parse error: " + std::string(e.what()));
        throw std::runtime_error(last_error_);
    } catch (const std::exception& e) {
        setError("Error parsing GeoJSON string: " + std::string(e.what()));
        throw std::runtime_error(last_error_);
    }
}

std::string GeoJSONReader::parseCRS(const nlohmann::json& geojson) {
    try {
        if (geojson.contains("crs") && geojson["crs"].is_object()) {
            const auto& crs_obj = geojson["crs"];
            
            // Handle standard GeoJSON CRS format: {"type": "name", "properties": {"name": "EPSG:32633"}}
            if (crs_obj.contains("type") && crs_obj["type"] == "name" &&
                crs_obj.contains("properties") && crs_obj["properties"].is_object() &&
                crs_obj["properties"].contains("name")) {
                return crs_obj["properties"]["name"].get<std::string>();
            }
            
            // Handle legacy CRS format: {"type": "EPSG", "properties": {"code": 32633}}
            // This format is supported for backward compatibility
            if (crs_obj.contains("type") && crs_obj["type"] == "EPSG" &&
                crs_obj.contains("properties") && crs_obj["properties"].is_object() &&
                crs_obj["properties"].contains("code")) {
                int code = crs_obj["properties"]["code"].get<int>();
                return "EPSG:" + std::to_string(code);
            }
        }
        
        // No CRS found, return empty string
        return "";
        
    } catch (const std::exception& e) {
        setError("Error parsing CRS: " + std::string(e.what()));
        return "";
    }
}

std::optional<graph::GeospatialFeature> GeoJSONReader::parseFeature(const nlohmann::json& feature_json, size_t id) {
    try {
        // Validate feature structure
        if (!feature_json.contains("type") || feature_json["type"] != "Feature") {
            throw std::runtime_error("Invalid feature: Expected Feature type");
        }
        
        // Check if geometry is present - if not, return nullopt
        if (!feature_json.contains("geometry")) {
            return std::nullopt;
        }
        
        // Extract geometry
        nlohmann::json geometry = feature_json["geometry"];
        
        // Extract properties - if not present, use empty object
        nlohmann::json properties = feature_json.value("properties", nlohmann::json::object());
        
        return graph::GeospatialFeature(id, geometry, properties);
        
    } catch (const std::exception& e) {
        throw std::runtime_error("Error parsing feature: " + std::string(e.what()));
    }
}

} // namespace io
} // namespace adjfind
