#include "graph/common.hpp"
#include <iostream>

namespace adjfind {
namespace graph {

// Geometry Conversion Utilities for GeoJSON
nlohmann::json pointToGeoJSON(const Point& point) {
    nlohmann::json geometry;
    geometry["type"] = "Point";
    geometry["coordinates"] = {bg::get<0>(point), bg::get<1>(point)};
    return geometry;
}

nlohmann::json lineStringToGeoJSON(const LineString& linestring) {
    nlohmann::json geometry;
    geometry["type"] = "LineString";
    nlohmann::json coordinates = nlohmann::json::array();
    
    for (const auto& point : linestring) {
        coordinates.push_back({bg::get<0>(point), bg::get<1>(point)});
    }
    
    geometry["coordinates"] = coordinates;
    return geometry;
}

nlohmann::json multiLineStringToGeoJSON(const MultiLineString& multiLineString) {
    nlohmann::json geometry;
    geometry["type"] = "MultiLineString";
    nlohmann::json coordinates = nlohmann::json::array();
    
    for (const auto& linestring : multiLineString) {
        nlohmann::json linestringCoords = nlohmann::json::array();
        for (const auto& point : linestring) {
            linestringCoords.push_back({bg::get<0>(point), bg::get<1>(point)});
        }
        coordinates.push_back(linestringCoords);
    }
    
    geometry["coordinates"] = coordinates;
    return geometry;
}

nlohmann::json polygonToGeoJSON(const Polygon& polygon) {
    nlohmann::json geometry;
    geometry["type"] = "Polygon";
    nlohmann::json coordinates = nlohmann::json::array();
    
    // Outer ring
    nlohmann::json outerRing = nlohmann::json::array();
    for (const auto& point : polygon.outer()) {
        outerRing.push_back({bg::get<0>(point), bg::get<1>(point)});
    }
    coordinates.push_back(outerRing);
    
    geometry["coordinates"] = coordinates;
    return geometry;
}

// GeoJSON to Boost Geometry Conversion Utilities
Point geoJSONPointToBoost(const nlohmann::json& geometry) {
    if (geometry["type"] == "Point") {
        const auto& coords = geometry["coordinates"];
        if (coords.size() >= 2) {
            return Point(coords[0].get<double>(), coords[1].get<double>());
        } else {
            return Point(); // Empty point
        }
    } else if (geometry["type"] == "MultiPoint") {
        const auto& coords = geometry["coordinates"];
        if (coords.size() > 1) {
            std::cerr << "Warning: MultiPoint geometry contains " << coords.size() 
                      << " points, using only the first point" << std::endl;
        }
        if (coords.size() > 0) {
            const auto& firstPoint = coords[0];
            if (firstPoint.size() >= 2) {
                return Point(firstPoint[0].get<double>(), firstPoint[1].get<double>());
            } else {
                return Point(); // Empty point
            }
        } else {
            return Point(); // Empty point
        }
    } else {
        throw std::runtime_error("Invalid geometry type for point conversion: " + geometry["type"].get<std::string>());
    }
}

LineString geoJSONLineStringToBoost(const nlohmann::json& geometry) {
    if (geometry["type"] == "LineString") {
        LineString linestring;
        const auto& coords = geometry["coordinates"];
        if (coords.size() > 1) {
            for (const auto& coord : coords) {
                if (coord.size() >= 2) {
                    linestring.push_back(Point(coord[0].get<double>(), coord[1].get<double>()));
                }
            }
        }
        return linestring;
    } else if (geometry["type"] == "MultiLineString") {
        const auto& coords = geometry["coordinates"];
        if (coords.size() > 1) {
            std::cerr << "Warning: MultiLineString geometry contains " << coords.size() 
                      << " linestrings, using only the first linestring" << std::endl;
        }
        if (coords.size() > 0) {
            LineString linestring;
            const auto& firstLineString = coords[0];
            if (firstLineString.size() > 1) {
                for (const auto& coord : firstLineString) {
                    if (coord.size() >= 2) {
                        linestring.push_back(Point(coord[0].get<double>(), coord[1].get<double>()));
                    }
                }
            }
            return linestring;
        } else {
            return LineString();
        }
    } else {
        throw std::runtime_error("Invalid geometry type for linestring conversion: " + geometry["type"].get<std::string>());
    }
}

Polygon geoJSONPolygonToBoost(const nlohmann::json& geometry) {
    if (geometry["type"] == "Polygon") {
        Polygon polygon;
        const auto& coords = geometry["coordinates"];
        
        if (coords.size() == 0) {
            return polygon;
        }
        
        // Outer ring
        const auto& outerRing = coords[0];
        if (outerRing.size() > 0) {
            LinearRing outer;
            for (const auto& coord : outerRing) {
                if (coord.size() >= 2) {
                    outer.push_back(Point(coord[0].get<double>(), coord[1].get<double>()));
                }
            }
            
            // Ensure the ring is closed (Boost geometry requirement)
            if (outer.size() > 0 && 
                (bg::get<0>(outer.front()) != bg::get<0>(outer.back()) ||
                 bg::get<1>(outer.front()) != bg::get<1>(outer.back()))) {
                outer.push_back(outer.front());
            }
            
            polygon.outer() = outer;
            
            // Validate and correct the polygon
            if (!bg::is_valid(polygon)) {
                bg::correct(polygon);
                
                if (!bg::is_valid(polygon)) {
                    std::string reason;
                    if (!bg::is_valid(polygon, reason)) {
                        std::cerr << "Error: Polygon validation failed after correction (" << reason << ")" << std::endl;
                    }
                    throw std::runtime_error("Failed to create valid polygon from GeoJSON");
                }
            }
        }
        
        return polygon;
    } else if (geometry["type"] == "MultiPolygon") {
        const auto& coords = geometry["coordinates"];
        if (coords.size() > 1) {
            std::cerr << "Warning: MultiPolygon geometry contains " << coords.size() 
                      << " polygons, using only the first polygon" << std::endl;
        }
        if (coords.size() > 0) {
            // Convert the first polygon
            nlohmann::json firstPolygon;
            firstPolygon["type"] = "Polygon";
            firstPolygon["coordinates"] = coords[0];
            return geoJSONPolygonToBoost(firstPolygon);
        } else {
            return Polygon();
        }
    } else {
        throw std::runtime_error("Invalid geometry type for polygon conversion: " + geometry["type"].get<std::string>());
    }
}

// GeoJSON Field Value Utilities
std::optional<size_t> getFieldValueAsSizeT(const nlohmann::json& properties,
                                          const std::string& field_name) {
    if (field_name.empty() || !properties.contains(field_name)) {
        return std::nullopt;
    }
    
    const auto& value = properties[field_name];
    
    if (value.is_number_integer()) {
        return static_cast<size_t>(value.get<int64_t>());
    } else if (value.is_number_float()) {
        return static_cast<size_t>(value.get<double>());
    } else if (value.is_string()) {
        try {
            return std::stoull(value.get<std::string>());
        } catch (...) {
            return std::nullopt;
        }
    }
    
    return std::nullopt;
}

double getFieldValueAsDouble(const nlohmann::json& properties,
                           const std::string& field_name,
                           double default_value) {
    if (field_name.empty() || !properties.contains(field_name)) {
        return default_value;
    }
    
    const auto& value = properties[field_name];
    
    if (value.is_number()) {
        return value.get<double>();
    } else if (value.is_string()) {
        try {
            return std::stod(value.get<std::string>());
        } catch (...) {
            return default_value;
        }
    }
    
    return default_value;
}

std::string getFieldValueAsString(const nlohmann::json& properties,
                                const std::string& field_name,
                                const std::string& default_value) {
    if (field_name.empty() || !properties.contains(field_name)) {
        return default_value;
    }
    
    const auto& value = properties[field_name];
    
    if (value.is_string()) {
        return value.get<std::string>();
    } else if (value.is_number()) {
        return std::to_string(value.get<double>());
    } else if (value.is_boolean()) {
        return value.get<bool>() ? "true" : "false";
    }
    
    return default_value;
}

bool getFieldValueAsBool(const nlohmann::json& properties,
                        const std::string& field_name,
                        bool default_value) {
    if (field_name.empty() || !properties.contains(field_name)) {
        return default_value;
    }
    
    const auto& value = properties[field_name];
    
    if (value.is_boolean()) {
        return value.get<bool>();
    } else if (value.is_number_integer()) {
        return value.get<int64_t>() != 0;
    } else if (value.is_string()) {
        const std::string str_val = value.get<std::string>();
        return str_val == "true" || str_val == "1" || str_val == "yes";
    }
    
    return default_value;
}

} // namespace graph
} // namespace adjfind
