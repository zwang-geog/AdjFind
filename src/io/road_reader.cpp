#include "io/road_reader.hpp"
#include <stdexcept>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <limits>

namespace adjfind {
namespace io {

RoadReader::RoadReader(const RoadReaderConfig& config)
    : config_(config), coordinate_transformation_(nullptr) {
    initGDAL();
}

RoadReader::~RoadReader() {
    // Clear features first
    roads_.clear();
    
    // Clean up coordinate transformation
    if (coordinate_transformation_) {
        OCTDestroyCoordinateTransformation(coordinate_transformation_);
        coordinate_transformation_ = nullptr;
    }
    
    // Then close dataset
    if (dataset_) {
        // Use raw pointer as GDALClose takes ownership
        GDALDataset* ds = dataset_.release();
        GDALClose(ds);
    }
}

void RoadReader::initGDAL() {
    GDALAllRegister();
}

bool RoadReader::read() {
    // Clear any existing features
    roads_.clear();
    
    // Close any existing dataset
    if (dataset_) {
        GDALDataset* ds = dataset_.release();
        GDALClose(ds);
    }

    // Open the dataset
    dataset_.reset(static_cast<GDALDataset*>(
        GDALOpenEx(config_.file_path.c_str(), GDAL_OF_VECTOR, nullptr, nullptr, nullptr)));
    
    if (!dataset_) {
        std::cerr << "Failed to open road file: " << config_.file_path << std::endl;
        return false;
    }

    // Handle coordinate system
    if (!handleCoordinateSystem()) {
        return false;
    }

    return readFeatures();
}

bool RoadReader::readFeatures() {
    roads_.clear();
    
    // Check if layer index is valid
    if (config_.layer_index < 0 || config_.layer_index >= dataset_->GetLayerCount()) {
        std::cerr << "Error: Layer index " << config_.layer_index << " is out of range. Dataset has " 
                  << dataset_->GetLayerCount() << " layers." << std::endl;
        return false;
    }
    
    // Get the specified layer
    OGRLayer* layer = dataset_->GetLayer(config_.layer_index);
    if (!layer) {
        std::cerr << "No layer found at index " << config_.layer_index << " in road dataset" << std::endl;
        return false;
    }

    // Print layer info
    std::cout << "Road layer " << config_.layer_index << " name: " << layer->GetName() << std::endl;
    std::cout << "Road feature count: " << layer->GetFeatureCount() << std::endl;
    
    OGRFeatureDefn* layerDefn = layer->GetLayerDefn();
    if (layerDefn) {
        OGRwkbGeometryType gType = layerDefn->GetGeomType();
        std::cout << "Road geometry type: " << OGRGeometryTypeToName(gType) << std::endl;
    }

    // Check if the specified ID field exists
    bool use_ogr_fid = false;
    if (!config_.id_field.empty()) {
        int field_idx = layer->GetLayerDefn()->GetFieldIndex(config_.id_field.c_str());
        if (field_idx < 0) {
            std::cout << "Warning: Specified ID field '" << config_.id_field 
                     << "' not found. Falling back to OGR FID." << std::endl;
            use_ogr_fid = true;
        }
    } else {
        use_ogr_fid = true;
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
    if (has_length_field) {
        int field_idx = layer->GetLayerDefn()->GetFieldIndex(config_.length_field.c_str());
        if (field_idx < 0) {
            std::cout << "Warning: Specified length field '" << config_.length_field 
                     << "' not found. Will compute length from geometry." << std::endl;
            has_length_field = false;
        }
    }

    // Reset reading
    layer->ResetReading();

    // Read all features
    OGRFeature* feature = nullptr;
    size_t feature_idx = 0;
    
    while ((feature = layer->GetNextFeature()) != nullptr) {
        // Get feature ID
        size_t feature_id;
        if (use_ogr_fid) {
            feature_id = feature->GetFID();
        } else {
            auto id_opt = getFieldValueAsSizeT(feature, config_.id_field);
            if (!id_opt) {
                std::cerr << "Warning: Could not get ID for feature " << feature_idx 
                         << ". Skipping." << std::endl;
                OGRFeature::DestroyFeature(feature);
                continue;
            }
            feature_id = *id_opt;
        }
        
        // Get geometry
        OGRGeometry* geom = feature->GetGeometryRef();
        if (!geom) {
            std::cerr << "Warning: No geometry for feature " << feature_idx 
                     << ". Skipping." << std::endl;
            OGRFeature::DestroyFeature(feature);
            continue;
        }
        
        // Transform geometry if needed
        if (!transformGeometry(geom)) {
            std::cerr << "Warning: Failed to transform geometry for feature " << feature_idx 
                     << ". Skipping." << std::endl;
            OGRFeature::DestroyFeature(feature);
            continue;
        }
        
        // Convert to LineString
        graph::LineString linestring = convertOGRToLineString(geom);
        
        // Get length
        double length;
        if (has_length_field) {
            length = getFieldValueAsDouble(feature, config_.length_field, 0.0);
            if (length <= 0.0) {
                length = graph::bg::length(linestring);
            }
        } else {
            length = graph::bg::length(linestring);
        }
        
        // Get z-levels
        double from_z = has_from_z ? 
            getFieldValueAsDouble(feature, config_.from_z_field, config_.default_from_z) : 
            config_.default_from_z;
        double to_z = has_to_z ? 
            getFieldValueAsDouble(feature, config_.to_z_field, config_.default_to_z) : 
            config_.default_to_z;
        
        // Create road feature
        roads_.emplace_back(feature_id, linestring, length, from_z, to_z);
        
        OGRFeature::DestroyFeature(feature);
        feature_idx++;
    }
    
    std::cout << "Successfully read " << roads_.size() << " road features" << std::endl;
    
    // Build spatial index
    buildSpatialIndex();
    
    return true;
}

bool RoadReader::handleCoordinateSystem() {
    if (!dataset_) {
        return false;
    }
    
    // Get coordinate system WKT from the specified layer
    coordinate_system_wkt_ = CoordinateSystemUtils::getCoordinateSystemWKT(dataset_.get(), config_.layer_index);
    
    // Get spatial reference
    OGRSpatialReference spatial_ref;
    if (!coordinate_system_wkt_.empty()) {
        if (spatial_ref.importFromWkt(coordinate_system_wkt_.c_str()) != OGRERR_NONE) {
            std::cerr << "Warning: Failed to parse coordinate system WKT" << std::endl;
            coordinate_system_epsg_ = -1;
            return true; // Continue anyway
        }
    } else {
        // No coordinate system info
        coordinate_system_epsg_ = -1;
        return true;
    }
    
    // Check if it's EPSG:4326
    if (CoordinateSystemUtils::isEPSG4326(&spatial_ref)) {
        std::cout << "Road dataset is in EPSG:4326 (WGS84)" << std::endl;
        coordinate_system_epsg_ = 4326;
        
        // Get dataset center for UTM zone determination
        double center_x, center_y;
        if (CoordinateSystemUtils::getDatasetCenter(dataset_.get(), center_x, center_y)) {
            int utm_epsg = CoordinateSystemUtils::determineUTMEPSG(center_x, center_y);
            std::cout << "Dataset center: (" << center_x << ", " << center_y << ")" << std::endl;
            std::cout << "Recommended UTM zone: EPSG:" << utm_epsg << std::endl;
            
            // Create coordinate transformation for UTM reprojection
            coordinate_transformation_ = CoordinateSystemUtils::createUTMTransformation(&spatial_ref, utm_epsg);
            if (coordinate_transformation_) {
                // Update coordinate system info
                coordinate_system_epsg_ = utm_epsg;
                OGRSpatialReference target_srs;
                target_srs.importFromEPSG(utm_epsg);
                coordinate_system_wkt_ = target_srs.exportToWkt();
                std::cout << "Successfully created UTM transformation to EPSG:" << utm_epsg << std::endl;
            } else {
                std::cerr << "ERROR: Failed to create coordinate transformation for EPSG:4326 to UTM zone EPSG:" << utm_epsg << std::endl;
                std::cerr << "Boost Geometry requires cartesian coordinates. Aborting program." << std::endl;
                return false;
            }
        } else {
            std::cerr << "ERROR: Could not determine dataset center for UTM zone calculation" << std::endl;
            std::cerr << "Boost Geometry requires cartesian coordinates. Aborting program." << std::endl;
            return false;
        }
    } else {
        // Not EPSG:4326, get EPSG code if available
        const char* authority_name = spatial_ref.GetAuthorityName(nullptr);
        const char* authority_code = spatial_ref.GetAuthorityCode(nullptr);
        
        if (authority_name && authority_code && std::string(authority_name) == "EPSG") {
            coordinate_system_epsg_ = std::stoi(authority_code);
            std::cout << "Road dataset coordinate system: EPSG:" << coordinate_system_epsg_ << std::endl;
        } else {
            coordinate_system_epsg_ = -1;
            std::cout << "Road dataset coordinate system: Unknown (WKT: " << coordinate_system_wkt_ << ")" << std::endl;
        }
    }
    
    return true;
}

bool RoadReader::transformGeometry(OGRGeometry* geom) const {
    if (!coordinate_transformation_ || !geom) {
        return true; // No transformation needed or no geometry
    }
    
    // Transform the geometry in-place
    if (!geom->transform(coordinate_transformation_)) {
        std::cerr << "Warning: Failed to transform geometry coordinates" << std::endl;
        return false;
    }
    
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



std::optional<OGRSpatialReference> RoadReader::getSpatialRef() const {
    if (!dataset_) {
        return std::nullopt;
    }
    
    OGRLayer* layer = dataset_->GetLayer(0);
    if (!layer) {
        return std::nullopt;
    }
    
    OGRSpatialReference* srs = layer->GetSpatialRef();
    if (srs) {
        return *srs;
    }
    
    return std::nullopt;
}

std::optional<size_t> RoadReader::getFieldValueAsSizeT(const OGRFeature* feature,
                                                      const std::string& field_name) const {
    if (!feature || field_name.empty()) {
        return std::nullopt;
    }
    
    int field_idx = feature->GetFieldIndex(field_name.c_str());
    if (field_idx < 0) {
        return std::nullopt;
    }
    
    if (feature->IsFieldSetAndNotNull(field_idx)) {
        OGRFieldType field_type = feature->GetFieldDefnRef(field_idx)->GetType();
        
        switch (field_type) {
            case OFTInteger:
                return static_cast<size_t>(feature->GetFieldAsInteger(field_idx));
            case OFTInteger64:
                return static_cast<size_t>(feature->GetFieldAsInteger64(field_idx));
            case OFTReal:
                return static_cast<size_t>(feature->GetFieldAsDouble(field_idx));
            case OFTString: {
                const char* str_val = feature->GetFieldAsString(field_idx);
                if (str_val) {
                    try {
                        return std::stoull(str_val);
                    } catch (...) {
                        return std::nullopt;
                    }
                }
                return std::nullopt;
            }
            default:
                return std::nullopt;
        }
    }
    
    return std::nullopt;
}

double RoadReader::getFieldValueAsDouble(const OGRFeature* feature,
                                       const std::string& field_name,
                                       double default_value) const {
    if (!feature || field_name.empty()) {
        return default_value;
    }
    
    int field_idx = feature->GetFieldIndex(field_name.c_str());
    if (field_idx < 0) {
        return default_value;
    }
    
    if (feature->IsFieldSetAndNotNull(field_idx)) {
        OGRFieldType field_type = feature->GetFieldDefnRef(field_idx)->GetType();
        
        switch (field_type) {
            case OFTInteger:
                return static_cast<double>(feature->GetFieldAsInteger(field_idx));
            case OFTInteger64:
                return static_cast<double>(feature->GetFieldAsInteger64(field_idx));
            case OFTReal:
                return feature->GetFieldAsDouble(field_idx);
            case OFTString: {
                const char* str_val = feature->GetFieldAsString(field_idx);
                if (str_val) {
                    try {
                        return std::stod(str_val);
                    } catch (...) {
                        return default_value;
                    }
                }
                return default_value;
            }
            default:
                return default_value;
        }
    }
    
    return default_value;
}

graph::LineString RoadReader::convertOGRToLineString(const OGRGeometry* ogr_geom) const {
    graph::LineString linestring;
    
    if (!ogr_geom) {
        return linestring;
    }
    
    OGRwkbGeometryType geom_type = wkbFlatten(ogr_geom->getGeometryType());
    
    if (geom_type == wkbLineString) {
        const OGRLineString* ogr_linestring = static_cast<const OGRLineString*>(ogr_geom);
        int num_points = ogr_linestring->getNumPoints();
        
        for (int i = 0; i < num_points; ++i) {
            double x = ogr_linestring->getX(i);
            double y = ogr_linestring->getY(i);
            linestring.push_back(graph::Point(x, y));
        }
    } else if (geom_type == wkbMultiLineString) {
        const OGRMultiLineString* ogr_multilinestring = static_cast<const OGRMultiLineString*>(ogr_geom);
        int num_linestrings = ogr_multilinestring->getNumGeometries();
        
        // For simplicity, take the first linestring
        if (num_linestrings > 0) {
            const OGRLineString* ogr_linestring = static_cast<const OGRLineString*>(
                ogr_multilinestring->getGeometryRef(0));
            int num_points = ogr_linestring->getNumPoints();
            
            for (int i = 0; i < num_points; ++i) {
                double x = ogr_linestring->getX(i);
                double y = ogr_linestring->getY(i);
                linestring.push_back(graph::Point(x, y));
            }
        }
    }
    
    return linestring;
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