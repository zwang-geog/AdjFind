#include "io/point_reader.hpp"
#include "io/coordinate_system_utils.hpp"
#include <stdexcept>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <gdal.h>
#include <ogr_api.h>
#include <boost/geometry/index/rtree.hpp>

namespace adjfind {
namespace io {

namespace bgi = boost::geometry::index;

PointReader::PointReader(const PointReaderConfig& config)
    : config_(config), dataset_(nullptr), coordinate_transformation_(nullptr) {
    initGDAL();
}

PointReader::~PointReader() {
    // Clean up coordinate transformation
    if (coordinate_transformation_) {
        OCTDestroyCoordinateTransformation(coordinate_transformation_);
        coordinate_transformation_ = nullptr;
    }
    
    if (dataset_) {
        GDALClose(dataset_);
        dataset_ = nullptr;
    }
}

void PointReader::initGDAL() {
    GDALAllRegister();
}

bool PointReader::read() {
    // Clear any existing features
    points_.clear();
    
    // Close any existing dataset
    if (dataset_) {
        GDALClose(dataset_);
        dataset_ = nullptr;
    }

    // Open the dataset
    dataset_ = GDALOpenEx(config_.file_path.c_str(), GDAL_OF_VECTOR, nullptr, nullptr, nullptr);
    
    if (!dataset_) {
        std::cerr << "Failed to open point file: " << config_.file_path << std::endl;
        return false;
    }

    // Handle coordinate system
    if (!handleCoordinateSystem()) {
        return false;
    }

    return readFeatures();
}

bool PointReader::readFeatures() {
    points_.clear();
    
    // Check if layer index is valid
    int layer_count = GDALDatasetGetLayerCount(dataset_);
    if (config_.layer_index < 0 || config_.layer_index >= layer_count) {
        std::cerr << "Error: Layer index " << config_.layer_index << " is out of range. Dataset has " 
                  << layer_count << " layers." << std::endl;
        return false;
    }
    
    // Get the specified layer
    OGRLayerH layer = GDALDatasetGetLayer(dataset_, config_.layer_index);
    if (!layer) {
        std::cerr << "No layer found at index " << config_.layer_index << " in point dataset" << std::endl;
        return false;
    }

    // Print layer info
    std::cout << "Point layer " << config_.layer_index << " name: " << OGR_L_GetName(layer) << std::endl;
    std::cout << "Point feature count: " << OGR_L_GetFeatureCount(layer, 1) << std::endl;
    
    OGRFeatureDefnH layerDefn = OGR_L_GetLayerDefn(layer);
    if (layerDefn) {
        OGRwkbGeometryType gType = OGR_FD_GetGeomType(layerDefn);
        std::cout << "Point geometry type: " << OGRGeometryTypeToName(gType) << std::endl;
    }

    // Check if the specified ID field exists
    bool use_ogr_fid = false;
    if (!config_.id_field.empty()) {
        int field_idx = OGR_FD_GetFieldIndex(layerDefn, config_.id_field.c_str());
        if (field_idx < 0) {
            std::cout << "Warning: Specified ID field '" << config_.id_field 
                     << "' not found. Falling back to OGR FID." << std::endl;
            use_ogr_fid = true;
        }
    } else {
        use_ogr_fid = true;
    }

    // Reset reading
    OGR_L_ResetReading(layer);

    // Read all features
    OGRFeatureH feature = nullptr;
    size_t feature_idx = 0;
    
    while ((feature = OGR_L_GetNextFeature(layer)) != nullptr) {
        // Get feature ID
        size_t feature_id;
        if (use_ogr_fid) {
            feature_id = OGR_F_GetFID(feature);
        } else {
            auto id_opt = getFieldValueAsSizeT(feature, config_.id_field);
            if (!id_opt) {
                std::cerr << "Warning: Could not get ID for feature " << feature_idx 
                         << ". Skipping." << std::endl;
                OGR_F_Destroy(feature);
                continue;
            }
            feature_id = *id_opt;
        }
        
        // Get geometry
        OGRGeometryH geom = OGR_F_GetGeometryRef(feature);
        if (!geom) {
            std::cerr << "Warning: No geometry for feature " << feature_idx 
                     << ". Skipping." << std::endl;
            OGR_F_Destroy(feature);
            continue;
        }
        
        // Transform geometry if needed
        if (!transformGeometry(geom)) {
            std::cerr << "Warning: Failed to transform geometry for feature " << feature_idx 
                     << ". Skipping." << std::endl;
            OGR_F_Destroy(feature);
            continue;
        }
        
        // Convert to Point
        graph::Point point = convertOGRToPoint(geom);
        
        // Create point feature
        points_.emplace_back(feature_id, point);
        
        OGR_F_Destroy(feature);
        feature_idx++;
    }
    
    std::cout << "Successfully read " << points_.size() << " point features" << std::endl;
    
    // Build spatial index
    buildSpatialIndex();
    
    return true;
}

bool PointReader::handleCoordinateSystem() {
    if (!dataset_) {
        return false;
    }
    
    // Get coordinate system WKT from the specified layer
    OGRLayerH layer = GDALDatasetGetLayer(dataset_, config_.layer_index);
    if (!layer) {
        std::cerr << "Error: Could not get layer for coordinate system detection" << std::endl;
        return false;
    }
    
    OGRSpatialReferenceH layer_srs = OGR_L_GetSpatialRef(layer);
    if (layer_srs) {
        char* wkt = nullptr;
        OSRExportToWkt(layer_srs, &wkt);
        if (wkt) {
            coordinate_system_wkt_ = wkt;
            CPLFree(wkt);
        }
    } else {
        coordinate_system_wkt_ = "";
    }
    
    // Get spatial reference
    OGRSpatialReferenceH spatial_ref = OSRNewSpatialReference(nullptr);
    if (!coordinate_system_wkt_.empty()) {
        char* wkt_copy = const_cast<char*>(coordinate_system_wkt_.c_str());
        if (OSRImportFromWkt(spatial_ref, &wkt_copy) != OGRERR_NONE) {
            std::cerr << "Warning: Failed to parse coordinate system WKT" << std::endl;
            coordinate_system_epsg_ = -1;
            OSRDestroySpatialReference(spatial_ref);
            return true; // Continue anyway
        }
    } else {
        // No coordinate system info
        coordinate_system_epsg_ = -1;
        OSRDestroySpatialReference(spatial_ref);
        return true;
    }
    
    // Check if it's EPSG:4326
    bool is_epsg4326 = false;
    const char* authority_name = OSRGetAuthorityName(spatial_ref, nullptr);
    const char* authority_code = OSRGetAuthorityCode(spatial_ref, nullptr);
    
    if (authority_name && authority_code && std::string(authority_name) == "EPSG") {
        int epsg_code = std::stoi(authority_code);
        if (epsg_code == 4326) {
            is_epsg4326 = true;
        }
    }
    
    if (is_epsg4326) {
        std::cout << "Point dataset is in EPSG:4326 (WGS84)" << std::endl;
        coordinate_system_epsg_ = 4326;
        
        // Get dataset center for UTM zone determination
        double center_x, center_y;
        if (CoordinateSystemUtils::getDatasetCenter(dataset_, center_x, center_y)) {
            int utm_epsg = CoordinateSystemUtils::determineUTMEPSG(center_x, center_y);
            std::cout << "Dataset center: (" << center_x << ", " << center_y << ")" << std::endl;
            std::cout << "Recommended UTM zone: EPSG:" << utm_epsg << std::endl;
            
            // Create coordinate transformation for UTM reprojection
            coordinate_transformation_ = CoordinateSystemUtils::createUTMTransformation(spatial_ref, utm_epsg);
            if (coordinate_transformation_) {
                // Update coordinate system info
                coordinate_system_epsg_ = utm_epsg;
                OGRSpatialReferenceH target_srs = OSRNewSpatialReference(nullptr);
                OSRImportFromEPSG(target_srs, utm_epsg);
                char *wkt = nullptr;
                OSRExportToWkt(target_srs, &wkt);
                if (wkt) {
                    coordinate_system_wkt_ = wkt;
                    CPLFree(wkt);
                }
                OSRDestroySpatialReference(target_srs);
                std::cout << "Successfully created UTM transformation to EPSG:" << utm_epsg << std::endl;
            } else {
                std::cerr << "ERROR: Failed to create coordinate transformation for EPSG:4326 to UTM zone EPSG:" << utm_epsg << std::endl;
                std::cerr << "Boost Geometry requires cartesian coordinates. Aborting program." << std::endl;
                OSRDestroySpatialReference(spatial_ref);
                return false;
            }
        } else {
            std::cerr << "ERROR: Could not determine dataset center for UTM zone calculation" << std::endl;
            std::cerr << "Boost Geometry requires cartesian coordinates. Aborting program." << std::endl;
            OSRDestroySpatialReference(spatial_ref);
            return false;
        }
    } else {
        // Not EPSG:4326, get EPSG code if available
        if (authority_name && authority_code && std::string(authority_name) == "EPSG") {
            coordinate_system_epsg_ = std::stoi(authority_code);
            std::cout << "Point dataset coordinate system: EPSG:" << coordinate_system_epsg_ << std::endl;
        } else {
            coordinate_system_epsg_ = -1;
            std::cout << "Point dataset coordinate system: Unknown (WKT: " << coordinate_system_wkt_ << ")" << std::endl;
        }
    }
    
    OSRDestroySpatialReference(spatial_ref);
    return true;
}



bool PointReader::transformGeometry(OGRGeometryH geom) const {
    if (!coordinate_transformation_ || !geom) {
        return true; // No transformation needed or no geometry
    }
    
    // Transform the geometry in-place
    if (OGR_G_Transform(geom, coordinate_transformation_) != OGRERR_NONE) {
        std::cerr << "Warning: Failed to transform geometry coordinates" << std::endl;
        return false;
    }
    
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

std::vector<size_t> PointReader::findPointsIntersectBoundingBox(const graph::Box& box) const {
    std::vector<size_t> point_indices;
    if (rtree_.empty()) {
        return point_indices;
    }
    std::vector<graph::PointRTreeValue> query_results;
    rtree_.query(bgi::intersects(box), std::back_inserter(query_results));
    point_indices.reserve(query_results.size());
    for (const auto& v : query_results) {
        point_indices.push_back(v.feature_index);
    }
    return point_indices;
}

const graph::PointFeature& PointReader::getPointFeature(size_t index) const {
    if (index >= points_.size()) {
        throw std::out_of_range("Point feature index out of range");
    }
    return points_[index];
}

std::vector<graph::PointFeature> PointReader::movePoints() {
    std::vector<graph::PointFeature> moved_points = std::move(points_);
    points_.clear();
    // Clear r-tree since points are moved out
    rtree_.clear();
    return moved_points;
}


std::optional<OGRSpatialReferenceH> PointReader::getSpatialRef() const {
    if (!dataset_) {
        return std::nullopt;
    }
    
    OGRLayerH layer = GDALDatasetGetLayer(dataset_, 0);
    if (!layer) {
        return std::nullopt;
    }
    
    OGRSpatialReferenceH srs = OGR_L_GetSpatialRef(layer);
    if (srs) {
        return srs;
    }
    
    return std::nullopt;
}

std::optional<size_t> PointReader::getFieldValueAsSizeT(const OGRFeatureH feature,
                                                       const std::string& field_name) const {
    if (!feature || field_name.empty()) {
        return std::nullopt;
    }
    
    int field_idx = OGR_F_GetFieldIndex(feature, field_name.c_str());
    if (field_idx < 0) {
        return std::nullopt;
    }
    
    if (OGR_F_IsFieldSetAndNotNull(feature, field_idx)) {
        OGRFieldDefnH field_defn = OGR_F_GetFieldDefnRef(feature, field_idx);
        OGRFieldType field_type = OGR_Fld_GetType(field_defn);
        
        switch (field_type) {
            case OFTInteger:
                return static_cast<size_t>(OGR_F_GetFieldAsInteger(feature, field_idx));
            case OFTInteger64:
                return static_cast<size_t>(OGR_F_GetFieldAsInteger64(feature, field_idx));
            case OFTReal:
                return static_cast<size_t>(OGR_F_GetFieldAsDouble(feature, field_idx));
            case OFTString: {
                const char* str_val = OGR_F_GetFieldAsString(feature, field_idx);
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

double PointReader::getFieldValueAsDouble(const OGRFeatureH feature,
                                        const std::string& field_name,
                                        double default_value) const {
    if (!feature || field_name.empty()) {
        return default_value;
    }
    
    int field_idx = OGR_F_GetFieldIndex(feature, field_name.c_str());
    if (field_idx < 0) {
        return default_value;
    }
    
    if (OGR_F_IsFieldSetAndNotNull(feature, field_idx)) {
        OGRFieldDefnH field_defn = OGR_F_GetFieldDefnRef(feature, field_idx);
        OGRFieldType field_type = OGR_Fld_GetType(field_defn);
        
        switch (field_type) {
            case OFTInteger:
                return static_cast<double>(OGR_F_GetFieldAsInteger(feature, field_idx));
            case OFTInteger64:
                return static_cast<double>(OGR_F_GetFieldAsInteger64(feature, field_idx));
            case OFTReal:
                return OGR_F_GetFieldAsDouble(feature, field_idx);
            case OFTString: {
                const char* str_val = OGR_F_GetFieldAsString(feature, field_idx);
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

graph::Point PointReader::convertOGRToPoint(const OGRGeometryH ogr_geom) const {
    graph::Point point(0.0, 0.0);
    
    if (!ogr_geom) {
        return point;
    }
    
    OGRwkbGeometryType geom_type = wkbFlatten(OGR_G_GetGeometryType(ogr_geom));
    
    if (geom_type == wkbPoint) {
        double x, y, z;
        OGR_G_GetPoint(ogr_geom, 0, &x, &y, &z);
        point = graph::Point(x, y);
    } else if (geom_type == wkbMultiPoint) {
        int num_points = OGR_G_GetGeometryCount(ogr_geom);
        
        // For simplicity, take the first point
        if (num_points > 0) {
            OGRGeometryH point_geom = OGR_G_GetGeometryRef(ogr_geom, 0);
            double x, y, z;
            OGR_G_GetPoint(point_geom, 0, &x, &y, &z);
            point = graph::Point(x, y);
        }
    }
    
    return point;
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