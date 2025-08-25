#include "io/point_reader.hpp"
#include <stdexcept>
#include <iostream>
#include <sstream>
#include <algorithm>

namespace adjfind {
namespace io {

PointReader::PointReader(const PointReaderConfig& config)
    : config_(config), coordinate_transformation_(nullptr) {
    initGDAL();
}

PointReader::~PointReader() {
    // Clean up coordinate transformation
    if (coordinate_transformation_) {
        OCTDestroyCoordinateTransformation(coordinate_transformation_);
        coordinate_transformation_ = nullptr;
    }
    
    if (dataset_) {
        GDALDataset* ds = dataset_.release();
        GDALClose(ds);
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
        GDALDataset* ds = dataset_.release();
        GDALClose(ds);
    }

    // Open the dataset
    dataset_.reset(static_cast<GDALDataset*>(
        GDALOpenEx(config_.file_path.c_str(), GDAL_OF_VECTOR, nullptr, nullptr, nullptr)));
    
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
    if (config_.layer_index < 0 || config_.layer_index >= dataset_->GetLayerCount()) {
        std::cerr << "Error: Layer index " << config_.layer_index << " is out of range. Dataset has " 
                  << dataset_->GetLayerCount() << " layers." << std::endl;
        return false;
    }
    
    // Get the specified layer
    OGRLayer* layer = dataset_->GetLayer(config_.layer_index);
    if (!layer) {
        std::cerr << "No layer found at index " << config_.layer_index << " in point dataset" << std::endl;
        return false;
    }

    // Print layer info
    std::cout << "Point layer " << config_.layer_index << " name: " << layer->GetName() << std::endl;
    std::cout << "Point feature count: " << layer->GetFeatureCount() << std::endl;
    
    OGRFeatureDefn* layerDefn = layer->GetLayerDefn();
    if (layerDefn) {
        OGRwkbGeometryType gType = layerDefn->GetGeomType();
        std::cout << "Point geometry type: " << OGRGeometryTypeToName(gType) << std::endl;
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
        
        // Convert to Point
        graph::Point point = convertOGRToPoint(geom);
        
        // Create point feature
        points_.emplace_back(feature_id, point);
        
        OGRFeature::DestroyFeature(feature);
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
        std::cout << "Point dataset is in EPSG:4326 (WGS84)" << std::endl;
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
                char *wkt = nullptr;
                target_srs.exportToWkt(&wkt);
                if (wkt) {
                    coordinate_system_wkt_ = wkt;
                    CPLFree(wkt);
                }
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
            std::cout << "Point dataset coordinate system: EPSG:" << coordinate_system_epsg_ << std::endl;
        } else {
            coordinate_system_epsg_ = -1;
            std::cout << "Point dataset coordinate system: Unknown (WKT: " << coordinate_system_wkt_ << ")" << std::endl;
        }
    }
    
    return true;
}

bool PointReader::transformGeometry(OGRGeometry* geom) const {
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



std::optional<OGRSpatialReference> PointReader::getSpatialRef() const {
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

std::optional<size_t> PointReader::getFieldValueAsSizeT(const OGRFeature* feature,
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

double PointReader::getFieldValueAsDouble(const OGRFeature* feature,
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

graph::Point PointReader::convertOGRToPoint(const OGRGeometry* ogr_geom) const {
    graph::Point point(0.0, 0.0);
    
    if (!ogr_geom) {
        return point;
    }
    
    OGRwkbGeometryType geom_type = wkbFlatten(ogr_geom->getGeometryType());
    
    if (geom_type == wkbPoint) {
        const OGRPoint* ogr_point = static_cast<const OGRPoint*>(ogr_geom);
        double x = ogr_point->getX();
        double y = ogr_point->getY();
        point = graph::Point(x, y);
    } else if (geom_type == wkbMultiPoint) {
        const OGRMultiPoint* ogr_multipoint = static_cast<const OGRMultiPoint*>(ogr_geom);
        int num_points = ogr_multipoint->getNumGeometries();
        
        // For simplicity, take the first point
        if (num_points > 0) {
            const OGRPoint* ogr_point = static_cast<const OGRPoint*>(
                ogr_multipoint->getGeometryRef(0));
            double x = ogr_point->getX();
            double y = ogr_point->getY();
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