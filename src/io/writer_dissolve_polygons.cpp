#include "io/writer_dissolve_polygons.hpp"
#include <gdal.h>
#include <ogr_api.h>
#include <ogr_spatialref.h>
#include <iostream>

namespace adjfind {
namespace io {

namespace {

OGRGeometryH convertPolygonToOGR(const graph::Polygon& polygon) {
    OGRGeometryH ogr_polygon = OGR_G_CreateGeometry(wkbPolygon);
    
    OGRGeometryH exterior_ring = OGR_G_CreateGeometry(wkbLinearRing);
    for (const auto& pt : polygon.outer()) {
        OGR_G_AddPoint_2D(exterior_ring, pt.get<0>(), pt.get<1>());
    }
    OGR_G_AddGeometryDirectly(ogr_polygon, exterior_ring);
    
    for (const auto& inner : polygon.inners()) {
        OGRGeometryH interior_ring = OGR_G_CreateGeometry(wkbLinearRing);
        for (const auto& pt : inner) {
            OGR_G_AddPoint_2D(interior_ring, pt.get<0>(), pt.get<1>());
        }
        OGR_G_AddGeometryDirectly(ogr_polygon, interior_ring);
    }
    
    return ogr_polygon;
}

} // namespace

DissolvePolygonsWriter::DissolvePolygonsWriter() {
    GDALAllRegister();
}

DissolvePolygonsWriter::~DissolvePolygonsWriter() = default;

GDALDatasetH DissolvePolygonsWriter::createGDALDataset(const DissolvePolygonsWriterConfig& config,
                                                      OGRCoordinateTransformationH& coord_trans,
                                                      std::string& output_file_path) {
    coord_trans = nullptr;
    
    bool reprojection_succeeded = false;
    if (config.reproject_to_epsg4326) {
        if (!config.crs_wkt.empty()) {
            OGRSpatialReferenceH source_srs = OSRNewSpatialReference(nullptr);
            OGRSpatialReferenceH target_srs = OSRNewSpatialReference(nullptr);
            
            char* wkt_copy = const_cast<char*>(config.crs_wkt.c_str());
            OSRImportFromWkt(source_srs, &wkt_copy);
            OSRImportFromEPSG(target_srs, 4326);
            
            OSRSetAxisMappingStrategy(source_srs, OAMS_TRADITIONAL_GIS_ORDER);
            OSRSetAxisMappingStrategy(target_srs, OAMS_TRADITIONAL_GIS_ORDER);
            
            coord_trans = OCTNewCoordinateTransformation(source_srs, target_srs);
            if (coord_trans) {
                reprojection_succeeded = true;
            } else {
                std::cerr << "Warning: Failed to create coordinate transformation to EPSG:4326" << std::endl;
                std::cerr << "Output will use original coordinate system" << std::endl;
            }
            
            OSRDestroySpatialReference(source_srs);
            OSRDestroySpatialReference(target_srs);
        } else {
            std::cerr << "Warning: Cannot reproject to EPSG:4326 - no source coordinate system specified" << std::endl;
            std::cerr << "Output will use original coordinate system" << std::endl;
        }
    }
    
    output_file_path = config.output_file_path;
    std::string format = GDALUtils::determineFormatAndModifyPath(output_file_path);
    
    GDALDriverH driver = GDALGetDriverByName(format.c_str());
    if (!driver) {
        last_error_ = "Failed to get GDAL driver for format: " + format;
        if (coord_trans) {
            OCTDestroyCoordinateTransformation(coord_trans);
            coord_trans = nullptr;
        }
        return nullptr;
    }
    
    GDALDatasetH dataset = GDALCreate(driver, output_file_path.c_str(), 0, 0, 0, GDT_Unknown, nullptr);
    if (!dataset) {
        last_error_ = "Failed to create GDAL dataset: " + output_file_path;
        if (coord_trans) {
            OCTDestroyCoordinateTransformation(coord_trans);
            coord_trans = nullptr;
        }
        return nullptr;
    }
    
    OGRSpatialReferenceH layer_srs = nullptr;
    
    if (config.reproject_to_epsg4326 && reprojection_succeeded) {
        layer_srs = OSRNewSpatialReference(nullptr);
        OSRImportFromEPSG(layer_srs, 4326);
        OSRSetAxisMappingStrategy(layer_srs, OAMS_TRADITIONAL_GIS_ORDER);
    } else if (!config.crs_wkt.empty()) {
        layer_srs = OSRNewSpatialReference(nullptr);
        char* wkt_copy = const_cast<char*>(config.crs_wkt.c_str());
        if (OSRImportFromWkt(layer_srs, &wkt_copy) != OGRERR_NONE) {
            OSRDestroySpatialReference(layer_srs);
            layer_srs = nullptr;
        } else {
            OSRSetAxisMappingStrategy(layer_srs, OAMS_TRADITIONAL_GIS_ORDER);
        }
    }
    
    std::string layer_name = "dissolved_polygons";
    OGRLayerH layer = GDALDatasetCreateLayer(dataset, layer_name.c_str(), layer_srs, wkbPolygon, nullptr);
    if (!layer) {
        last_error_ = "Failed to create layer: " + layer_name;
        if (coord_trans) {
            OCTDestroyCoordinateTransformation(coord_trans);
            coord_trans = nullptr;
        }
        if (layer_srs) {
            OSRDestroySpatialReference(layer_srs);
        }
        GDALClose(dataset);
        return nullptr;
    }
    
    OGRFieldDefnH id_field = OGR_Fld_Create("id", OFTInteger64);
    OGR_L_CreateField(layer, id_field, 1);
    OGR_Fld_Destroy(id_field);
    
    if (layer_srs) {
        OSRDestroySpatialReference(layer_srs);
    }
    return dataset;
}

bool DissolvePolygonsWriter::writeFeature(OGRLayerH layer,
                                          OGRCoordinateTransformationH coord_trans,
                                          const graph::Polygon& polygon,
                                          size_t feature_id) {
    OGRFeatureDefnH layer_defn = OGR_L_GetLayerDefn(layer);
    OGRFeatureH feature = OGR_F_Create(layer_defn);
    if (!feature) {
        last_error_ = "Failed to create feature";
        return false;
    }
    
    OGRGeometryH ogr_polygon = convertPolygonToOGR(polygon);
    
    if (coord_trans) {
        if (OGR_G_Transform(ogr_polygon, coord_trans) != OGRERR_NONE) {
            std::cerr << "Warning: Failed to reproject geometry to EPSG:4326" << std::endl;
        }
    }
    
    OGR_F_SetGeometry(feature, ogr_polygon);
    OGR_F_SetFieldInteger64(feature, OGR_F_GetFieldIndex(feature, "id"), static_cast<GIntBig>(feature_id));
    
    if (OGR_L_CreateFeature(layer, feature) != OGRERR_NONE) {
        last_error_ = "Failed to create feature in layer";
        OGR_F_Destroy(feature);
        return false;
    }
    
    OGR_F_Destroy(feature);
    return true;
}

bool DissolvePolygonsWriter::writeDissolvedPolygons(const DissolvePolygonsWriterConfig& config,
                                                    const std::vector<graph::Polygon>& polygons) {
    clearError();
    
    if (polygons.empty()) {
        last_error_ = "No dissolved polygons to write";
        return false;
    }
    
    OGRCoordinateTransformationH coord_trans = nullptr;
    std::string output_file_path;
    GDALDatasetH dataset = createGDALDataset(config, coord_trans, output_file_path);
    if (!dataset) {
        return false;
    }
    
    OGRLayerH layer = GDALDatasetGetLayer(dataset, 0);
    if (!layer) {
        last_error_ = "Failed to get layer from dataset";
        if (coord_trans) {
            OCTDestroyCoordinateTransformation(coord_trans);
        }
        GDALClose(dataset);
        return false;
    }
    
    size_t feature_id = 0;
    for (const auto& polygon : polygons) {
        if (!writeFeature(layer, coord_trans, polygon, feature_id)) {
            if (coord_trans) {
                OCTDestroyCoordinateTransformation(coord_trans);
            }
            GDALClose(dataset);
            return false;
        }
        feature_id++;
    }
    
    if (coord_trans) {
        OCTDestroyCoordinateTransformation(coord_trans);
    }
    
    GDALClose(dataset);
    
    std::cout << "Successfully wrote " << polygons.size() << " dissolved polygons to: " << output_file_path << std::endl;
    return true;
}

} // namespace io
} // namespace adjfind
