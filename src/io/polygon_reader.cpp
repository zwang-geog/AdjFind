#include "io/polygon_reader.hpp"
#include "io/coordinate_system_utils.hpp"
#include "io/point_reader.hpp"
#include "graph/adj_graph.hpp"
#include <gdal.h>
#include <ogr_api.h>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <unordered_set>
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/geometry/strategies/buffer.hpp>
#include <boost/geometry/geometries/box.hpp>

namespace adjfind {
namespace io {

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

PolygonReader::PolygonReader(const PolygonReaderConfig& config)
    : config_(config), dataset_(nullptr), coordinate_transformation_(nullptr), coordinate_system_epsg_(-1) {
    initGDAL();
}

PolygonReader::~PolygonReader() {
    if (coordinate_transformation_) {
        OCTDestroyCoordinateTransformation(coordinate_transformation_);
    }
    
    if (dataset_) {
        GDALClose(dataset_);
        dataset_ = nullptr;
    }
}

bool PolygonReader::read() {
    // Clear any existing features
    polygons_.clear();
    
    // Close any existing dataset
    if (dataset_) {
        GDALClose(dataset_);
        dataset_ = nullptr;
    }

    // Open the dataset
    dataset_ = GDALOpenEx(config_.file_path.c_str(), GDAL_OF_VECTOR, nullptr, nullptr, nullptr);
    
    if (!dataset_) {
        std::cerr << "Failed to open polygon file: " << config_.file_path << std::endl;
        return false;
    }
    
    if (!handleCoordinateSystem()) {
        std::cerr << "Error: Failed to handle coordinate system" << std::endl;
        return false;
    }
    
    if (!readFeatures()) {
        std::cerr << "Error: Failed to read features" << std::endl;
        return false;
    }
    
    return true;
}

const graph::PolygonFeature& PolygonReader::getPolygonFeature(size_t index) const {
    if (index >= polygons_.size()) {
        throw std::out_of_range("Polygon index out of range");
    }
    return polygons_[index];
}

std::optional<OGRSpatialReferenceH> PolygonReader::getSpatialRef() const {
    if (!dataset_) {
        return std::nullopt;
    }
    
    OGRLayerH layer = GDALDatasetGetLayer(dataset_, config_.layer_index);
    if (!layer) {
        return std::nullopt;
    }
    
    OGRSpatialReferenceH spatial_ref = OGR_L_GetSpatialRef(layer);
    if (!spatial_ref) {
        return std::nullopt;
    }
    
    // Clone the spatial reference
    OGRSpatialReferenceH cloned_ref = OSRClone(spatial_ref);
    return std::optional<OGRSpatialReferenceH>(cloned_ref);
}

void PolygonReader::populateSnappableRoadIds(graph::AdjGraph& adj_graph) {
    // Spatial index is already built in readFeatures()
    // Just populate snappable road IDs for polygons that don't have them
    // If snappable road IDs are already populated, map them to edge indices

    if (!config_.snappable_ids_field.empty()) {
        adj_graph.buildFeatureIdMapping();
    }

    for (auto& polygon : polygons_) {
        // Even if obstacle-only, we still need to populate snappable road IDs because query will be made for obstacle as well
        
        if (polygon.snappable_ids.empty()) {
            std::vector<std::pair<std::optional<size_t>, graph::Point>> road_edge_index_vector; // Expand snappable_road_ids for later sectional search
            // Find nearest road edge for each point in the polygon's outer ring
            std::unordered_set<size_t> unique_road_ids;
            for (const auto& point : polygon.outer_ring) {
                auto nearest_edge = adj_graph.findNearestEdge(point);
                road_edge_index_vector.push_back(std::make_pair(nearest_edge, point));
                if (nearest_edge.has_value()) {
                    unique_road_ids.insert(nearest_edge.value());
                }
            }
            
            // Convert to vector and assign
            polygon.snappable_ids.assign(unique_road_ids.begin(), unique_road_ids.end());

            // Loop through polygon.snappable_road_ids to obtain a set of unique nearest_point_vertex_position_index
            std::unordered_set<size_t> unique_nearest_point_vertex_position_indices;
            for (size_t edge_index : polygon.snappable_ids) {
                // Get the edge directly using the edge index
                graph::Edge edge = adj_graph.getEdge(edge_index);
                if (edge.nearest_point_vertex_position_index != std::numeric_limits<size_t>::max()) {
                    unique_nearest_point_vertex_position_indices.insert(edge.nearest_point_vertex_position_index);
                }
            }

            // Loop through the road_edge_index_vector starting from 0
            for (size_t i = 0; i < road_edge_index_vector.size(); ++i) {
                if (road_edge_index_vector[i].first.has_value() && road_edge_index_vector[(i + 1) % road_edge_index_vector.size()].first.has_value()) {
                    size_t edge_index_1 = road_edge_index_vector[i].first.value();
                    size_t edge_index_2 = road_edge_index_vector[(i + 1) % road_edge_index_vector.size()].first.value();

                    // Criteria 1: Adjacent sanpped edge must be different
                    if (edge_index_1 != edge_index_2) {
                        graph::Edge edge_1 = adj_graph.getEdge(edge_index_1);
                        graph::Edge edge_2 = adj_graph.getEdge(edge_index_2);

                        // Criteria 2: Closest point vertex must be different
                        // if (edge_1.nearest_point_vertex_position_index == edge_2.nearest_point_vertex_position_index) {
                        //     continue;
                        // }

                        // Criteria 3: The snapped edges must not share the same endpoint vertex
                        if (edge_1.from_vertex == edge_2.to_vertex || edge_1.to_vertex == edge_2.from_vertex) {
                            continue;
                        }

                        // Criteria 4: The polygon boundary segment length must be greater than the minimum polygon boundary segment length for nearest road edge detection
                        if (bg::distance(road_edge_index_vector[i].second, road_edge_index_vector[(i + 1) % road_edge_index_vector.size()].second) <= (getMinPolygonBoundarySegmentLengthForNearestRoadEdgeDetection() + 1.0)) {
                            continue;
                        }

                        graph::MultiPoint polygon_boundary_segment_interpolated_points;
                        graph::LineString polygon_boundary_segment_line;
                        polygon_boundary_segment_line.push_back(road_edge_index_vector[i].second);
                        polygon_boundary_segment_line.push_back(road_edge_index_vector[(i + 1) % road_edge_index_vector.size()].second);
                        double interpolation_distance = std::round(getMinPolygonBoundarySegmentLengthForNearestRoadEdgeDetection());

                        bg::line_interpolate(polygon_boundary_segment_line, interpolation_distance, polygon_boundary_segment_interpolated_points);
                        
                        std::unordered_set<size_t> snappable_edge_indices_for_interpolated_points;
                        for (const auto& point : polygon_boundary_segment_interpolated_points) {
                            auto nearest_edge = adj_graph.findNearestEdge(point);
                            if (nearest_edge.has_value()) {
                                // Criteria 5: If the edge_index is already in polygon.snappable_road_ids, continue
                                if (std::find(polygon.snappable_ids.begin(), polygon.snappable_ids.end(), nearest_edge.value()) != polygon.snappable_ids.end()) {
                                    continue;
                                }

                                // compute snapped point on edge
                                auto [projected_point, distance_to_nearest_point, dist_to_closest_edge, nearest_point_vertex_position_index, edge_index, is_outside_segment] = adj_graph.computeSnappedPointOnEdge(nearest_edge.value(), point);
                                
                                // Criteria 6: If the segment from the interpolated point to the projected point intersects polygon.shrink_geometry (or crosses polygon.geometry if shrink_geometry is not available), continue
                                graph::LineString segment_to_projected_point;
                                segment_to_projected_point.push_back(point);
                                segment_to_projected_point.push_back(projected_point);
                                
                                bool polygon_boundary_at_back = false;
                                if (polygon.shrink_geometry.has_value()) {
                                    if (bg::intersects(segment_to_projected_point, polygon.shrink_geometry.value())) {
                                        polygon_boundary_at_back = true;
                                    }
                                } else {
                                    if (bg::crosses(segment_to_projected_point, polygon.geometry)) {
                                        polygon_boundary_at_back = true;
                                    }
                                }
                                
                                if (polygon_boundary_at_back) {
                                    continue;
                                }

                                snappable_edge_indices_for_interpolated_points.insert(nearest_edge.value());
                            }
                        }

                        // Only add to map if we found snappable edges
                        if (!snappable_edge_indices_for_interpolated_points.empty()) {
                            // Add the snappable edge indices to the interpolated_snappable_edge_indices_map
                            // Map the current boundary segment index i to the vector of edge indices
                            std::vector<size_t> edge_indices_vector(snappable_edge_indices_for_interpolated_points.begin(),
                                                                    snappable_edge_indices_for_interpolated_points.end());

                            polygon.interpolated_snappable_edge_indices_map[i] = edge_indices_vector;
                        }
                    }
                }
            }
        }
        else {
            // Map snappable road IDs to edge indices
            // Collect all edge indices for the feature IDs
            std::unordered_set<size_t> unique_edge_indices;
            for (size_t feature_id : polygon.snappable_ids) {
                std::vector<size_t> edge_indices = adj_graph.getEdgeIndicesForFeatureId(feature_id);
                unique_edge_indices.insert(edge_indices.begin(), edge_indices.end());
            }
            
            // Update the polygon's snappable_road_ids with the mapped edge indices
            polygon.snappable_ids.assign(unique_edge_indices.begin(), unique_edge_indices.end());
        }
    }
    
    // Clear feature ID mapping if snappable_ids_field is configured
    if (!config_.snappable_ids_field.empty()) {
        adj_graph.clearFeatureIdMapping();
    }
}

void PolygonReader::populateSnappablePointIds(PointReader& point_reader) {
    double distance = config_.candidate_access_points_search_distance;
    
    for (auto& polygon : polygons_) {
        // Skip obstacle-only polygons - they don't need access points defined
        if (polygon.is_obstacle_only) {
            continue;
        }
        
        // Skip if snappable_ids already populated by reading user-provided data from snappable_ids_field
        if (!polygon.snappable_ids.empty()) {
            continue;
        }
        
        // Generalize polygon with bounding box
        graph::Box poly_box;
        bg::envelope(polygon.geometry, poly_box);
        
        // Buffer the bounding box by candidate_access_points_search_distance (simple math: expand each side)
        double min_x = bg::get<0>(poly_box.min_corner()) - distance;
        double min_y = bg::get<1>(poly_box.min_corner()) - distance;
        double max_x = bg::get<0>(poly_box.max_corner()) + distance;
        double max_y = bg::get<1>(poly_box.max_corner()) + distance;
        graph::Box query_box(graph::Point(min_x, min_y), graph::Point(max_x, max_y));
        
        // Query PointReader's r-tree (built in readFeatures) for points within the buffered box
        // Returns point feature indices (positions in points_ vector), not feature IDs
        std::vector<size_t> point_indices = point_reader.findPointsIntersectBoundingBox(query_box);
        polygon.snappable_ids = std::move(point_indices);
    }
}

void PolygonReader::clearSpatialIndex() {
    rtree_.clear();
}

void PolygonReader::clearPolygons() {
    polygons_.clear();
    rtree_.clear();
}

void PolygonReader::initGDAL() {
    GDALAllRegister();
}

bool PolygonReader::readFeatures() {
    OGRLayerH layer = GDALDatasetGetLayer(dataset_, config_.layer_index);
    if (!layer) {
        std::cerr << "Error: Failed to get layer " << config_.layer_index << std::endl;
        return false;
    }
    
    OGR_L_ResetReading(layer);
    OGRFeatureH feature;
    
    while ((feature = OGR_L_GetNextFeature(layer)) != nullptr) {
        OGRGeometryH geometry = OGR_F_GetGeometryRef(feature);
        if (!geometry) {
            OGR_F_Destroy(feature);
            continue;
        }
        
        // Check if geometry is a supported polygon type
        OGRwkbGeometryType geom_type = OGR_G_GetGeometryType(geometry);
        if (geom_type != wkbPolygon && geom_type != wkbPolygon25D &&
            geom_type != wkbMultiPolygon && geom_type != wkbMultiPolygon25D) {
            std::cout << "Warning: Skipping feature " << OGR_F_GetFID(feature) 
                     << " with unsupported geometry type: " << OGRGeometryTypeToName(geom_type) << std::endl;
            OGR_F_Destroy(feature);
            continue;
        }
        
        // Transform geometry if needed
        if (!transformGeometry(geometry)) {
            std::cerr << "Warning: Failed to transform geometry for feature " << OGR_F_GetFID(feature) 
                     << ". Skipping." << std::endl;
            OGR_F_Destroy(feature);
            continue;
        }
        
        // Get feature ID
        size_t feature_id = OGR_F_GetFID(feature);
        if (!config_.id_field.empty()) {
            auto id_value = getFieldValueAsSizeT(feature, config_.id_field);
            if (id_value.has_value()) {
                feature_id = id_value.value();
            }
        }
        
        // Get obstacle-only flag
        bool is_obstacle_only = false;
        if (!config_.is_obstacle_only_field.empty()) {
            is_obstacle_only = getFieldValueAsBool(feature, config_.is_obstacle_only_field, false);
        }
        
        // Get snappable road IDs (from snappable_ids_field; when road dataset is provided these are road IDs)
        std::vector<size_t> snappable_ids;
        if (!config_.snappable_ids_field.empty()) {
            std::string road_ids_str = getFieldValueAsString(feature, config_.snappable_ids_field);
            if (!road_ids_str.empty()) {
                snappable_ids = parseRoadIds(road_ids_str);
            }
        }
        
        // Process the geometry - handle both single polygons and multipolygons
        if (geom_type == wkbPolygon || geom_type == wkbPolygon25D) {
            // Single polygon
            processSinglePolygon(geometry, feature_id, is_obstacle_only, snappable_ids);
        } else if (geom_type == wkbMultiPolygon || geom_type == wkbMultiPolygon25D) {
            // Multipolygon - use only the first polygon
            if (OGR_G_GetGeometryCount(geometry) > 0) {
                OGRGeometryH first_polygon = OGR_G_GetGeometryRef(geometry, 0);
                if (first_polygon && (OGR_G_GetGeometryType(first_polygon) == wkbPolygon || 
                                     OGR_G_GetGeometryType(first_polygon) == wkbPolygon25D)) {
                    processSinglePolygon(first_polygon, feature_id, is_obstacle_only, snappable_ids);
                } else {
                    std::cout << "Warning: Invalid first polygon in multipolygon feature " << feature_id << ". Skipping." << std::endl;
                }
            } else {
                std::cout << "Warning: Empty multipolygon feature " << feature_id << ". Skipping." << std::endl;
            }
        }
        
        OGR_F_Destroy(feature);
    }
    
    std::cout << "Successfully read " << polygons_.size() << " polygon features" << std::endl;
    
    // Build spatial index
    buildSpatialIndex();
    
    return true;
}

void PolygonReader::buildSpatialIndex() {
    std::vector<graph::PolygonRTreeValue> rtree_values;
    rtree_values.reserve(polygons_.size());
    
    for (const auto& polygon : polygons_) {
        // Use shrink geometry if available, otherwise use original
        graph::Box bounding_box;
        if (polygon.shrink_geometry.has_value()) {
            bg::envelope(polygon.shrink_geometry.value(), bounding_box);
        } else {
            graph::Polygon temp_polygon = polygon.geometry;
            bg::envelope(temp_polygon, bounding_box);
        }
        
        rtree_values.emplace_back(bounding_box, polygon.index);
    }
    
    // Build R-tree
    rtree_ = PolygonRTree(rtree_values.begin(), rtree_values.end());
    
    std::cout << "Built spatial index for " << rtree_values.size() << " polygons" << std::endl;
}

std::optional<size_t> PolygonReader::getFieldValueAsSizeT(const OGRFeatureH feature,
                                                          const std::string& field_name) const {
    int field_index = OGR_F_GetFieldIndex(feature, field_name.c_str());
    if (field_index == -1) {
        return std::nullopt;
    }
    
    if (OGR_F_IsFieldNull(feature, field_index)) {
        return std::nullopt;
    }
    
    return static_cast<size_t>(OGR_F_GetFieldAsInteger64(feature, field_index));
}

bool PolygonReader::getFieldValueAsBool(const OGRFeatureH feature,
                                        const std::string& field_name,
                                        bool default_value) const {
    int field_index = OGR_F_GetFieldIndex(feature, field_name.c_str());
    if (field_index == -1) {
        return default_value;
    }
    
    if (OGR_F_IsFieldNull(feature, field_index)) {
        return default_value;
    }
    
    return OGR_F_GetFieldAsInteger(feature, field_index) != 0;
}

std::string PolygonReader::getFieldValueAsString(const OGRFeatureH feature,
                                                 const std::string& field_name) const {
    int field_index = OGR_F_GetFieldIndex(feature, field_name.c_str());
    if (field_index == -1) {
        return "";
    }
    
    if (OGR_F_IsFieldNull(feature, field_index)) {
        return "";
    }
    
    const char* value = OGR_F_GetFieldAsString(feature, field_index);
    return value ? std::string(value) : "";
}

std::vector<size_t> PolygonReader::parseRoadIds(const std::string& road_ids_str) const {
    std::vector<size_t> road_ids;
    std::istringstream iss(road_ids_str);
    std::string token;
    
    while (std::getline(iss, token, ',')) {
        // Trim whitespace
        token.erase(0, token.find_first_not_of(" \t\r\n"));
        token.erase(token.find_last_not_of(" \t\r\n") + 1);
        
        if (!token.empty()) {
            try {
                size_t road_id = std::stoull(token);
                road_ids.push_back(road_id);
            } catch (const std::exception& e) {
                std::cerr << "Warning: Failed to parse road ID '" << token << "': " << e.what() << std::endl;
            }
        }
    }
    
    return road_ids;
}

graph::Polygon PolygonReader::convertOGRToPolygon(const OGRGeometryH ogr_geom) const {
    if (OGR_G_GetGeometryType(ogr_geom) != wkbPolygon && OGR_G_GetGeometryType(ogr_geom) != wkbPolygon25D) {
        throw std::runtime_error("Invalid geometry type for polygon conversion");
    }
    
    // Get exterior ring
    OGRGeometryH exterior_ring = OGR_G_GetGeometryRef(ogr_geom, 0);
    
    if (!exterior_ring || OGR_G_GetPointCount(exterior_ring) < 3) {
        throw std::runtime_error("Invalid exterior ring for polygon");
    }
    
    // Convert exterior ring to Boost polygon
    graph::Polygon boost_polygon;
    auto& outer_ring = boost_polygon.outer();
    
    int num_points = OGR_G_GetPointCount(exterior_ring);
    for (int i = 0; i < num_points; ++i) {
        double x, y, z;
        OGR_G_GetPoint(exterior_ring, i, &x, &y, &z);
        
        // Use raw coordinates to preserve full precision
        outer_ring.push_back(graph::Point(x, y));
    }
    
    // Ensure the ring is closed (Boost geometry requirement)
    if (outer_ring.size() > 0 && 
        (bg::get<0>(outer_ring.front()) != bg::get<0>(outer_ring.back()) ||
         bg::get<1>(outer_ring.front()) != bg::get<1>(outer_ring.back()))) {
        outer_ring.push_back(outer_ring.front());
    }
    
    // Validate and correct the polygon
    if (!bg::is_valid(boost_polygon)) {
        bg::correct(boost_polygon);
        
        if (!bg::is_valid(boost_polygon)) {
            std::string reason;
            if (!bg::is_valid(boost_polygon, reason)) {
                std::cerr << "Error: Polygon validation failed after correction (" << reason << ")" << std::endl;
            }
            throw std::runtime_error("Failed to create valid polygon");
        }
    }
    
    return boost_polygon;
}

std::vector<graph::PolygonFeature> PolygonReader::queryPolygonRTree(const graph::LineString& segment) const {
    std::vector<graph::PolygonFeature> crossed_obstacles;
    std::vector<graph::PolygonRTreeValue> candidates;
    
    rtree_.query(
        bgi::intersects(segment) && 
        bgi::satisfies([this, &segment](const graph::PolygonRTreeValue& candidate) {
            const auto& polygon = polygons_[candidate.polygon_index];

            // Use pre-computed shrink geometry for performance optimization
            if (polygon.shrink_geometry.has_value()) {
                const graph::Polygon& shrunk_polygon = *polygon.shrink_geometry;
                return bg::intersects(segment, shrunk_polygon);
            } else {
                // Interior intersects
                return bg::crosses(segment, polygon.geometry) || 
                       bg::within(segment, polygon.geometry);
            }
        }),
        std::back_inserter(candidates)
    );
    
    for (const auto& candidate : candidates) {
        crossed_obstacles.push_back(polygons_[candidate.polygon_index]);
    }
    
    return crossed_obstacles;
}

bool PolygonReader::transformGeometry(OGRGeometryH geom) const {
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

bool PolygonReader::handleCoordinateSystem() {
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
        std::cout << "Polygon dataset is in EPSG:4326 (WGS84)" << std::endl;
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
            std::cout << "Polygon dataset coordinate system: EPSG:" << coordinate_system_epsg_ << std::endl;
        } else {
            coordinate_system_epsg_ = -1;
            std::cout << "Polygon dataset coordinate system: Unknown (WKT: " << coordinate_system_wkt_ << ")" << std::endl;
        }
    }
    
    OSRDestroySpatialReference(spatial_ref);
    return true;
}

std::optional<graph::Polygon> PolygonReader::computeShrinkGeometry(const graph::Polygon& geometry, size_t feature_id) const {
    // Static buffer strategies - declared once and reused for performance  
    static const boost::geometry::strategy::buffer::distance_symmetric<double> distance_strategy(-1e-2);
    static const boost::geometry::strategy::buffer::end_flat end_strategy;
    static const boost::geometry::strategy::buffer::side_straight side_strategy;
    static const boost::geometry::strategy::buffer::join_miter join_strategy;
    static const boost::geometry::strategy::buffer::point_circle point_strategy(8);

    try {
        // Buffer operation outputs to multi_polygon, not single polygon
        bg::model::multi_polygon<graph::Polygon> buffered_multi_polygon;
        
        bg::buffer(geometry, buffered_multi_polygon, 
                  distance_strategy, side_strategy, join_strategy, end_strategy, point_strategy);
        
        // Check if buffering produced any result
        if (!buffered_multi_polygon.empty()) {
            // Use the first (largest) polygon from the result
            return buffered_multi_polygon[0];
        }
        // If buffering failed or produced no result, shrink_geometry remains nullopt
        
    } catch (const std::exception& e) {
        // If buffering fails, leave shrink_geometry as nullopt to fall back to original intersection checking
        std::cerr << "Warning: Failed to compute shrink geometry for polygon " << feature_id 
                  << ". Will fall back to original intersection checking. Error: " << e.what() << std::endl;
    }
    
    return std::nullopt;
}

void PolygonReader::processSinglePolygon(OGRGeometryH geometry, size_t feature_id, bool is_obstacle_only, 
                                        const std::vector<size_t>& snappable_ids) {
    
    // Convert geometry to boost geometry polygon
    graph::Polygon boost_polygon = convertOGRToPolygon(geometry);
    
    // Extract outer ring
    const auto& outer = bg::exterior_ring(boost_polygon);
    std::vector<graph::Point> outer_ring;
    
    // Copy all points from exterior ring
    outer_ring.assign(outer.begin(), outer.end());
    
    // Remove duplicate closing point if present
    if (outer_ring.size() > 1 && 
        bg::get<0>(outer_ring.front()) == bg::get<0>(outer_ring.back()) &&
        bg::get<1>(outer_ring.front()) == bg::get<1>(outer_ring.back())) {
        outer_ring.pop_back();
    }
    
    // Ensure we have at least 3 points for a valid polygon
    if (outer_ring.size() < 3) {
        std::cerr << "Warning: Invalid polygon feature " << feature_id 
                 << " with insufficient points (" << outer_ring.size() << "). Skipping." << std::endl;
        return;
    }
    
    // Compute shrink geometry
    auto shrink_geometry = computeShrinkGeometry(boost_polygon, feature_id);
    
    // Create polygon feature
    size_t index = polygons_.size();
    polygons_.emplace_back(index, feature_id, outer_ring, boost_polygon, shrink_geometry, 
                          snappable_ids, is_obstacle_only);
}

} // namespace io
} // namespace adjfind
