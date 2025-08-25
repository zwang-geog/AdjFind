#ifndef ADJFIND_COMMON_HPP
#define ADJFIND_COMMON_HPP

#include <cstddef>
#include <vector>
#include <tuple>
#include <limits>
#include <unordered_set>
#include <optional>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include <boost/geometry/index/rtree.hpp>

namespace adjfind {
namespace graph {

// Boost Geometry namespace aliases
namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

// 2D Geometry types (for data reading)
using Point = bg::model::point<double, 2, bg::cs::cartesian>;
using LineString = bg::model::linestring<Point>;
using MultiLineString = bg::model::multi_linestring<LineString>;
using Segment = bg::model::segment<Point>;
using Box = bg::model::box<Point>;
using Polygon = bg::model::polygon<Point>;
using LinearRing = bg::model::ring<Point>;
using MultiPoint = bg::model::multi_point<Point>;

// 3D Geometry types (for graph structures)
using Point3D = bg::model::point<double, 3, bg::cs::cartesian>;
using LineString3D = bg::model::linestring<Point3D>;
using Segment3D = bg::model::segment<Point3D>;
using Box3D = bg::model::box<Point3D>;

// Edge types for convex path graph
enum class EdgeType {
    HULL_EDGE,              // Edge from convex hull boundary
    CONNECTION_TO_START,    // Connection edge to root start vertex
    CONNECTION_TO_END       // Connection edge to root end vertex
};

// Vertex types
enum class VertexType {
    POINT,          // Point vertex (from input points)
    INTERSECTION,   // Road intersection vertex
    SPLIT           // Split vertex (created at equilibrium points)
};

// Vertex structure
struct Vertex {
    size_t vertex_index;              // Graph vertex index
    size_t feature_id;                // Raw data feature ID if type is POINT, otherwise same as vertex_index
    Point3D geometry;                 // 3D geometry for graph vertices
    VertexType type;
    double distance_to_point_vertex;  // Distance to nearest point vertex
    size_t nearest_point_vertex_position_index;  // Index of nearest point vertex
    
    Vertex(size_t vertex_idx, size_t feat_id, const Point3D& geom, VertexType vertex_type)
        : vertex_index(vertex_idx), feature_id(feat_id), geometry(geom), type(vertex_type),
          distance_to_point_vertex(vertex_type == VertexType::POINT ? 0.0 : std::numeric_limits<double>::max()),
          nearest_point_vertex_position_index(vertex_type == VertexType::POINT ? vertex_idx : std::numeric_limits<size_t>::max()) {}
};

// Edge structure
struct Edge {
    size_t edge_index;                // Graph edge index
    size_t feature_id;                // Raw data feature ID
    size_t from_vertex;
    size_t to_vertex;
    double weight;
    LineString geometry;              // 2D road geometry for graph edges
    size_t nearest_point_vertex_position_index;  // Index of nearest point vertex
    size_t endpoint_vertex_to_nearest_point;    // Vertex index of endpoint connected to nearest point
    bool endpoint_vertex_to_nearest_point_is_from_vertex;  // True if endpoint vertex is from_vertex, false if to_vertex
    
    Edge(size_t edge_idx, size_t feat_id, size_t from, size_t to, double edge_weight, const LineString& geom)
        : edge_index(edge_idx), feature_id(feat_id), from_vertex(from), to_vertex(to), weight(edge_weight), geometry(geom),
          nearest_point_vertex_position_index(std::numeric_limits<size_t>::max()), 
          endpoint_vertex_to_nearest_point(std::numeric_limits<size_t>::max()), 
          endpoint_vertex_to_nearest_point_is_from_vertex(true) {}
};

// Road feature structure
struct RoadFeature {
    size_t id;
    LineString geometry;
    double length;
    double from_z;
    double to_z;
    bool is_deleted;
    
    RoadFeature(size_t feature_id, const LineString& geom, double feature_length, double from_z_level, double to_z_level)
        : id(feature_id), geometry(geom), length(feature_length), from_z(from_z_level), to_z(to_z_level), is_deleted(false) {}
};

// Point feature structure
struct PointFeature {
    size_t id;
    Point geometry;
    
    PointFeature(size_t feature_id, const Point& geom)
        : id(feature_id), geometry(geom) {}
};

// Polygon feature structure
struct PolygonFeature {
    size_t index;                           // Index position in the vector
    size_t feature_id;                      // Original feature ID or OGR FID
    std::vector<Point> outer_ring;         // Outer boundary points as boost geometry Point objects excluding duplicate closing point
    Polygon geometry;                       // Original polygon geometry
    std::optional<Polygon> shrink_geometry; // Shrunk geometry for intersection checking
    std::vector<size_t> snappable_road_ids; // Road IDs that can be snapped to
    bool is_obstacle_only;                  // Whether this polygon is obstacle-only
    std::unordered_map<size_t, std::vector<size_t>> interpolated_snappable_edge_indices_map; // Map of polygon outer ring point index to vector of edge indices
    
    PolygonFeature(size_t idx, size_t feat_id, const std::vector<Point>& outer, 
                   const Polygon& geom,
                   const std::optional<Polygon>& shrink = std::nullopt,
                   const std::vector<size_t>& road_ids = std::vector<size_t>(),
                   bool obstacle_only = false)
        : index(idx), feature_id(feat_id), outer_ring(outer), geometry(geom),
          shrink_geometry(shrink), snappable_road_ids(road_ids), is_obstacle_only(obstacle_only), 
          interpolated_snappable_edge_indices_map() {}
};

// Snapped point information
struct SnappedPointInfo {
    size_t original_point_index;      // Index in original point vector
    Point snapped_geometry;           // Snapped point geometry
    double distance_to_first_point;   // Distance from snapped point to linestring first point
    size_t segment_index;             // Which segment of the linestring it's snapped to
    
    SnappedPointInfo(size_t point_idx, const Point& snapped_geom, double dist_to_first, size_t seg_idx)
        : original_point_index(point_idx), snapped_geometry(snapped_geom), 
          distance_to_first_point(dist_to_first), segment_index(seg_idx) {}
};

// Road split by distance brackets output structure
struct RoadSplitByDistanceBracketsOutput {
    LineString geometry;
    double length;
    std::string distance_category;
    double from_distance;
    double to_distance;
    size_t raw_feature_id;
    size_t nearest_point_id;
    
    RoadSplitByDistanceBracketsOutput(const LineString& geom, double len, const std::string& category, 
                                     double from_dist, double to_dist, size_t feature_id, size_t point_id)
        : geometry(geom), length(len), distance_category(category), from_distance(from_dist), to_distance(to_dist),
          raw_feature_id(feature_id), nearest_point_id(point_id) {}
};

// R-tree value types for data reading (2D)
struct RoadRTreeValue {
    Segment segment;
    size_t road_index;  // Changed from road_id to road_index (vector position)
    size_t segment_index;
    
    RoadRTreeValue(const Segment& seg, size_t road_idx, size_t seg_idx)
        : segment(seg), road_index(road_idx), segment_index(seg_idx) {}
};

struct PointRTreeValue {
    Point location;
    size_t point_id;
    size_t feature_index;
    
    PointRTreeValue(const Point& pt, size_t point_id_val, size_t feat_idx)
        : location(pt), point_id(point_id_val), feature_index(feat_idx) {}
};

// R-tree value types for graph operations (3D)
struct VertexRTreeValue {
    Point3D location;
    size_t vertex_id;
    
    VertexRTreeValue(const Point3D& pt, size_t vertex_idx)
        : location(pt), vertex_id(vertex_idx) {}
};

// Edge R-tree value structure
struct EdgeRTreeValue {
    Segment segment;
    size_t edge_index;
    
    EdgeRTreeValue(const Segment& seg, size_t edge_idx)
        : segment(seg), edge_index(edge_idx) {}
};

// R-tree value structure for polygons
struct PolygonRTreeValue {
    Box bounding_box;
    size_t polygon_index;
    
    PolygonRTreeValue(const Box& box, size_t poly_idx)
        : bounding_box(box), polygon_index(poly_idx) {}
};

/**
 * Structure representing a vertex in the convex path graph
 */
struct ConvexPathGraphVertex {
    Point location;
    size_t vertex_id;
    
    ConvexPathGraphVertex(const Point& loc, size_t id) 
        : location(loc), vertex_id(id) {}
};

/**
 * Structure representing an edge in the convex path graph
 */
struct ConvexPathGraphEdge {
    size_t from_vertex;
    size_t to_vertex;
    double weight;
    bool is_deleted;
    EdgeType edge_type;
    size_t nearest_point_vertex_position_index;  // Index of nearest point vertex on the road edge
    size_t edge_index;                           // Index of the road edge this connects to
    
    ConvexPathGraphEdge(size_t from, size_t to, double w) 
        : from_vertex(from), to_vertex(to), weight(w), is_deleted(false), edge_type(EdgeType::HULL_EDGE),
          nearest_point_vertex_position_index(std::numeric_limits<size_t>::max()), edge_index(std::numeric_limits<size_t>::max()) {}
    
    ConvexPathGraphEdge(size_t from, size_t to, EdgeType type) 
        : from_vertex(from), to_vertex(to), weight(0.0), is_deleted(false), edge_type(type),
          nearest_point_vertex_position_index(std::numeric_limits<size_t>::max()), edge_index(std::numeric_limits<size_t>::max()) {}
    
    ConvexPathGraphEdge(size_t from, size_t to, double w, bool is_deleted, EdgeType type, size_t nearest_point_idx, size_t edge_idx) 
        : from_vertex(from), to_vertex(to), weight(w), is_deleted(is_deleted), edge_type(type),
          nearest_point_vertex_position_index(nearest_point_idx), edge_index(edge_idx) {}
};

/**
 * Structure representing a pair of vertices for edge tracking
 */
struct ConvexPathEdgePair {
    size_t vertex1;
    size_t vertex2;
    
    ConvexPathEdgePair(size_t v1, size_t v2) 
        : vertex1(std::min(v1, v2)), vertex2(std::max(v1, v2)) {}
    
    bool operator==(const ConvexPathEdgePair& other) const {
        return vertex1 == other.vertex1 && vertex2 == other.vertex2;
    }
};

/**
 * Hash function for ConvexPathEdgePair
 */
struct ConvexPathEdgePairHash {
    std::size_t operator()(const ConvexPathEdgePair& pair) const {
        return std::hash<size_t>()(pair.vertex1) ^ (std::hash<size_t>()(pair.vertex2) << 1);
    }
};

/**
 * Hash function for std::pair<size_t, size_t> (used for vertex pairs)
 */
struct PairHash {
    std::size_t operator()(const std::pair<size_t, size_t>& p) const {
        return std::hash<size_t>{}(p.first) ^ (std::hash<size_t>{}(p.second) << 1);
    }
};

/**
 * Enumeration for different types of convex path results
 */
enum class ConvexPathResultType {
    NOT_FOUND,              // No path found
    BUILDING_CORNER,        // Path to building corner
    LEAST_ACCESSIBLE_POINT  // Path to least accessible point
};

/**
 * Structure representing the result of a convex path computation
 */
struct ConvexPathResult {
    LineString path_geometry;
    std::vector<size_t> edge_indices;  // Edge indices used in the path for easy deletion
    double total_length;
    double objective_distance;          // Objective distance (depend on include_network_distance flag)
    bool path_found;
    size_t nearest_point_vertex_position_index;  // Index of nearest point vertex for artificial edge to vertex 1
    size_t edge_index;                           // Index of the road edge for artificial edge to vertex 1
    ConvexPathResultType start_point_type;       // Type of start point for the path
    
    ConvexPathResult() : total_length(0.0), objective_distance(0.0), path_found(false), 
                        nearest_point_vertex_position_index(std::numeric_limits<size_t>::max()), 
                        edge_index(std::numeric_limits<size_t>::max()),
                        start_point_type(ConvexPathResultType::NOT_FOUND) {}
    
    ConvexPathResult(const LineString& geometry, const std::vector<size_t>& edges, double total_len, double objective_dist, 
                    size_t nearest_point_idx, size_t edge_idx) 
        : path_geometry(geometry), edge_indices(edges), total_length(total_len), objective_distance(objective_dist), path_found(true),
          nearest_point_vertex_position_index(nearest_point_idx), edge_index(edge_idx),
          start_point_type(ConvexPathResultType::NOT_FOUND) {}
};

/**
 * Structure representing intersection information for polygon splitting
 */
struct PolygonSplittingIntersectionInfo {
    Point from_point;                    // First point of the intersection linestring
    Point to_point;          // Last point of the intersection linestring
    size_t from_crossed_edge_index;      // Index of the crossed polygon edge corresponding to from_point
    size_t to_crossed_edge_index;        // Index of the crossed polygon edge corresponding to to_point
    size_t from_inserted_position;       // Position in the modified ring for from_point
    size_t to_inserted_position;         // Position in the modified ring for to_point
    
    PolygonSplittingIntersectionInfo(const Point& from_pt, const Point& to_pt, size_t from_edge_idx, size_t to_edge_idx, size_t from_inserted_pos = SIZE_MAX, size_t to_inserted_pos = SIZE_MAX)
        : from_point(from_pt), to_point(to_pt), from_crossed_edge_index(from_edge_idx), to_crossed_edge_index(to_edge_idx), from_inserted_position(from_inserted_pos), to_inserted_position(to_inserted_pos) {}
};

/**
 * Structure representing a candidate edge with metadata
 * Note: Convexity information is stored in the parent ConvexHullResult structure
 */
struct CandidateEdge {
    size_t from_vertex_id;
    size_t to_vertex_id;
    EdgeType type;
    
    CandidateEdge(size_t from, size_t to, EdgeType edge_type) 
        : from_vertex_id(from), to_vertex_id(to), type(edge_type) {}
};

/**
 * Structure representing the result of convex hull segment generation
 */
struct ConvexHullResult {
    std::vector<CandidateEdge> candidate_edges;  // Generated candidate edges
    bool is_concave_case;                              // Whether this is a concave case
    
    ConvexHullResult() : is_concave_case(false) {}
};

/**
 * Structure to track iteration state for edge decision logic
 */
struct IterationState {
    size_t current_obstacle_count = 0;
    size_t previous_obstacle_count = 0;
    std::vector<PolygonFeature> global_obstacles;       // Psi - all obstacles for intersection checking
    std::vector<PolygonFeature> current_obstacles;      // psi_check (scope iii) - obstacles found in current iteration only
    std::unordered_set<size_t> seen_building_ids;           // Track unique building feature IDs
    
    void updateIteration(size_t new_count) {
        previous_obstacle_count = current_obstacle_count;
        current_obstacle_count = new_count;
    }
    
    bool obstacleCountChanged() const {
        return current_obstacle_count != previous_obstacle_count;
    }
};

} // namespace graph
} // namespace adjfind

// Boost Geometry R-tree specializations
namespace boost { namespace geometry { namespace index {

template<>
struct indexable<adjfind::graph::RoadRTreeValue> {
    using result_type = adjfind::graph::Segment;
    result_type const& operator()(adjfind::graph::RoadRTreeValue const& v) const {
        return v.segment;
    }
};

template<>
struct indexable<adjfind::graph::PointRTreeValue> {
    using result_type = adjfind::graph::Point;
    result_type const& operator()(adjfind::graph::PointRTreeValue const& v) const {
        return v.location;
    }
};

template<>
struct indexable<adjfind::graph::VertexRTreeValue> {
    using result_type = adjfind::graph::Point3D;
    result_type const& operator()(adjfind::graph::VertexRTreeValue const& v) const {
        return v.location;
    }
};

template<>
struct indexable<adjfind::graph::EdgeRTreeValue> {
    using result_type = adjfind::graph::Segment;
    result_type const& operator()(adjfind::graph::EdgeRTreeValue const& v) const {
        return v.segment;
    }
};

template<>
struct indexable<adjfind::graph::PolygonRTreeValue> {
    using result_type = adjfind::graph::Box;
    result_type const& operator()(adjfind::graph::PolygonRTreeValue const& v) const {
        return v.bounding_box;
    }
};

}}} // namespace boost::geometry::index

#endif // ADJFIND_COMMON_HPP 