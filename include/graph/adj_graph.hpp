#ifndef ADJFIND_ADJ_GRAPH_HPP
#define ADJFIND_ADJ_GRAPH_HPP

#include <vector>
#include <unordered_map>
#include <queue>
#include <limits>
#include <iostream>
#include <optional>
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include "graph/common.hpp"
#include "io/road_reader.hpp"
#include "io/point_reader.hpp"

namespace adjfind {
namespace graph {

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

// R-tree types for vertex management (3D)
using VertexRTree = bgi::rtree<VertexRTreeValue, bgi::quadratic<16>>;

// R-tree types for edge management (2D segments)
using EdgeRTree = bgi::rtree<EdgeRTreeValue, bgi::quadratic<16>>;

class AdjGraph {
public:
    AdjGraph();
    ~AdjGraph() = default;
    
    // Disable copy constructor and assignment
    AdjGraph(const AdjGraph&) = delete;
    AdjGraph& operator=(const AdjGraph&) = delete;
    
    /**
     * Add a vertex to the graph
     * @param geometry 3D Point geometry of the vertex
     * @param type Type of the vertex
     * @return Vertex ID
     */
    size_t addVertex(const Point3D& geometry, VertexType type, size_t feature_id);
    
    /**
     * Add an edge to the graph
     * @param from_vertex Source vertex ID
     * @param to_vertex Target vertex ID
     * @param weight Edge weight
     * @param feature_id Raw data feature ID
     * @param geometry 2D Road geometry
     * @return Edge ID
     */
    size_t addEdge(size_t from_vertex, size_t to_vertex, double weight, size_t feature_id, const LineString& geometry);
    
    /**
     * Get a vertex by ID
     * @param vertex_id Vertex ID
     * @return Vertex if found
     */
    const Vertex& getVertex(size_t vertex_id) const;
    
    /**
     * Get an edge by ID
     * @param edge_id Edge ID
     * @return Edge if found
     */
    const Edge& getEdge(size_t edge_id) const;
    
    /**
     * Get all vertices
     * @return Vector of vertices
     */
    const std::vector<Vertex>& getVertices() const { return vertices_; }
    
    /**
     * Get all edges
     * @return Vector of edges
     */
    const std::vector<Edge>& getEdges() const { return edges_; }
    
    /**
     * Get number of vertices
     * @return Number of vertices
     */
    size_t getVertexCount() const { return vertices_.size(); }
    
    /**
     * Get number of edges
     * @return Number of edges
     */
    size_t getEdgeCount() const { return edges_.size(); }
    
    /**
     * Build R-tree index for all edges
     * Creates segments from linestring geometries and stores them with edge indices
     */
    void buildEdgeRTree();
    
    /**
     * Find the nearest edge to a query point
     * @param query_point Query point (2D)
     * @return Optional edge index of the nearest edge
     */
    std::optional<size_t> findNearestEdge(const Point& query_point) const;
    
    /**
     * Clear the edge R-tree to free memory
     */
    void clearEdgeRTree();
    
    /**
     * Build mapping from feature_id to vector of edge indices
     * This mapping is needed when converting between raw data IDs and internal edge indices
     */
    void buildFeatureIdMapping();
    
    /**
     * Clear the feature ID to edge indices mapping
     * This method should be called after the mapping is no longer needed to free memory
     */
    void clearFeatureIdMapping();
    
    /**
     * Get all edge indices for a given feature ID
     * @param feature_id Raw data feature ID
     * @return Vector of edge indices corresponding to the feature ID
     */
    std::vector<size_t> getEdgeIndicesForFeatureId(size_t feature_id) const;

    
    /**
     * Get vertices connected to a vertex with edge information
     * @param vertex_id Vertex ID
     * @return Vector of tuples (connected_vertex_id, edge_id, edge_weight)
     */
    const std::vector<std::tuple<size_t, size_t, double>>& getConnectedVertices(size_t vertex_id) const;
    

    /**
     * Find or create a vertex for a given point (with spatial tolerance)
     * @param point The point to find or create vertex for
     * @return Vertex ID
     */
    size_t findOrCreateVertex(const Point3D& point);
    
    /**
     * Find existing vertex for a point (with spatial tolerance)
     * @param point The 3D point to find vertex for
     * @return Optional vertex ID if found
     */
    std::optional<size_t> findVertexForPoint(const Point3D& point) const;
    
    /**
     * Add a road segment to the graph
     * @param geometry Original 2D road linestring geometry
     * @param from_z Z-value at start (for vertex creation)
     * @param to_z Z-value at end (for vertex creation)
     * @param length Length of the segment
     * @param feature_id Raw data feature ID
     */
    void addRoadSegment(const LineString& geometry, double from_z, double to_z, double length, size_t feature_id);
    
    /**
     * Get vertices with type POINT
     * @return Vector of point vertex IDs
     */
    std::vector<size_t> getPointVertices() const;
    
    /**
     * Get vertices with type INTERSECTION
     * @return Vector of intersection vertex IDs
     */
    std::vector<size_t> getIntersectionVertices() const;
    
    /**
     * Update vertex's nearest point vertex information
     * @param vertex_id Vertex ID to update
     * @param nearest_point_vertex_position_index Index of nearest point vertex
     * @param distance_to_point_vertex Distance to nearest point vertex
     */
    void updateVertexNearestPointVertex(size_t vertex_id, size_t nearest_point_vertex_position_index, double distance_to_point_vertex);
    
    /**
     * Read road and point data, snap points to roads, and build initial graph
     * This method implements steps 1-4 of the road segmentation workflow
     * @param road_config Road reader configuration
     * @param point_config Point reader configuration
     * @return true if successful, false otherwise
     */
    bool readAndSnapPointToRoad(const io::RoadReaderConfig& road_config, const io::PointReaderConfig& point_config);
    
    /**
     * Get coordinate system WKT
     * @return Coordinate system WKT string
     */
    std::string getCoordinateSystemWKT() const { return coordinate_system_wkt_; }
    
    /**
     * Get coordinate system EPSG code
     * @return Coordinate system EPSG code
     */
    int getCoordinateSystemEPSG() const { return coordinate_system_epsg_; }

    /**
     * Set coordinate system (used when not set via readAndSnapPointToRoad, e.g. in processConvexPathModeNoRoad)
     * @param wkt Coordinate system WKT string
     * @param epsg EPSG code
     */
    void setCoordinateSystem(const std::string& wkt, int epsg);

    /**
     * Compute the snapped point on an edge
     * @param snappable_road_edge_index Single road edge index to snap to
     * @param point The point to snap to the edge
     * @return Tuple containing (projected_point, distance_to_nearest_point, dist_to_closest_edge, nearest_point_vertex_position_index, edge_index, is_outside_segment)
     */
     std::tuple<Point, double, double, size_t, size_t, bool> computeSnappedPointOnEdge(size_t snappable_road_edge_index, 
        const Point& point) const;


public:
    // Graph data structures (public for derived classes)
    std::vector<Vertex> vertices_;
    std::vector<Edge> edges_;
    std::unordered_map<size_t, std::vector<std::tuple<size_t, size_t, double>>> adjacency_list_;  // Adjacency relationships with edge information
    size_t next_edge_id_;  // ID counter for edges
    
private:
    // Spatial index for vertices
    VertexRTree vertex_rtree_;
    
    // Spatial index for edges
    EdgeRTree edge_rtree_;
    
    // Mapping from feature_id to vector of edge indices
    std::unordered_map<size_t, std::vector<size_t>> feature_id_to_edge_indices_;
    
    // ID counters
    size_t next_vertex_id_;
    
    // Coordinate system information
    std::string coordinate_system_wkt_;
    int coordinate_system_epsg_;
    
    // Spatial tolerance for vertex matching
    static constexpr double ROAD_VERTEX_SNAPPING_TOLERANCE = 1e-2;
    

};

} // namespace graph
} // namespace adjfind

#endif // ADJFIND_ADJ_GRAPH_HPP 