#ifndef ADJFIND_CONVEX_PATH_HPP
#define ADJFIND_CONVEX_PATH_HPP

#include "graph/road_segmentation.hpp"
#include "io/polygon_reader.hpp"
#include "io/point_reader.hpp"
#include <unordered_set>
#include <vector>
#include <optional>

namespace adjfind {
namespace graph {

class ConvexPath : public RoadSegmentation {
public:
    ConvexPath();
    ~ConvexPath() = default;
    
    // Disable copy constructor and assignment
    ConvexPath(const ConvexPath&) = delete;
    ConvexPath& operator=(const ConvexPath&) = delete;
    
    /**
     * Process convex path mode with road, point, and building configurations
     * This method implements the complete convex path workflow
     * @param road_config Road reader configuration
     * @param point_config Point reader configuration
     * @param building_config Building reader configuration
     * @return true if successful, false otherwise
     */
    bool processConvexPathMode(const io::RoadReaderConfig& road_config, 
                              const io::PointReaderConfig& point_config,
                              const io::PolygonReaderConfig& building_config);
    
    /**
     * Process convex path mode with point and building configurations only (no road dataset).
     * Uses PointReader::findPointsIntersectBoundingBox via PolygonReader::populateSnappablePointIds.
     * @param point_config Point reader configuration
     * @param building_config Building reader configuration
     * @return true if successful, false otherwise
     */
    bool processConvexPathModeNoRoad(const io::PointReaderConfig& point_config,
                                    const io::PolygonReaderConfig& building_config);
    
    /**
     * Find or create a vertex in the convex path graph
     * @param point Location of the vertex
     * @return Vertex ID
     */
    size_t findOrCreateConvexPathGraphVertex(const Point& point);
    
    /**
     * Find a vertex in the convex path graph for a given point
     * @param point Location of the vertex
     * @return Optional vertex ID if found
     */
    std::optional<size_t> findConvexPathGraphVertexForPoint(const Point& point) const;
    
    /**
     * Add a segment to the convex path graph using start and end points
     * @param start_point Start point of the segment
     * @param end_point End point of the segment
     */
    void addSegmentToConvexPathGraph(const Point& start_point, const Point& end_point);
    
    /**
     * Add a segment to the convex path graph using a Segment object
     * @param segment Segment to add
     */
    void addSegmentToConvexPathGraph(const Segment& segment);
    
    /**
     * Add a segment to the convex path graph using vertex IDs
     * @param from_vertex Source vertex ID
     * @param to_vertex Target vertex ID
     * @param check_duplicate Whether to check for duplicate edges
     */
    void addSegmentToConvexPathGraph(size_t from_vertex, size_t to_vertex, bool check_duplicate = true);
    
    /**
     * Compute shortest path between two vertices on the convex path graph
     * @param start_vertex_id Starting vertex ID
     * @param end_vertex_id Ending vertex ID
     * @return ConvexPathResult containing the path geometry, edge indices, and total length
     */
    ConvexPathResult computeShortestPathOnConvexPathGraph(size_t start_vertex_id, size_t end_vertex_id);
    
    /**
     * Split a polygon using a splitting line
     * @param polygon The polygon to split
     * @param splitting_line The line used to split the polygon
     * @return Vector of polylines representing the split result
     */
    std::vector<std::vector<Point>> splitPolygon(const PolygonFeature& polygon_feature, const LineString& splitting_line);
    
     /**
      * Find or create a vertex in the polygon split graph
      * @param point Location of the vertex
      * @param is_boundary_vertex Whether this vertex is on the boundary (default: true)
      * @return Vertex ID
      */
     size_t findOrCreatePolygonSplitGraphVertex(const Point& point, bool is_boundary_vertex = true);
     
     /**
      * Add a segment to the polygon split graph
      * @param start_point Start point of the segment
      * @param end_point End point of the segment
      * @param is_boundary_edge Whether this edge is on the boundary (default: true)
      */
     void addSegmentToSplitPolygonGraph(const Point& start_point, const Point& end_point, bool is_boundary_edge = true);
     
     /**
      * Get boundary vertices from the polygon split graph
      * @return Vector of boundary vertex IDs
      */
     std::vector<size_t> getPolygonSplitBoundaryVertices() const;
     
     /**
      * Get vertex location by vertex ID from the polygon split graph
      * @param vertex_id The vertex ID to look up
      * @return Optional Point location if vertex exists
      */
     std::optional<Point> getPolygonSplitVertexLocation(size_t vertex_id) const;

    /**
     * Generate convex hull segments for a polygon feature
     * @param polygon_feature The polygon feature to process
     * @param start_point The start point for path finding
     * @param end_point The end point for path finding
     * @param add_direct_connection Whether to add direct connections to root vertices
     * @param end_point_is_last_path_point Whether the end point is the last point in the path (default: false)
     * @return ConvexHullResult containing candidate edges and case information
     */
    ConvexHullResult generateConvexHullSegments(const PolygonFeature& polygon_feature, 
                                               const Point& start_point, 
                                               const Point& end_point,
                                               bool add_direct_connection,
                                               bool end_point_is_last_path_point = false);

    /**
     * Update global obstacle sets and iteration state
     * @param obstacles Vector of obstacles found in current iteration
     * @param iteration_state Reference to iteration state to update
     */
    void updateGlobalObstacleSets(const std::vector<PolygonFeature>& obstacles,
                                 IterationState& iteration_state);

    /**
     * Decide which edges to add based on hull results and iteration state
     * @param hull_results Vector of convex hull results
     * @param iteration_state Current iteration state
     * @param is_indirect_obstacle Whether these are indirect obstacles
     */
    void decideEdgesToAdd(const std::vector<ConvexHullResult>& hull_results,
                         const IterationState& iteration_state,
                         bool is_indirect_obstacle);

    /**
     * Check if an edge intersects with any obstacles in a vector
     * @param edge The candidate edge to check
     * @param obstacles Vector of obstacles to check against
     * @return true if edge intersects with any obstacle, false otherwise
     */
    bool checkInteriorIntersectWithVectorOfObstacles(const CandidateEdge& edge,
                                                    const std::vector<PolygonFeature>& obstacles);

    /**
     * Check if a line segment intersects with a polygon's interior
     * @param segment The line segment to check
     * @param polygon_feature The polygon feature to check against
     * @return true if segment intersects polygon interior, false otherwise
     */
    bool segmentIntersectsPolygonInterior(const LineString& segment, 
                                         const PolygonFeature& polygon_feature);

    /**
     * Identify and resolve crossing obstacles on the shortest path
     * @param iteration_state Reference to iteration state to update
     * @param is_indirect_obstacle Whether these are indirect obstacles
     * @return Pair containing path result and obstacle count
     */
    std::pair<ConvexPathResult, size_t> identifyAndResolveCrossingObstaclesOnShortestPath(
        IterationState& iteration_state,
        bool is_indirect_obstacle);

    /**
     * Find convex path from start point using snappable road IDs
     * @param start_point The starting point for path finding
     * @param snappable_ids Vector of snappable (edge or point) IDs that can be snapped to
     * @return ConvexPathResult containing the path geometry, edge indices, and total length
     */
    ConvexPathResult findConvexPath(const Point& start_point, const std::vector<size_t>& snappable_ids);

    /**
     * Process a single polygon to find the least accessible point
     * @param polygon_feature The polygon feature to process
     * @return Tuple containing vertex paths, feature ID, and least accessible point
     */
    std::tuple<std::vector<ConvexPathResult>, size_t, Point> processSinglePolygon(const PolygonFeature& polygon_feature);

    /**
     * Perform sectional search to find optimal point between two polygon vertices
     * @param p1 First polygon vertex point
     * @param p2 Second polygon vertex point
     * @param p1_path Path result for first vertex
     * @param p2_path Path result for second vertex
     * @param max_distance Maximum distance constraint
     * @param snappable_ids Vector of snappable (edge or point) IDs that can be snapped to
     * @return ConvexPathResult containing the optimal path found
     */
    ConvexPathResult sectionalSearch(const Point& p1, const Point& p2,
                                   const ConvexPathResult& p1_path,
                                   const ConvexPathResult& p2_path,
                                   double max_distance,
                                   const std::vector<size_t>& snappable_ids);

    /**
     * Get the results from polygon processing
     * @return Vector of tuples containing vertex paths, feature IDs, and least accessible points
     */
    const std::vector<std::tuple<std::vector<ConvexPathResult>, size_t, Point>>& getPolygonResults() const { return polygon_results_; }
    
    /**
     * Set whether to include network distance information in the output
     * @param include_network_distance Flag to include network distance information
     */
    void setIncludeNetworkDistance(bool include_network_distance) { include_network_distance_ = include_network_distance; }
    
    /**
     * Get whether network distance information is included in the output
     * @return true if network distance information is included, false otherwise
     */
    bool getIncludeNetworkDistance() const { return include_network_distance_; }
    
    /**
     * Set the graph vertex snapping tolerance
     * @param tolerance Distance tolerance for snapping vertices (default: 1e-6)
     */
    void setGraphVertexSnappingTolerance(double tolerance) { graph_vertex_snapping_tolerance_ = tolerance; }
    
    /**
     * Get the graph vertex snapping tolerance
     * @return Current graph vertex snapping tolerance
     */
    double getGraphVertexSnappingTolerance() const { return graph_vertex_snapping_tolerance_; }
    
    /**
     * Get access point by index (used in processConvexPathModeNoRoad)
     * @param index Index of the access point
     * @return Optional PointFeature if index is valid, nullopt otherwise
     */
    std::optional<PointFeature> getAccessPoint(size_t index) const;

private:
    /**
     * Initialize and read building dataset using PolygonReader
     * @param building_config Building reader configuration
     * @return true if successful, false otherwise
     */
    bool initializeBuildingDataset(const io::PolygonReaderConfig& building_config);
    
    /**
     * Clear all convex path graph data structures
     */
    void clear();

    /**
     * Clear all polygon split graph data structures
     */
    void clearPolygonSplitGraph();
    
    // Building dataset reader
    std::unique_ptr<io::PolygonReader> building_reader_;
    
    // Flag to track if building dataset has been initialized
    bool building_dataset_initialized_;
    
    // Flag to indicate whether road data is being used (true for processConvexPathMode, false for processConvexPathModeNoRoad)
    bool useRoadData_;
    
    // Flag to include network distance information in output
    bool include_network_distance_;
    
    // Graph vertex snapping tolerance
    double graph_vertex_snapping_tolerance_;
    
    // Convex path graph data structures
    std::vector<ConvexPathGraphVertex> convex_path_vertices_;
    std::vector<ConvexPathGraphEdge> convex_path_edges_;
    std::unordered_set<ConvexPathEdgePair, ConvexPathEdgePairHash> added_convex_path_edges_;
    size_t next_convex_path_vertex_id_;
    
    // R-tree for convex path vertices
    using ConvexPathVertexRTree = bgi::rtree<std::pair<Point, size_t>, bgi::quadratic<16>>;
    ConvexPathVertexRTree convex_path_vertex_rtree_;

    // Polygon split graph data structures
    std::vector<PolygonSplitGraphVertex> polygon_split_vertices_;
    std::vector<PolygonSplitGraphEdge> polygon_split_edges_;
    std::unordered_set<PolygonSplitEdgePair, PolygonSplitEdgePairHash> added_polygon_split_edges_;
    size_t polygon_split_next_vertex_id_;
    
    // R-tree for polygon split vertices
    using PolygonSplitVertexRTree = bgi::rtree<std::pair<Point, size_t>, bgi::quadratic<16>>;
    PolygonSplitVertexRTree polygon_split_vertex_rtree_;
    
    // Results from polygon processing
    std::vector<std::tuple<std::vector<ConvexPathResult>, size_t, Point>> polygon_results_;
    
    // Access points (only used in processConvexPathModeNoRoad, not in processConvexPathMode)
    std::vector<PointFeature> access_points_;
};

} // namespace graph
} // namespace adjfind

#endif // ADJFIND_CONVEX_PATH_HPP
