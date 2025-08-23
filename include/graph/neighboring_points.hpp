#ifndef ADJFIND_NEIGHBORING_POINTS_HPP
#define ADJFIND_NEIGHBORING_POINTS_HPP

#include <vector>
#include <optional>
#include <tuple>
#include <queue>
#include <algorithm>
#include <limits>
#include <unordered_map>
#include <boost/geometry.hpp>
#include "graph/adj_graph.hpp"

namespace adjfind {
namespace graph {

namespace bg = boost::geometry;

/**
 * Class for finding neighboring points in a graph
 * Inherits from AdjGraph to provide graph functionality
 */
class NeighboringPoints : public AdjGraph {
public:
    NeighboringPoints() = default;
    ~NeighboringPoints() = default;
    
    // Disable copy constructor and assignment
    NeighboringPoints(const NeighboringPoints&) = delete;
    NeighboringPoints& operator=(const NeighboringPoints&) = delete;
    
    /**
     * Find path from a source vertex to the closest point vertex
     * @param source Source vertex ID
     * @param cutoff Optional distance cutoff for the search
     * @param edges_to_exclude Vector of edge IDs to exclude from the search
     * @return Tuple containing (feature_id, distance, path_geometry) or default values if no path found
     */
    std::tuple<size_t, double, MultiLineString> findPathToClosestPointVertex(
        size_t source,
        std::optional<double> cutoff,
        const std::vector<size_t>& edges_to_exclude) const;

    /**
     * Find neighboring points for a given vertex with intersection vertex handling
     * @param source_vertex_index Source vertex ID
     * @param intersection_vertex_distance_threshold Distance threshold for treating vertices as intersection vertices
     * @param cutoff Optional distance cutoff for the search
     * @return Pair containing (source_feature_id, vector of path results)
     */
    std::pair<size_t, std::vector<std::tuple<size_t, double, MultiLineString>>> findNeighboringPointsForAGivenVertex(
        size_t source_vertex_index,
        double intersection_vertex_distance_threshold, 
        std::optional<double> cutoff) const;

    /**
     * Process neighboring points mode with road and point configurations
     * This method implements the complete neighboring points workflow
     * @param road_config Road reader configuration
     * @param point_config Point reader configuration
     * @param intersection_vertex_distance_threshold Distance threshold for treating vertices as intersection vertices
     * @param cutoff Optional distance cutoff for the search
     * @return true if successful, false otherwise
     */
    bool processNeighboringPoints(const io::RoadReaderConfig& road_config, 
                                 const io::PointReaderConfig& point_config,
                                 double intersection_vertex_distance_threshold,
                                 std::optional<double> cutoff = std::nullopt);

    /**
     * Get the results from neighboring points processing
     * @return Map of vertex pairs to shortest paths
     */
    const std::unordered_map<std::pair<size_t, size_t>, std::tuple<size_t, double, MultiLineString>, PairHash>& getNeighboringPointsResults() const { return neighboring_points_results_; }

private:
    // Helper function to build path from source to a given vertex
    MultiLineString buildPath(
        size_t end,
        size_t source,
        const std::vector<size_t>& predecessors_vertex,
        const std::vector<size_t>& predecessors_edge) const;

    // Results from neighboring points processing - deduplicated map of vertex pairs to shortest paths
    std::unordered_map<std::pair<size_t, size_t>, std::tuple<size_t, double, MultiLineString>, PairHash> neighboring_points_results_;
};

} // namespace graph
} // namespace adjfind

#endif // ADJFIND_NEIGHBORING_POINTS_HPP
