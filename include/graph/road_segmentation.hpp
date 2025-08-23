#ifndef ADJFIND_ROAD_SEGMENTATION_HPP
#define ADJFIND_ROAD_SEGMENTATION_HPP

#include "graph/adj_graph.hpp"
#include "io/road_reader.hpp"
#include "io/point_reader.hpp"

namespace adjfind {
namespace graph {

class RoadSegmentation : public AdjGraph {
public:
    RoadSegmentation();
    ~RoadSegmentation() = default;
    
    // Disable copy constructor and assignment
    RoadSegmentation(const RoadSegmentation&) = delete;
    RoadSegmentation& operator=(const RoadSegmentation&) = delete;
    
    /**
     * Process road segmentation mode with road and point configurations
     * This method implements the complete road segmentation workflow
     * @param road_config Road reader configuration
     * @param point_config Point reader configuration
     * @param distance_breakpoints Vector of distance values to split at
     * @return Vector of RoadSplitByDistanceBracketsOutput structures
     */
    std::vector<RoadSplitByDistanceBracketsOutput> processRoadSegmentationMode(const io::RoadReaderConfig& road_config, const io::PointReaderConfig& point_config, const std::vector<double>& distance_breakpoints);
    
    /**
     * Populate distance to point vertex for all vertices
     * This method implements the modified Dijkstra algorithm as specified
     */
    void populateDistanceToPointVertex();
    
    /**
     * Split linestrings at equilibrium points
     * This method splits edges where vertices have different nearest point vertices
     * and creates new vertices at equilibrium points
     */
    void splitLinestringAtEquilibrium();
    
    /**
     * Split road linestrings by distance breakpoints
     * This method splits edges based on distance breakpoints and returns output structures
     * @param distance_breakpoints Vector of distance values to split at
     * @return Vector of RoadSplitByDistanceBracketsOutput structures
     */
    std::vector<RoadSplitByDistanceBracketsOutput> splitRoadLinestringsByDistanceVector(const std::vector<double>& distance_breakpoints);
    
    /**
     * Extract edge information without distance splitting
     * This method loops through all edges and extracts information for output
     * @return Vector of RoadSplitByDistanceBracketsOutput structures
     */
    std::vector<RoadSplitByDistanceBracketsOutput> extractEdgeInformation();
    
private:
    /**
     * Modified Dijkstra algorithm for finding nearest point vertex
     * @param start_vertex Starting vertex ID
     * @return Pair of (distance, nearest_point_vertex_id)
     */
    std::pair<double, size_t> findNearestPointVertex(size_t start_vertex);
    

};

} // namespace graph
} // namespace adjfind

#endif // ADJFIND_ROAD_SEGMENTATION_HPP 