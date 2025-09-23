#ifndef ADJFIND_WASM_INTERFACE_HPP
#define ADJFIND_WASM_INTERFACE_HPP

#include <string>

namespace wasm_interface {

/**
 * Road Segmentation Tool
 * Processes road segmentation with distance breakpoints
 * @param writer_config_json JSON string for writer configuration
 * @param road_config_json JSON string for road reader configuration
 * @param point_config_json JSON string for point reader configuration
 * @param road_segmentation_config_json JSON string for road segmentation configuration (distance_breakpoints_str)
 * @return Result message (success or error)
 */
std::string processRoadSegmentationTool(
    const std::string& writer_config_json,
    const std::string& road_config_json,
    const std::string& point_config_json,
    const std::string& road_segmentation_config_json
);

/**
 * Convex Path Tool (Structure Access)
 * Processes convex path analysis for structure access
 * @param writer_config_json JSON string for writer configuration
 * @param road_config_json JSON string for road reader configuration
 * @param point_config_json JSON string for point reader configuration
 * @param building_config_json JSON string for building reader configuration
 * @return Result message (success or error)
 */
std::string processConvexPathTool(
    const std::string& writer_config_json,
    const std::string& road_config_json,
    const std::string& point_config_json,
    const std::string& building_config_json
);

/**
 * Neighboring Points Tool
 * Processes neighboring points analysis
 * @param writer_config_json JSON string for writer configuration
 * @param road_config_json JSON string for road reader configuration
 * @param point_config_json JSON string for point reader configuration
 * @param neighboring_points_config_json JSON string for neighboring points configuration (intersection_vertex_distance_threshold, cutoff)
 * @return Result message (success or error)
 */
std::string processNeighboringPointsTool(
    const std::string& writer_config_json,
    const std::string& road_config_json,
    const std::string& point_config_json,
    const std::string& neighboring_points_config_json
);


} // namespace wasm_interface

#endif // ADJFIND_WASM_INTERFACE_HPP
