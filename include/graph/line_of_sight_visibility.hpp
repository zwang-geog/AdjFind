#ifndef ADJFIND_LINE_OF_SIGHT_VISIBILITY_HPP
#define ADJFIND_LINE_OF_SIGHT_VISIBILITY_HPP

#include "graph/common.hpp"
#include "io/road_reader.hpp"
#include "io/point_reader.hpp"
#include <gdal.h>
#include <cstdint>
#include <string>
#include <vector>
#include <utility>

namespace adjfind {
namespace graph {

/**
 * Line-of-sight visibility analysis for wildfire ignition points.
 * Samples road network, computes distance matrix and visibility (LOS) vs DEM,
 * for optimal placement of fire detection resources.
 */
class LineOfSightVisibility {
public:
    LineOfSightVisibility() = default;

    /**
     * Run the full pipeline: road sampling, distance matrix, visibility matrix.
     * @param road_config  Road network reader config
     * @param ignition_config  Ignition points reader config
     * @param dem_path  Path to DEM GeoTIFF
     * @param road_sample_interval  Spacing between road samples (same units as CRS)
     * @param visibility_range  Max distance for LOS checks (same units as CRS)
     * @param min_road_length_to_include  Roads shorter than this are skipped (same units as CRS, default 20)
     * @param road_points_output  Road sample points (x,y) after sampling
     * @param ignition_points_output  Ignition point coordinates (x,y)
     * @param distance_matrix_output  Dense N x M (road x ignition) distances
     * @param visibility_matrix_output  Dense N x M uint8 (1 = visible, 0 = not visible)
     * @return true on success
     */
    bool run(
        const io::RoadReaderConfig& road_config,
        const io::PointReaderConfig& ignition_config,
        const std::string& dem_path,
        double road_sample_interval,
        double visibility_range,
        double min_road_length_to_include,
        std::vector<Point>& road_points_output,
        std::vector<Point>& ignition_points_output,
        std::vector<float>& distance_matrix_output,
        std::vector<uint8_t>& visibility_matrix_output);

    /**
     * Get the coordinate system WKT of the road (and point) data used in run().
     * Used to transform road point coords to EPSG:4326 (lon/lat) when writing output.
     */
    const std::string& getCoordinateSystemWKT() const { return coordinate_system_wkt_; }

private:
    std::string coordinate_system_wkt_;

    /**
     * Sample road linestrings at fixed interval; short segments use midpoint.
     * Roads with length < min_road_length_to_include are skipped entirely.
     */
    static void sampleRoadPoints(
        const std::vector<RoadFeature>& roads,
        double interval,
        double min_road_length_to_include,
        std::vector<Point>& out);

    /**
     * Compute dense N x M Euclidean distance matrix (road point to ignition point).
     */
    static void computeDistanceMatrix(
        const std::vector<Point>& road_points,
        const std::vector<Point>& ignition_points,
        std::vector<float>& out);

    /**
     * Line-of-sight check between two points using preloaded DEM buffer.
     * Returns true if observer can see target (no terrain blocking).
     */
    static bool lineOfSight(
        const float* elevations,
        int dem_width,
        int dem_height,
        const double* geo_transform,
        double x1, double y1,
        double x2, double y2,
        double observer_height = 0.0,
        double target_height = 0.0);

    /**
     * Compute dense N x M visibility matrix (1 = visible, 0 = not visible).
     * Uses preloaded DEM buffer for efficient elevation lookups.
     */
    void computeVisibilityMatrix(
        const float* elevations,
        int dem_width,
        int dem_height,
        const double* geo_transform,
        const std::vector<Point>& road_points,
        const std::vector<Point>& ignition_points,
        const std::vector<float>& distance_matrix,
        double visibility_range,
        std::vector<uint8_t>& visibility_matrix);
};

} // namespace graph
} // namespace adjfind

#endif // ADJFIND_LINE_OF_SIGHT_VISIBILITY_HPP
