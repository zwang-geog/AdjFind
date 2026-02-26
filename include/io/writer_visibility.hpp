#ifndef ADJFIND_WRITER_VISIBILITY_HPP
#define ADJFIND_WRITER_VISIBILITY_HPP

#include "graph/common.hpp"
#include <cstdint>
#include <string>
#include <vector>

namespace adjfind {
namespace io {

/**
 * Writes line-of-sight visibility analysis results to HDF5.
 * Flat layout (all datasets at file root):
 *   /coords             [N×2 float64] road point coordinates (lon/lat if source_crs_wkt set; else raw x,y)
 *   /distances          [N×M float32] Euclidean distance matrix
 *   /visibility_matrix  [N×M uint8]   1 = visible, 0 = not visible
 */
class VisibilityWriter {
public:
    VisibilityWriter() = default;

    /**
     * Write visibility analysis to HDF5 file.
     * Road point coords are transformed to EPSG:4326 (lon, lat) only when source_crs_wkt is non-empty.
     * @param output_path  Path to .h5 file
     * @param source_crs_wkt  Optional WKT of road/point CRS; if non-empty, coords are transformed to 4326
     * @param road_points  N road sample points (x,y in source CRS)
     * @param ignition_points  M ignition points (unused for writing; for size checks)
     * @param distance_matrix  Dense N×M float32
     * @param visibility_matrix  Dense N×M uint8 (1 = visible, 0 = not visible)
     * @return true on success
     */
    bool write(
        const std::string& output_path,
        const std::string& source_crs_wkt,
        const std::vector<graph::Point>& road_points,
        const std::vector<graph::Point>& ignition_points,
        const std::vector<float>& distance_matrix,
        const std::vector<uint8_t>& visibility_matrix);
};

} // namespace io
} // namespace adjfind

#endif // ADJFIND_WRITER_VISIBILITY_HPP
