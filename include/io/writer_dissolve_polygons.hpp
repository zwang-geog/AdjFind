#ifndef ADJFIND_WRITER_DISSOLVE_POLYGONS_HPP
#define ADJFIND_WRITER_DISSOLVE_POLYGONS_HPP

#include <string>
#include <vector>
#include <gdal.h>
#include <ogr_api.h>
#include "graph/common.hpp"
#include "io/gdal_utils.hpp"

namespace adjfind {
namespace io {

struct DissolvePolygonsWriterConfig {
    std::string output_file_path;
    std::string crs_wkt;
    bool reproject_to_epsg4326;
    
    DissolvePolygonsWriterConfig() : reproject_to_epsg4326(false) {}
};

class DissolvePolygonsWriter {
public:
    DissolvePolygonsWriter();
    ~DissolvePolygonsWriter();
    
    DissolvePolygonsWriter(const DissolvePolygonsWriter&) = delete;
    DissolvePolygonsWriter& operator=(const DissolvePolygonsWriter&) = delete;
    
    bool writeDissolvedPolygons(const DissolvePolygonsWriterConfig& config,
                                const std::vector<graph::Polygon>& polygons);
    
    std::string getLastError() const { return last_error_; }
    void clearError() { last_error_.clear(); }
    
private:
    std::string last_error_;
    
    GDALDatasetH createGDALDataset(const DissolvePolygonsWriterConfig& config,
                                   OGRCoordinateTransformationH& coord_trans,
                                   std::string& output_file_path);
    
    bool writeFeature(OGRLayerH layer,
                      OGRCoordinateTransformationH coord_trans,
                      const graph::Polygon& polygon,
                      size_t feature_id);
};

} // namespace io
} // namespace adjfind

#endif // ADJFIND_WRITER_DISSOLVE_POLYGONS_HPP
