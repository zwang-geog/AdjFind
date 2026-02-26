#include "graph/line_of_sight_visibility.hpp"
#include "io/road_reader.hpp"
#include "io/point_reader.hpp"
#include <gdal.h>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <limits>
#include <utility>
#include <vector>

namespace adjfind {
namespace graph {

namespace bg = boost::geometry;

namespace {

// Bresenham's line algorithm: append every (pixel_x, pixel_y) the line passes through.
void bresenhamLine(int px0, int py0, int px1, int py1, std::vector<std::pair<int, int>>& out) {
    out.clear();
    int dx = std::abs(px1 - px0);
    int dy = std::abs(py1 - py0);
    int sx = (px0 < px1) ? 1 : -1;
    int sy = (py0 < py1) ? 1 : -1;
    if (dx >= dy) {
        int err = 2 * dy - dx;
        int x = px0, y = py0;
        for (int i = 0; i <= dx; ++i) {
            out.emplace_back(x, y);
            if (x == px1 && y == py1) break;
            if (err > 0) { y += sy; err -= 2 * dx; }
            err += 2 * dy;
            x += sx;
        }
    } else {
        int err = 2 * dx - dy;
        int x = px0, y = py0;
        for (int i = 0; i <= dy; ++i) {
            out.emplace_back(x, y);
            if (x == px1 && y == py1) break;
            if (err > 0) { x += sx; err -= 2 * dy; }
            err += 2 * dx;
            y += sy;
        }
    }
}

} // anonymous namespace

void LineOfSightVisibility::sampleRoadPoints(
    const std::vector<RoadFeature>& roads,
    double interval,
    double min_road_length_to_include,
    std::vector<Point>& out) {
    out.clear();
    if (interval <= 0) return;
    const double short_threshold = interval * 1.5;
    for (const auto& road : roads) {
        if (road.length < min_road_length_to_include) continue;
        const LineString& ls = road.geometry;
        if (ls.empty()) continue;
        if (ls.size() == 1) continue;
        const double total_length = road.length;
        if (total_length <= short_threshold) {
            Point midpoint;
            bg::line_interpolate(ls, total_length * 0.5, midpoint);
            out.push_back(midpoint);
            continue;
        }
        MultiPoint interpolated;
        bg::line_interpolate(ls, interval, interpolated);
        for (const auto& p : interpolated) {
            out.push_back(p);
        }
    }
}

void LineOfSightVisibility::computeDistanceMatrix(
    const std::vector<Point>& road_points,
    const std::vector<Point>& ignition_points,
    std::vector<float>& out) {
    const size_t N = road_points.size();
    const size_t M = ignition_points.size();
    out.resize(N * M);
    for (size_t i = 0; i < N; ++i) {
        const Point& p = road_points[i];
        for (size_t j = 0; j < M; ++j) {
            out[i * M + j] = static_cast<float>(bg::distance(p, ignition_points[j]));
        }
    }
}

bool LineOfSightVisibility::lineOfSight(
    const float* elevations,
    int dem_width,
    int dem_height,
    const double* geo_transform,
    double x1, double y1,
    double x2, double y2,
    double observer_height,
    double target_height) {
    // geo_transform: [0]=origin X, [1]=pixel width, [2]=row rotation, [3]=origin Y, [4]=col rotation, [5]=pixel height
    auto worldToPixel = [&](double x, double y, int& px, int& py) {
        double pxf = (x - geo_transform[0]) / geo_transform[1];
        double pyf = (y - geo_transform[3]) / geo_transform[5];
        px = static_cast<int>(pxf);
        py = static_cast<int>(pyf);
    };
    auto pixelCenterToWorld = [&](int px, int py, double& x, double& y) {
        x = geo_transform[0] + (px + 0.5) * geo_transform[1];
        y = geo_transform[3] + (py + 0.5) * geo_transform[5];
    };
    int px1, py1, px2, py2;
    worldToPixel(x1, y1, px1, py1);
    worldToPixel(x2, y2, px2, py2);
    if (px1 < 0 || px1 >= dem_width || py1 < 0 || py1 >= dem_height ||
        px2 < 0 || px2 >= dem_width || py2 < 0 || py2 >= dem_height) {
        std::cerr << "Line-of-sight lookup out of range (x1,y1)=(" << x1 << "," << y1 << ") -> (px1,py1)=(" << px1 << "," << py1 << ")" << std::endl;
        std::cerr << "Line-of-sight lookup out of range (x2,y2)=(" << x2 << "," << y2 << ") -> (px2,py2)=(" << px2 << "," << py2 << ")" << std::endl;
        std::cerr << "DEM width: " << dem_width << ", DEM height: " << dem_height << std::endl;
        std::cerr << "Geo transform: " << geo_transform[0] << ", " << geo_transform[1] << ", " << geo_transform[2] << ", " << geo_transform[3] << ", " << geo_transform[4] << ", " << geo_transform[5] << std::endl;
        return false;
    }
    if (px1 == px2 && py1 == py2) return true;
    // Sight line in world space: true observer (x1,y1) to true target (x2,y2); z1/z2 from DEM at containing cells
    double z1 = static_cast<double>(elevations[py1 * dem_width + px1]) + observer_height;
    double z2 = static_cast<double>(elevations[py2 * dem_width + px2]) + target_height;
    double dx = x2 - x1;
    double dy = y2 - y1;
    double dist_sq = dx * dx + dy * dy;
    if (dist_sq <= 0) return true;
    std::vector<std::pair<int, int>> pixels;
    bresenhamLine(px1, py1, px2, py2, pixels);
    if (pixels.size() < 3) return true;
    // For each inner cell: use cell-center world coords to get t (distance-based), then expected_z on sight line
    for (size_t i = 1; i + 1 < pixels.size(); ++i) {
        int px = pixels[i].first;
        int py = pixels[i].second;
        if (px < 0 || px >= dem_width || py < 0 || py >= dem_height) continue;
        double actual_z = static_cast<double>(elevations[py * dem_width + px]);
        double x, y;
        pixelCenterToWorld(px, py, x, y);
        double t = ((x - x1) * dx + (y - y1) * dy) / dist_sq;
        t = std::max(0.0, std::min(1.0, t));
        double expected_z = z1 + t * (z2 - z1);
        if (actual_z > expected_z) return false;
    }
    return true;
}

void LineOfSightVisibility::computeVisibilityMatrix(
    const float* elevations,
    int dem_width,
    int dem_height,
    const double* geo_transform,
    const std::vector<Point>& road_points,
    const std::vector<Point>& ignition_points,
    const std::vector<float>& distance_matrix,
    double visibility_range,
    std::vector<uint8_t>& visibility_matrix) {
    const size_t N = road_points.size();
    const size_t M = ignition_points.size();
    visibility_matrix.assign(N * M, 0);
    for (size_t i = 0; i < N; ++i) {
        double x1 = bg::get<0>(road_points[i]);
        double y1 = bg::get<1>(road_points[i]);
        for (size_t j = 0; j < M; ++j) {
            float d = distance_matrix[i * M + j];
            if (d > visibility_range) continue;
            double x2 = bg::get<0>(ignition_points[j]);
            double y2 = bg::get<1>(ignition_points[j]);
            if (lineOfSight(elevations, dem_width, dem_height, geo_transform, x1, y1, x2, y2, 0.0, 0.0)) {
                visibility_matrix[i * M + j] = 1;
            }
        }
    }
}

bool LineOfSightVisibility::run(
    const io::RoadReaderConfig& road_config,
    const io::PointReaderConfig& ignition_config,
    const std::string& dem_path,
    double road_sample_interval,
    double visibility_range,
    double min_road_length_to_include,
    std::vector<Point>& road_points_output,
    std::vector<Point>& ignition_points_output,
    std::vector<float>& distance_matrix_output,
    std::vector<uint8_t>& visibility_matrix_output) {
    GDALAllRegister();
    io::RoadReader road_reader(road_config);
    if (!road_reader.read()) {
        std::cerr << "Error: Failed to read road network" << std::endl;
        return false;
    }
    io::PointReader point_reader(ignition_config);
    if (!point_reader.read()) {
        std::cerr << "Error: Failed to read ignition points" << std::endl;
        return false;
    }
    // Check coordinate system compatibility
    int road_epsg = road_reader.getCoordinateSystemEPSG();
    int point_epsg = point_reader.getCoordinateSystemEPSG();
    if (road_epsg != point_epsg) {
        std::cerr << "Error: Coordinate system mismatch!" << std::endl;
        std::cerr << "  Road dataset: EPSG:" << road_epsg << std::endl;
        std::cerr << "  Point dataset: EPSG:" << point_epsg << std::endl;
        std::cerr << "  Both datasets must have the same coordinate system." << std::endl;
        return false;
    }
    std::cout << "Coordinate systems match: EPSG:" << road_epsg << std::endl;
    coordinate_system_wkt_ = road_reader.getCoordinateSystemWKT();
    sampleRoadPoints(road_reader.getRoadFeatures(), road_sample_interval, min_road_length_to_include, road_points_output);
    ignition_points_output.clear();
    for (const auto& pf : point_reader.getPointFeatures()) {
        ignition_points_output.push_back(pf.geometry);
    }
    if (road_points_output.empty()) {
        std::cerr << "Error: No road sample points produced" << std::endl;
        return false;
    }
    if (ignition_points_output.empty()) {
        std::cerr << "Error: No ignition points" << std::endl;
        return false;
    }
    std::cout << "Road sample points: " << road_points_output.size()
              << ", Ignition points: " << ignition_points_output.size() << std::endl;
    computeDistanceMatrix(road_points_output, ignition_points_output, distance_matrix_output);
    GDALDatasetH dem_dataset = GDALOpen(dem_path.c_str(), GA_ReadOnly);
    if (!dem_dataset) {
        std::cerr << "Error: Failed to open DEM: " << dem_path << std::endl;
        return false;
    }
    int dem_width = GDALGetRasterXSize(dem_dataset);
    int dem_height = GDALGetRasterYSize(dem_dataset);
    double geo_transform[6];
    if (GDALGetGeoTransform(dem_dataset, geo_transform) != CE_None) {
        std::cerr << "Error: Failed to get DEM geotransform" << std::endl;
        GDALClose(dem_dataset);
        return false;
    }
    std::vector<float> elevations(static_cast<size_t>(dem_width) * static_cast<size_t>(dem_height));
    GDALRasterBandH band = GDALGetRasterBand(dem_dataset, 1);
    if (!band || GDALRasterIO(band, GF_Read, 0, 0, dem_width, dem_height,
                              elevations.data(), dem_width, dem_height, GDT_Float32, 0, 0) != CE_None) {
        std::cerr << "Error: Failed to read DEM band (one-time load)" << std::endl;
        GDALClose(dem_dataset);
        return false;
    }
    GDALClose(dem_dataset);
    std::cout << "DEM preloaded: " << dem_width << " x " << dem_height << " (" << (elevations.size() * sizeof(float) / (1024 * 1024)) << " MB)" << std::endl;
    computeVisibilityMatrix(
        elevations.data(), dem_width, dem_height, geo_transform,
        road_points_output,
        ignition_points_output,
        distance_matrix_output,
        visibility_range,
        visibility_matrix_output);
    return true;
}

} // namespace graph
} // namespace adjfind
