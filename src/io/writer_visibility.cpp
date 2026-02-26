#include "io/writer_visibility.hpp"
#include "graph/common.hpp"
#include <hdf5.h>
#include <boost/geometry.hpp>
#include <gdal.h>
#include <ogr_api.h>
#include <ogr_spatialref.h>
#include <cstring>
#include <iostream>

namespace adjfind {
namespace io {

namespace bg = boost::geometry;

bool VisibilityWriter::write(
    const std::string& output_path,
    const std::string& source_crs_wkt,
    const std::vector<graph::Point>& road_points,
    const std::vector<graph::Point>& ignition_points,
    const std::vector<float>& distance_matrix,
    const std::vector<uint8_t>& visibility_matrix) {
    const size_t N = road_points.size();
    const size_t M = ignition_points.size();
    if (distance_matrix.size() != N * M) {
        std::cerr << "Error: Distance matrix size mismatch" << std::endl;
        return false;
    }
    if (visibility_matrix.size() != N * M) {
        std::cerr << "Error: Visibility matrix size mismatch" << std::endl;
        return false;
    }
    std::vector<double> coords(N * 2);
    for (size_t i = 0; i < N; ++i) {
        coords[i * 2] = bg::get<0>(road_points[i]);
        coords[i * 2 + 1] = bg::get<1>(road_points[i]);
    }
    if (!source_crs_wkt.empty()) {
        OGRCoordinateTransformationH coord_trans = nullptr;
        OGRSpatialReferenceH source_srs = OSRNewSpatialReference(nullptr);
        OGRSpatialReferenceH target_srs = OSRNewSpatialReference(nullptr);
        char* wkt_copy = const_cast<char*>(source_crs_wkt.c_str());
        if (OSRImportFromWkt(source_srs, &wkt_copy) == OGRERR_NONE) {
            OSRImportFromEPSG(target_srs, 4326);
            OSRSetAxisMappingStrategy(source_srs, OAMS_TRADITIONAL_GIS_ORDER);
            OSRSetAxisMappingStrategy(target_srs, OAMS_TRADITIONAL_GIS_ORDER);
            coord_trans = OCTNewCoordinateTransformation(source_srs, target_srs);
            if (coord_trans) {
                for (size_t i = 0; i < N; ++i) {
                    double x = coords[i * 2], y = coords[i * 2 + 1];
                    OGRGeometryH pt = OGR_G_CreateGeometry(wkbPoint);
                    OGR_G_SetPoint_2D(pt, 0, x, y);
                    if (OGR_G_Transform(pt, coord_trans) == OGRERR_NONE) {
                        coords[i * 2] = OGR_G_GetX(pt, 0);
                        coords[i * 2 + 1] = OGR_G_GetY(pt, 0);
                    }
                    OGR_G_DestroyGeometry(pt);
                }
                OCTDestroyCoordinateTransformation(coord_trans);
            }
        }
        OSRDestroySpatialReference(source_srs);
        OSRDestroySpatialReference(target_srs);
    }

    hid_t file_id = H5Fcreate(output_path.c_str(), H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
    if (file_id < 0) {
        std::cerr << "Error: Failed to create HDF5 file: " << output_path << std::endl;
        return false;
    }

    // coords [N×2] — road point coordinates (lon/lat or raw x,y)
    hsize_t dims_coords[2] = { N, 2 };
    hid_t space_coords = H5Screate_simple(2, dims_coords, nullptr);
    if (space_coords < 0) {
        std::cerr << "Error: Failed to create dataspace for coords" << std::endl;
        H5Fclose(file_id);
        return false;
    }
    hid_t dset_coords = H5Dcreate2(file_id, "coords", H5T_NATIVE_DOUBLE, space_coords,
                                    H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    H5Sclose(space_coords);
    if (dset_coords < 0) {
        std::cerr << "Error: Failed to create coords dataset" << std::endl;
        H5Fclose(file_id);
        return false;
    }
    if (H5Dwrite(dset_coords, H5T_NATIVE_DOUBLE, H5S_ALL, H5S_ALL, H5P_DEFAULT, coords.data()) < 0) {
        std::cerr << "Error: Failed to write coords" << std::endl;
        H5Dclose(dset_coords);
        H5Fclose(file_id);
        return false;
    }
    H5Dclose(dset_coords);

    // distances [N×M] — Euclidean distance matrix
    hsize_t dims_dist[2] = { N, M };
    hid_t space_dist = H5Screate_simple(2, dims_dist, nullptr);
    if (space_dist < 0) {
        std::cerr << "Error: Failed to create dataspace for distances" << std::endl;
        H5Fclose(file_id);
        return false;
    }
    hid_t dset_dist = H5Dcreate2(file_id, "distances", H5T_NATIVE_FLOAT, space_dist,
                                 H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    H5Sclose(space_dist);
    if (dset_dist < 0) {
        std::cerr << "Error: Failed to create distances dataset" << std::endl;
        H5Fclose(file_id);
        return false;
    }
    if (H5Dwrite(dset_dist, H5T_NATIVE_FLOAT, H5S_ALL, H5S_ALL, H5P_DEFAULT, distance_matrix.data()) < 0) {
        std::cerr << "Error: Failed to write distances" << std::endl;
        H5Dclose(dset_dist);
        H5Fclose(file_id);
        return false;
    }
    H5Dclose(dset_dist);

    // visibility_matrix [N×M] — 1 = visible, 0 = not visible
    hsize_t dims_vis[2] = { N, M };
    hid_t space_vis = H5Screate_simple(2, dims_vis, nullptr);
    if (space_vis < 0) {
        std::cerr << "Error: Failed to create dataspace for visibility_matrix" << std::endl;
        H5Fclose(file_id);
        return false;
    }
    hid_t dset_vis = H5Dcreate2(file_id, "visibility_matrix", H5T_NATIVE_UCHAR, space_vis,
                                H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    H5Sclose(space_vis);
    if (dset_vis < 0) {
        std::cerr << "Error: Failed to create visibility_matrix dataset" << std::endl;
        H5Fclose(file_id);
        return false;
    }
    if (H5Dwrite(dset_vis, H5T_NATIVE_UCHAR, H5S_ALL, H5S_ALL, H5P_DEFAULT, visibility_matrix.data()) < 0) {
        std::cerr << "Error: Failed to write visibility_matrix" << std::endl;
        H5Dclose(dset_vis);
        H5Fclose(file_id);
        return false;
    }
    H5Dclose(dset_vis);

    if (H5Fclose(file_id) < 0) {
        std::cerr << "Error: Failed to close HDF5 file" << std::endl;
        return false;
    }
    return true;
}

} // namespace io
} // namespace adjfind
