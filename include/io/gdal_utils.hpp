#ifndef ADJFIND_GDAL_UTILS_HPP
#define ADJFIND_GDAL_UTILS_HPP

#include <string>
#include <filesystem>

namespace adjfind {
namespace io {

/**
 * GDAL utility functions for format detection and validation
 */
class GDALUtils {
public:
    /**
     * Check if a specific GDAL driver is available and supports creation
     * @param driver_name GDAL driver name
     * @return true if driver is available and supports creation, false otherwise
     */
    static bool isDriverAvailable(const std::string& driver_name);
    
    /**
     * Determine GDAL output format and modify file path if driver is not available
     * @param file_path File path with extension (will be modified if format fallback occurs)
     * @return GDAL format string
     */
    static std::string determineFormatAndModifyPath(std::string& file_path);

private:
    // Disable instantiation
    GDALUtils() = delete;
};

} // namespace io
} // namespace adjfind

#endif // ADJFIND_GDAL_UTILS_HPP 