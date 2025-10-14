cmake_minimum_required(VERSION 3.16)
project(adjfind VERSION 0.1.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set policies for modern CMake
if(POLICY CMP0167)
    cmake_policy(SET CMP0167 NEW)
endif()

if(POLICY CMP0144)
    cmake_policy(SET CMP0144 NEW)
endif()

# Find required packages
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(GDAL REQUIRED)

# Include directories
include_directories(include)

# Source files
set(SOURCES
    src/main.cpp
    src/io/road_reader.cpp
    src/io/point_reader.cpp
    src/io/polygon_reader.cpp
    src/io/gdal_utils.cpp
    src/io/coordinate_system_utils.cpp
    src/io/writer_road_segmentation.cpp
    src/io/writer_convex_path.cpp
    src/io/writer_neighboring_points.cpp
    src/graph/adj_graph.cpp
    src/graph/road_segmentation.cpp
    src/graph/convex_path.cpp
    src/graph/neighboring_points.cpp
)

# Header files
set(HEADERS
    include/io/road_reader.hpp
    include/io/point_reader.hpp
    include/io/polygon_reader.hpp
    include/io/gdal_utils.hpp
    include/io/coordinate_system_utils.hpp
    include/io/writer_road_segmentation.hpp
    include/io/writer_convex_path.hpp
    include/io/writer_neighboring_points.hpp
    include/graph/adj_graph.hpp
    include/graph/road_segmentation.hpp
    include/graph/common.hpp
    include/graph/convex_path.hpp
    include/graph/neighboring_points.hpp
)

# Create executable
add_executable(adjfind ${SOURCES} ${HEADERS})

# Link libraries using modern CMake targets
target_link_libraries(adjfind
    PRIVATE
    GDAL::GDAL
    Boost::boost
    Boost::filesystem
)

# Enable packaging
include(CPack)
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "AdjFind - Adjacency/Proximity Path Finding Tools")
set(CPACK_PACKAGE_VENDOR "Zifan Wang")
set(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/README.md")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")

# Installation configuration
install(TARGETS adjfind DESTINATION bin)
install(FILES README.md LICENSE DESTINATION share/adjfind)
install(DIRECTORY assets DESTINATION share/adjfind)