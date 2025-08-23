cmake_minimum_required(VERSION 3.16)
project(adjfind)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(GDAL REQUIRED)
find_package(nlohmann_json REQUIRED)

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

# Link libraries
target_link_libraries(adjfind
    PRIVATE
    ${GDAL_LIBRARIES}
    Boost::system
    Boost::filesystem
    nlohmann_json::nlohmann_json
)

# Include directories for GDAL
target_include_directories(adjfind PRIVATE ${GDAL_INCLUDE_DIRS})

# Compiler flags
target_compile_definitions(adjfind PRIVATE ${GDAL_DEFINITIONS}) 