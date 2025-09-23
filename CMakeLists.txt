cmake_minimum_required(VERSION 3.16)
project(adjfind VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set policy for Boost
if(POLICY CMP0167)
    cmake_policy(SET CMP0167 NEW)
endif()

# Detect WASM build
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang" AND CMAKE_CXX_COMPILER_FRONTEND_VARIANT MATCHES "GNU")
    execute_process(
        COMMAND ${CMAKE_CXX_COMPILER} --version
        OUTPUT_VARIABLE COMPILER_VERSION
        ERROR_VARIABLE COMPILER_ERROR
    )
    if(COMPILER_VERSION MATCHES "emcc")
        set(IS_WASM_BUILD TRUE)
        message(STATUS "Building for WebAssembly (Emscripten)")
    endif()
endif()

# Find required packages
if(IS_WASM_BUILD)
    # WASM build - GDAL-free architecture
    message(STATUS "Building GDAL-free WASM version...")
    
    # For WASM builds, we'll use the nlohmann_json as a header-only library
    # Always download the header-only version for WASM builds to avoid compatibility issues
    message(STATUS "Using header-only nlohmann_json for WASM build...")
    
    # Create directory for external dependencies with proper structure
    set(JSON_INCLUDE_DIR "${CMAKE_BINARY_DIR}")
    file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/nlohmann")
    
    # Download the single header file to the correct location
    file(DOWNLOAD
        "https://github.com/nlohmann/json/releases/download/v3.12.0/json.hpp"
        "${CMAKE_BINARY_DIR}/nlohmann/json.hpp"
        SHOW_PROGRESS
    )
    
    # Add the include directory globally for WASM build
    include_directories(${JSON_INCLUDE_DIR})
    
    # Create an interface library for nlohmann_json
    add_library(nlohmann_json INTERFACE)
    target_include_directories(nlohmann_json INTERFACE ${CMAKE_BINARY_DIR})
    add_library(nlohmann_json::nlohmann_json ALIAS nlohmann_json)
    
    # Boost libraries for WASM (minimal dependencies)
    add_library(boost_system_wasm INTERFACE)
    add_library(boost_filesystem_wasm INTERFACE)
    add_compile_definitions(-DBOOST_NO_CXX98_FUNCTION_BASE)
    
    message(STATUS "GDAL-free WASM build configured")
else()
    # Native build - GDAL-free architecture
    find_package(Boost REQUIRED COMPONENTS system filesystem)
    find_package(nlohmann_json REQUIRED)
endif()

# Include directories
include_directories(include)

# Source files
set(SOURCES
    src/main.cpp
    src/io/road_reader.cpp
    src/io/point_reader.cpp
    src/io/polygon_reader.cpp
    src/io/writer_road_segmentation.cpp
    src/io/writer_convex_path.cpp
    src/io/writer_neighboring_points.cpp
    src/io/geojson_reader.cpp
    src/io/geojson_writer.cpp
    src/graph/adj_graph.cpp
    src/graph/geometry_conversion.cpp
    src/graph/road_segmentation.cpp
    src/graph/convex_path.cpp
    src/graph/neighboring_points.cpp
)

# Add WASM interface source for both builds (useful for testing)
list(APPEND SOURCES src/wasm/wasm_interface.cpp)

# Header files
set(HEADERS
    include/io/road_reader.hpp
    include/io/point_reader.hpp
    include/io/polygon_reader.hpp
    include/io/writer_road_segmentation.hpp
    include/io/writer_convex_path.hpp
    include/io/writer_neighboring_points.hpp
    include/io/geojson_reader.hpp
    include/io/geojson_writer.hpp
    include/graph/adj_graph.hpp
    include/graph/road_segmentation.hpp
    include/graph/common.hpp
    include/graph/convex_path.hpp
    include/graph/neighboring_points.hpp
)

# Add WASM interface header for both builds (useful for testing)
list(APPEND HEADERS include/wasm/wasm_interface.hpp)

# Create executable/library
if(IS_WASM_BUILD)
    # WASM build - create executable (will generate .js and .wasm files)
    add_executable(adjfind_wasm ${SOURCES} ${HEADERS})
    set_target_properties(adjfind_wasm PROPERTIES OUTPUT_NAME "adjfind_wasm")
    
    # Link libraries for WASM (GDAL-free)
    target_link_libraries(adjfind_wasm
        PRIVATE
        boost_system_wasm
        boost_filesystem_wasm
    )
    
    # Link nlohmann_json (always available as header-only for WASM)
    target_link_libraries(adjfind_wasm PRIVATE nlohmann_json::nlohmann_json)
    
    # Ensure JSON include directory is available to the target
    target_include_directories(adjfind_wasm PRIVATE ${CMAKE_BINARY_DIR})
    
    # Emscripten settings
    set_target_properties(adjfind_wasm PROPERTIES
        LINK_FLAGS "-s WASM=1 -s EXPORTED_RUNTIME_METHODS=['ccall','cwrap','FS','UTF8ToString','allocateUTF8'] -s ALLOW_MEMORY_GROWTH=1 -s INITIAL_MEMORY=536870912 -s MAXIMUM_MEMORY=1073741824 -s ASSERTIONS=2 -s SAFE_HEAP=1 -s STACK_OVERFLOW_CHECK=2 -s TOTAL_STACK=67108864 -gsource-map -s EXPORTED_FUNCTIONS=['_processRoadSegmentationTool','_processConvexPathTool','_processNeighboringPointsTool'] -s EXPORT_NAME='AdjFindModule' -sNO_DISABLE_EXCEPTION_CATCHING=1"
    )
else()
    # Native build - create executable
    add_executable(adjfind ${SOURCES} ${HEADERS})
    
    # Link libraries for native (GDAL-free)
    target_link_libraries(adjfind
        PRIVATE
        Boost::system
        Boost::filesystem
        nlohmann_json::nlohmann_json
    )
endif()

# Include directories and compiler flags
if(IS_WASM_BUILD)
    # GDAL-free WASM build - no GDAL includes needed
    # Add Emscripten system includes
    target_include_directories(adjfind_wasm SYSTEM PRIVATE $ENV{EMSDK}/upstream/emscripten/system/include)
else()
    # Native build - GDAL-free architecture
    # No GDAL includes or definitions needed
endif()

# Enable packaging
include(CPack)
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "AdjFind - Transportation Network Algorithms")
set(CPACK_PACKAGE_VENDOR "Zifan Wang")
set(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/README.md")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")

# Installation configuration
if(IS_WASM_BUILD)
    install(TARGETS adjfind_wasm DESTINATION lib)
    install(FILES README.md LICENSE WASM_INTERFACE_README.md DESTINATION share/adjfind)
    install(DIRECTORY assets DESTINATION share/adjfind)
    install(DIRECTORY web DESTINATION share/adjfind)
else()
    install(TARGETS adjfind DESTINATION bin)
    install(FILES README.md LICENSE DESTINATION share/adjfind)
    install(DIRECTORY assets DESTINATION share/adjfind)
endif()