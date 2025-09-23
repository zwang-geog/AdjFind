cmake_minimum_required(VERSION 3.16)
project(adjfind VERSION 1.0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set policy for Boost
if(POLICY CMP0167)
    cmake_policy(SET CMP0167 NEW)
endif()

# Find required packages
find_package(GDAL REQUIRED)
find_package(nlohmann_json REQUIRED)

# Manual Boost setup for AppleClang 17.0.0 compatibility
if(APPLE)
    message(STATUS "=== Debugging Boost detection on macOS ===")
    
    # Check what's in Homebrew lib directory
    execute_process(
        COMMAND ls -la /opt/homebrew/lib/libboost*
        OUTPUT_VARIABLE BOOST_LIBS_LIST
        ERROR_VARIABLE BOOST_LIBS_ERROR
    )
    message(STATUS "Boost libraries in /opt/homebrew/lib:")
    message(STATUS "${BOOST_LIBS_LIST}")
    
    # Find Boost headers manually
    find_path(BOOST_INCLUDE_DIR boost/version.hpp
        PATHS
        /opt/homebrew/include
        /usr/local/include
        /usr/include
    )
    message(STATUS "BOOST_INCLUDE_DIR: ${BOOST_INCLUDE_DIR}")
    
    # Try different library name patterns
    find_library(BOOST_SYSTEM_LIB 
        NAMES boost_system libboost_system boost_system-mt libboost_system-mt
        PATHS
        /opt/homebrew/lib
        /usr/local/lib
        /usr/lib
    )
    message(STATUS "BOOST_SYSTEM_LIB: ${BOOST_SYSTEM_LIB}")
    
    find_library(BOOST_FILESYSTEM_LIB 
        NAMES boost_filesystem libboost_filesystem boost_filesystem-mt libboost_filesystem-mt
        PATHS
        /opt/homebrew/lib
        /usr/local/lib
        /usr/lib
    )
    message(STATUS "BOOST_FILESYSTEM_LIB: ${BOOST_FILESYSTEM_LIB}")
    
    if(BOOST_INCLUDE_DIR AND BOOST_FILESYSTEM_LIB)
        message(STATUS "✅ Found Boost manually:")
        message(STATUS "  Headers: ${BOOST_INCLUDE_DIR}")
        message(STATUS "  Filesystem library: ${BOOST_FILESYSTEM_LIB}")
        
        # Create Boost filesystem target
        add_library(Boost::filesystem UNKNOWN IMPORTED)
        set_target_properties(Boost::filesystem PROPERTIES
            IMPORTED_LOCATION ${BOOST_FILESYSTEM_LIB}
            INTERFACE_INCLUDE_DIRECTORIES ${BOOST_INCLUDE_DIR}
        )
        
        # Create Boost system target if found, otherwise create a dummy one
        if(BOOST_SYSTEM_LIB)
            message(STATUS "  System library: ${BOOST_SYSTEM_LIB}")
            add_library(Boost::system UNKNOWN IMPORTED)
            set_target_properties(Boost::system PROPERTIES
                IMPORTED_LOCATION ${BOOST_SYSTEM_LIB}
                INTERFACE_INCLUDE_DIRECTORIES ${BOOST_INCLUDE_DIR}
            )
        else()
            message(STATUS "  System library: NOT FOUND - creating dummy target")
            # Create a dummy system target (some Boost installations don't include system)
            add_library(Boost::system INTERFACE)
            target_include_directories(Boost::system INTERFACE ${BOOST_INCLUDE_DIR})
        endif()
    else()
        message(STATUS "❌ Could not find Boost manually, falling back to find_package")
        # Fall back to normal find_package for AppleClang 16.0.0 or when Boost isn't installed
        find_package(Boost REQUIRED COMPONENTS system filesystem)
    endif()
else()
    # Use normal find_package for non-Apple systems
    find_package(Boost REQUIRED COMPONENTS system filesystem)
endif()

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
    Boost::filesystem
    nlohmann_json::nlohmann_json
)

# Only link to Boost::system if we found the actual library
if(APPLE AND BOOST_SYSTEM_LIB)
    target_link_libraries(adjfind PRIVATE Boost::system)
endif()

# Include directories for GDAL
target_include_directories(adjfind PRIVATE ${GDAL_INCLUDE_DIRS})

# Compiler flags
target_compile_definitions(adjfind PRIVATE ${GDAL_DEFINITIONS})

# Enable packaging
include(CPack)
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "AdjFind - Transportation Network Algorithms")
set(CPACK_PACKAGE_VENDOR "Zifan Wang")
set(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/README.md")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")

# Installation configuration
install(TARGETS adjfind DESTINATION bin)
install(FILES README.md LICENSE DESTINATION share/adjfind)
install(DIRECTORY assets DESTINATION share/adjfind)