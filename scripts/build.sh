#!/bin/bash

# Exit on error
set -e

# Default values
BUILD_TYPE="Release"
INSTALL_PREFIX="/usr/local"
BUILD_DIR="build"

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    key="$1"
    case $key in
        --build-type)
        BUILD_TYPE="$2"
        shift
        shift
        ;;
        --prefix)
        INSTALL_PREFIX="$2"
        shift
        shift
        ;;
        --build-dir)
        BUILD_DIR="$2"
        shift
        shift
        ;;
        *)
        echo "Unknown option: $1"
        exit 1
        ;;
    esac
done

# Create build directory if it doesn't exist
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# Configure with CMake
cmake .. \
    -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
    -DCMAKE_INSTALL_PREFIX="$INSTALL_PREFIX" \
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON

# Build
cmake --build . --config "$BUILD_TYPE" -j$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 1)

# Create package
cpack -G TGZ

echo "Build completed successfully!"
