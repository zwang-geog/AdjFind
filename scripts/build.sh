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

# Configure Boost detection for AppleClang 17.0.0
CMAKE_ARGS=""
if [[ "$OSTYPE" == "darwin"* ]]; then
    echo "Detected macOS build environment"
    CLANG_VERSION=$(clang++ --version | grep -o '[0-9]\+\.[0-9]\+\.[0-9]\+' | head -1)
    echo "Detected AppleClang version: $CLANG_VERSION"
    
    if [[ "$CLANG_VERSION" == "17.0.0" ]]; then
        echo "Applying AppleClang 17.0.0 specific Boost configuration"
        # AppleClang 17.0.0 needs explicit Boost configuration
        CMAKE_ARGS="-DBoost_NO_BOOST_CMAKE=ON -DBoost_NO_SYSTEM_PATHS=ON"
        
        # Add Homebrew paths
        if command -v brew &> /dev/null; then
            HOMEBREW_PREFIX=$(brew --prefix)
            CMAKE_ARGS="$CMAKE_ARGS -DCMAKE_PREFIX_PATH=$HOMEBREW_PREFIX"
            CMAKE_ARGS="$CMAKE_ARGS -DBOOST_ROOT=$HOMEBREW_PREFIX"
            CMAKE_ARGS="$CMAKE_ARGS -DBoost_INCLUDE_DIR=$HOMEBREW_PREFIX/include"
            CMAKE_ARGS="$CMAKE_ARGS -DBoost_LIBRARY_DIR=$HOMEBREW_PREFIX/lib"
        fi
    fi
fi

# Configure with CMake
cmake .. \
    -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
    -DCMAKE_INSTALL_PREFIX="$INSTALL_PREFIX" \
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    $CMAKE_ARGS 

# Build
cmake --build . --config "$BUILD_TYPE" -j$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 1)

# Create package
cpack -G TGZ

echo "Build completed successfully!"
