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

# Configure Boost paths for macOS
CMAKE_PREFIX_ARGS=""
echo "Detected OS: $OSTYPE"
if [[ "$OSTYPE" == "darwin"* ]]; then
    echo "macOS detected, configuring Boost paths..."
    
    # On macOS, add Homebrew paths for other dependencies
    if command -v brew &> /dev/null; then
        echo "Homebrew found, getting prefix..."
        HOMEBREW_PREFIX=$(brew --prefix)
        echo "Homebrew prefix: $HOMEBREW_PREFIX"
        CMAKE_PREFIX_ARGS="-DCMAKE_PREFIX_PATH=$HOMEBREW_PREFIX"
    else
        echo "Homebrew not found"
    fi
    
    # Use downloaded Boost instead of Homebrew Boost
    echo "Checking for downloaded Boost directory..."
    # Look for boost in the parent directory (workspace root)
    BOOST_PATH="../boost"
    if [[ -d "$BOOST_PATH" ]]; then
        echo "Found boost directory, setting Boost_ROOT to: $PWD/$BOOST_PATH"
        CMAKE_PREFIX_ARGS="$CMAKE_PREFIX_ARGS -DBoost_ROOT=$PWD/$BOOST_PATH -DBoost_NO_BOOST_CMAKE=ON"
    else
        echo "Boost directory not found at: $PWD/$BOOST_PATH"
        echo "Current directory contents:"
        ls -la ../
    fi
    
    echo "Final CMAKE_PREFIX_ARGS: $CMAKE_PREFIX_ARGS"
else
    echo "Not macOS, skipping Boost configuration"
fi

# Configure with CMake
cmake .. \
    -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
    -DCMAKE_INSTALL_PREFIX="$INSTALL_PREFIX" \
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    $CMAKE_PREFIX_ARGS 

# Build
cmake --build . --config "$BUILD_TYPE" -j$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 1)

# Create package
cpack -G TGZ

echo "Build completed successfully!"
