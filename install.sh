#!/bin/bash

# AdjFind Installation Script
# This script helps install dependencies and build the project

set -e

echo "🚀 AdjFind Installation Script"
echo "================================"

# Detect OS
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    OS="linux"
    PKG_MANAGER="apt"
elif [[ "$OSTYPE" == "darwin"* ]]; then
    OS="macos"
    PKG_MANAGER="brew"
else
    echo "❌ Unsupported operating system: $OSTYPE"
    exit 1
fi

echo "📋 Detected OS: $OS"

# Check if CMake is installed
if ! command -v cmake &> /dev/null; then
    echo "❌ CMake not found. Installing..."
    if [[ "$OS" == "linux" ]]; then
        sudo apt update
        sudo apt install -y cmake build-essential
    elif [[ "$OS" == "macos" ]]; then
        brew install cmake
    fi
else
    echo "✅ CMake found: $(cmake --version | head -n1)"
fi

# Check if C++ compiler is installed
if ! command -v g++ &> /dev/null && ! command -v clang++ &> /dev/null; then
    echo "❌ C++ compiler not found. Installing..."
    if [[ "$OS" == "linux" ]]; then
        sudo apt update
        sudo apt install -y g++ clang
    elif [[ "$OS" == "macos" ]]; then
        echo "⚠️  Please install Xcode Command Line Tools: xcode-select --install"
    fi
else
    echo "✅ C++ compiler found"
fi

# Install GDAL
if ! pkg-config --exists gdal; then
    echo "❌ GDAL not found. Installing..."
    if [[ "$OS" == "linux" ]]; then
        sudo apt update
        sudo apt install -y libgdal-dev
    elif [[ "$OS" == "macos" ]]; then
        brew install gdal
    fi
else
    echo "✅ GDAL found: $(pkg-config --modversion gdal)"
fi

# Install Boost
if ! pkg-config --exists boost; then
    echo "❌ Boost not found. Installing..."
    if [[ "$OS" == "linux" ]]; then
        sudo apt update
        sudo apt install -y libboost-all-dev
    elif [[ "$OS" == "macos" ]]; then
        brew install boost
    fi
else
    echo "✅ Boost found"
fi

# Install nlohmann_json
if ! pkg-config --exists nlohmann_json; then
    echo "❌ nlohmann_json not found. Installing..."
    if [[ "$OS" == "linux" ]]; then
        sudo apt update
        sudo apt install -y nlohmann-json3-dev
    elif [[ "$OS" == "macos" ]]; then
        brew install nlohmann-json
    fi
else
    echo "✅ nlohmann_json found"
fi

echo ""
echo "🔨 Building AdjFind..."
echo "======================"

# Create build directory
mkdir -p build
cd build

# Configure with CMake
echo "📐 Configuring with CMake..."
cmake .. -DCMAKE_BUILD_TYPE=Release

# Build the project
echo "🏗️  Building project..."
cmake --build . --config Release --parallel

echo ""
echo "✅ Build completed successfully!"
echo "🎯 Executable location: $(pwd)/adjfind"
echo ""
echo "📖 Usage:"
echo "   ./adjfind --help"
echo ""
echo "🚀 Happy coding!"
