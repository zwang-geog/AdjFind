#!/bin/bash

# Setup script for installing conda and Boost on Debian/Ubuntu VM
# This script installs miniconda and uses it to get a newer Boost version

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_status "Setting up conda for Boost installation on Debian/Ubuntu..."

# Check if conda is already installed
if command -v conda &> /dev/null; then
    print_warning "Conda is already installed. Skipping conda installation."
else
    print_status "Installing miniforge (conda-forge focused)..."
    
    # Download miniforge for Linux (same as macOS approach)
    cd ~
    wget -O miniforge.sh https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh
    
    # Install miniforge
    bash miniforge.sh -b -p $HOME/miniforge3
    
    # Add conda to PATH
    echo 'export PATH="$HOME/miniforge3/bin:$PATH"' >> ~/.bashrc
    source ~/.bashrc
    
    # Clean up
    rm miniforge.sh
    
    print_status "Miniforge installed successfully!"
fi

# Initialize conda
print_status "Initializing conda..."
export PATH="$HOME/miniforge3/bin:$PATH"
conda init bash
source ~/.bashrc

# Install Boost and GDAL via conda (no ToS needed with miniforge)
print_status "Installing Boost and GDAL via conda..."
conda install -c conda-forge boost-cpp gdal -y

# Verify installation
print_status "Verifying Boost and GDAL installation..."
conda list boost-cpp
conda list gdal

print_status "Setting up environment variables for CMake..."
echo 'export CMAKE_PREFIX_PATH="$HOME/miniforge3"' >> ~/.bashrc
echo 'export CONDA_PREFIX="$HOME/miniforge3"' >> ~/.bashrc

print_status "Conda setup complete!"
echo ""
echo "To use conda Boost in your build:"
echo "1. Source your bashrc: source ~/.bashrc"
echo "2. Run your build script: ./scripts/build-deb.sh"
echo ""
echo "The conda Boost will be automatically found by CMake."
