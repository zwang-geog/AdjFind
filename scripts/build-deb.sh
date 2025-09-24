#!/bin/bash

# Build script for creating Debian packages for AdjFind
# This script automates the process of building .deb packages

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if we're in the right directory
if [ ! -f "CMakeLists.txt" ] || [ ! -d "debian" ]; then
    print_error "Please run this script from the root directory of the AdjFind project"
    exit 1
fi

# Check for required tools
print_status "Checking for required tools..."

for tool in dpkg-buildpackage debhelper cmake make; do
    if ! command -v $tool &> /dev/null; then
        print_error "$tool is not installed. Please install it first:"
        echo "sudo apt update && sudo apt install dpkg-dev debhelper cmake build-essential"
        exit 1
    fi
done

print_status "All required tools are available"

# Clean previous builds
print_status "Cleaning previous builds..."
rm -rf build/ *.deb *.dsc *.tar.gz *.changes *.buildinfo

# Build the package
print_status "Building Debian package..."
dpkg-buildpackage -us -uc

# Check if build was successful
if [ $? -eq 0 ]; then
    print_status "Package built successfully!"
    echo ""
    echo "Generated files:"
    ls -la *.deb *.dsc *.tar.gz *.changes *.buildinfo 2>/dev/null || true
    echo ""
    echo "To install the package locally:"
    echo "sudo dpkg -i adjfind_*.deb"
    echo ""
    echo "To fix any dependency issues:"
    echo "sudo apt-get install -f"
    echo ""
    echo "To test the package:"
    echo "adjfind --help"
else
    print_error "Package build failed!"
    exit 1
fi
