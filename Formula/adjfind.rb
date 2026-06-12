class Adjfind < Formula
  desc "C++ program with specialized path finding algorithms for adjacency/proximity"
  homepage "https://github.com/zwang-geog/AdjFind"
  url "https://github.com/zwang-geog/AdjFind/archive/refs/tags/v0.2.2.tar.gz"
  sha256 "e5867d5c90080b64bbf8afd5351442f987df0047ade300604d713774ccb843cc"
  license "MIT"

  depends_on "cmake" => :build
  depends_on "boost"
  depends_on "gdal"
  depends_on "hdf5"

  def install
    # Create build directory
    mkdir "build" do
      # Configure with CMake
      system "cmake", "..",
             "-DCMAKE_BUILD_TYPE=Release",
             "-DCMAKE_INSTALL_PREFIX=#{prefix}",
             *std_cmake_args

      # Build
      system "cmake", "--build", ".", "--config", "Release"

      # Install
      system "cmake", "--install", "."
    end
  end

  test do
    # Test that the executable runs and shows help
    system bin/"adjfind", "--help"
  end
end
