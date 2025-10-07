class Adjfind < Formula
  desc "C++ program with specialized path finding algorithms for adjacency/proximity"
  homepage "https://github.com/zwang-geog/AdjFind"
  url "https://github.com/zwang-geog/AdjFind/archive/refs/tags/v0.1.2.tar.gz"
  sha256 "dad985fbb2b752e993bc1631af50eee90f409a1a83351894882652bcebbd5ca2"
  license "MIT"

  depends_on "cmake" => :build
  depends_on "boost"
  depends_on "gdal"

  def install
    # Create build directory
    mkdir "build" do
      # Configure with CMake
      system "cmake", "..",
             "-DCMAKE_BUILD_TYPE=Release",
             "-DCMAKE_INSTALL_PREFIX=#{prefix}",
             "-DCMAKE_POSITION_INDEPENDENT_CODE=ON",
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
