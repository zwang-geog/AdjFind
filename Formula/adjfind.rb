class Adjfind < Formula
  desc "C++ program with specialized path finding algorithms for adjacency/proximity"
  homepage "https://github.com/zwang-geog/AdjFind"
  url "https://github.com/zwang-geog/AdjFind/archive/refs/tags/v0.1.1.tar.gz"
  sha256 "061413bd1bcce01615b5a440cf09d8bf88c7b35fd29e581ec62b271a541a6827"
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
