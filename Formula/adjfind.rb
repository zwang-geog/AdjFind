class Adjfind < Formula
  desc "C++ program with specialized path finding algorithms for adjacency/proximity"
  homepage "https://github.com/zwang-geog/AdjFind"
  url "https://github.com/zwang-geog/AdjFind/archive/refs/tags/v0.2.0.tar.gz"
  sha256 "861fb40762edaafd9a014be4c306c3e47f7c7a2d6d90523b8d4cdc83c67e9ce1"
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
