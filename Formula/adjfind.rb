class Adjfind < Formula
  desc "A C++ program containing specialized path finding algorithms related to adjacency/proximity"
  homepage "https://github.com/zwang-geog/AdjFind"
  url "https://github.com/zwang-geog/AdjFind.git", tag: "v0.1.0"
  version "0.1.0"
  license "MIT"

  depends_on "cmake" => :build
  depends_on "gdal"
  depends_on "boost"

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
    system "#{bin}/adjfind", "--help"
  end
end