from conan import ConanFile
from conan.tools.cmake import CMake, CMakeDeps, CMakeToolchain, cmake_layout


class LaserScannerToolkitConan(ConanFile):
    name = "laser-scanner-toolkit"
    version = "1.0.0"
    license = "MIT"
    author = "Laser Scanner Toolkit contributors"
    description = "3D line-laser profiler calibration, simulation, reconstruction, and measurement toolkit"
    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeDeps", "CMakeToolchain"

    def requirements(self):
        self.requires("eigen/3.4.0")
        self.requires("opencv/4.8.1")

    def layout(self):
        cmake_layout(self)

    def generate(self):
        CMakeDeps(self).generate()
        CMakeToolchain(self).generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()
