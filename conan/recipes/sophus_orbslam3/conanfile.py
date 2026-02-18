import os

from conan.tools.cmake import CMake, CMakeDeps, CMakeToolchain, cmake_layout
from conan.tools.files import copy

from conan import ConanFile


class SophusOrbslam3Conan(ConanFile):
    name = "sophus_orbslam3"
    version = "1.0"
    package_type = "header-library"

    license = "MIT"
    url = "https://github.com/strasdat/Sophus"
    description = "Sophus fork used by ORB_SLAM3"

    settings = "os", "arch", "compiler", "build_type"
    no_copy_source = True

    def requirements(self):
        self.requires("eigen/3.4.0")

    def export_sources(self):
        src = os.path.normpath(
            os.path.join(self.recipe_folder, "../../../third_party/ORB_SLAM3/Thirdparty/Sophus")
        )
        dst = os.path.join(self.export_sources_folder, "source")
        copy(self, "*", src=src, dst=dst)

    def layout(self):
        cmake_layout(self)

    def generate(self):
        deps = CMakeDeps(self)
        deps.generate()

        tc = CMakeToolchain(self)
        tc.variables["BUILD_TESTS"] = False
        tc.variables["BUILD_EXAMPLES"] = False
        tc.generate()

    def source(self):
        src = os.path.join(self.export_sources_folder, "source")
        copy(self, "*", src=src, dst=self.source_folder)

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()

    def package_info(self):
        self.cpp_info.set_property("cmake_file_name", "Sophus")
        self.cpp_info.set_property("cmake_target_name", "Sophus::Sophus")
