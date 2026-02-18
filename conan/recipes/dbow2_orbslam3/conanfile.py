import os

from conan.tools.cmake import CMake, CMakeDeps, CMakeToolchain, cmake_layout
from conan.tools.files import collect_libs, copy

from conan import ConanFile


class DBow2Orbslam3Conan(ConanFile):
    name = "dbow2_orbslam3"
    version = "1.0"
    package_type = "library"

    license = "BSD-3-Clause"
    url = "https://github.com/UZ-SLAMLab/ORB_SLAM3"
    description = "DBoW2 fork used by ORB_SLAM3"

    settings = "os", "arch", "compiler", "build_type"
    options = {"shared": [True, False], "fPIC": [True, False]}
    default_options = {
        "shared": True,
        "fPIC": True,
        "opencv/*:with_ffmpeg": False,
        "opencv/*:with_wayland": False,
        "opencv/*:with_gtk": False,
    }

    def requirements(self):
        self.requires("opencv/4.8.1")

    def export_sources(self):
        src = os.path.normpath(
            os.path.join(self.recipe_folder, "../../../third_party/ORB_SLAM3/Thirdparty/DBoW2")
        )
        dst = os.path.join(self.export_sources_folder, "source")
        copy(self, "*", src=src, dst=dst)

    def config_options(self):
        if self.settings.os == "Windows":
            self.options.rm_safe("fPIC")

    def layout(self):
        cmake_layout(self)

    def generate(self):
        deps = CMakeDeps(self)
        deps.generate()

        tc = CMakeToolchain(self)
        tc.variables["LIBRARY_OUTPUT_PATH"] = os.path.join(self.source_folder, "lib")
        tc.generate()

    def source(self):
        src = os.path.join(self.export_sources_folder, "source")
        copy(self, "*", src=src, dst=self.source_folder)

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        copy(
            self,
            pattern="*.h",
            src=self.source_folder,
            dst=os.path.join(self.package_folder, "include"),
        )
        copy(
            self,
            pattern="*.hpp",
            src=self.source_folder,
            dst=os.path.join(self.package_folder, "include"),
        )
        copy(
            self,
            pattern="*.a",
            src=os.path.join(self.source_folder, "lib"),
            dst=os.path.join(self.package_folder, "lib"),
            keep_path=False,
        )
        copy(
            self,
            pattern="*.so*",
            src=os.path.join(self.source_folder, "lib"),
            dst=os.path.join(self.package_folder, "lib"),
            keep_path=False,
        )
        copy(
            self,
            pattern="*.dylib*",
            src=os.path.join(self.source_folder, "lib"),
            dst=os.path.join(self.package_folder, "lib"),
            keep_path=False,
        )

    def package_info(self):
        self.cpp_info.set_property("cmake_file_name", "DBoW2")
        self.cpp_info.set_property("cmake_target_name", "DBoW2::DBoW2")
        self.cpp_info.libs = collect_libs(self)
