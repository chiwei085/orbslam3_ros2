import os

from conan.tools.cmake import CMake, CMakeDeps, CMakeToolchain, cmake_layout
from conan.tools.files import collect_libs, copy

from conan import ConanFile


class PangolinConan(ConanFile):
    name = "pangolin"
    version = "0.8"
    package_type = "library"

    license = "MIT"
    url = "https://github.com/stevenlovegrove/Pangolin"
    description = "Lightweight portable rapid development library for managing OpenGL display / interaction and abstracting video input."

    settings = "os", "arch", "compiler", "build_type"
    options = {"shared": [True, False], "fPIC": [True, False]}
    default_options = {"shared": True, "fPIC": True}

    def requirements(self):
        self.requires("eigen/3.4.0")
        self.requires("opencv/4.8.1")

    def export_sources(self):
        src = os.path.normpath(os.path.join(self.recipe_folder, "../../../third_party/Pangolin"))
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
        tc.variables["BUILD_TOOLS"] = False
        tc.variables["BUILD_EXAMPLES"] = False
        tc.variables["BUILD_TESTS"] = False
        tc.variables["BUILD_PANGOLIN_PYTHON"] = False
        tc.variables["BUILD_PANGOLIN_LIBOPENEXR"] = False
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
        self.cpp_info.set_property("cmake_file_name", "Pangolin")
        self.cpp_info.set_property("cmake_target_name", "Pangolin::pangolin")
        self.cpp_info.libs = collect_libs(self)
