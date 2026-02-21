import os

from conan.tools.cmake import CMake, CMakeDeps, CMakeToolchain, cmake_layout
from conan.tools.files import collect_libs, copy, replace_in_file

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
        # Pangolin's pango_image links with ${JPEG_LIBRARY}, which is not
        # reliably defined by Conan's FindJPEG module. Prefer imported target.
        replace_in_file(
            self,
            os.path.join(self.source_folder, "components/pango_image/CMakeLists.txt"),
            "        target_link_libraries(${COMPONENT} PRIVATE ${JPEG_LIBRARY})",
            "        if(TARGET JPEG::JPEG)\n"
            "            target_link_libraries(${COMPONENT} PRIVATE JPEG::JPEG)\n"
            "        else()\n"
            "            target_link_libraries(${COMPONENT} PRIVATE ${JPEG_LIBRARIES})\n"
            "        endif()",
            strict=True,
        )
        # Conan's libjpeg package may not provide jpeg_skip_scanlines while
        # system headers still define LIBJPEG_TURBO_VERSION. Use the generic
        # scanline loop path to avoid unresolved jpeg_skip_scanlines at link.
        replace_in_file(
            self,
            os.path.join(self.source_folder, "components/pango_image/src/image_io_jpg.cpp"),
            "#ifdef LIBJPEG_TURBO_VERSION\n"
            "                // bug in libjpeg-turbo prevents us from skipping to end, so skip to end-1\n"
            "                jpeg_skip_scanlines(&cinfo, cinfo.output_height-1);\n"
            "                jpeg_read_scanlines(&cinfo, imageBuffer, 1);\n"
            "#else\n"
            "                for (size_t y = 0; y < cinfo.output_height; y++) {\n"
            "                    jpeg_read_scanlines(&cinfo, imageBuffer, 1);\n"
            "                }\n"
            "#endif",
            "                for (size_t y = 0; y < cinfo.output_height; y++) {\n"
            "                    jpeg_read_scanlines(&cinfo, imageBuffer, 1);\n"
            "                }",
            strict=True,
        )

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
