from conan import ConanFile


class Orbslam3Ros2Deps(ConanFile):
    name = "orbslam3_ros2_deps"
    version = "0.1"
    package_type = "application"

    settings = "os", "arch", "compiler", "build_type"

    generators = ("CMakeDeps", "CMakeToolchain")

    def requirements(self):
        self.requires("eigen/3.4.0")
        self.requires("opencv/4.8.1")
        self.requires("boost/1.84.0")

        self.requires("pangolin/0.8@local/stable")
        self.requires("sophus_orbslam3/1.0@local/stable")
        self.requires("g2o_orbslam3/1.0@local/stable")
        self.requires("dbow2_orbslam3/1.0@local/stable")

    def configure(self):
        self.settings.compiler.cppstd = "17"
