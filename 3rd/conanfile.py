from conan import ConanFile


class ExampleRecipe(ConanFile):
    settings = "os", "compiler", "build_type", "arch"

    # Keep legacy generators to support older CMake (e.g. ROS Noetic default CMake)
    # and existing catkin packages that include 3rd/conanbuildinfo.cmake.
    generators = "cmake", "cmake_find_package"

    def configure(self):
        self.options["osqp"].shared = True
        self.options["libunwind"].shared = True
        self.options["ceres-solver"].shared = True
        self.options["ceres-solver"].use_glog = True
        self.options["glog"].shared = True
        self.options["glog"].with_threads = True
        self.options["gflags"].shared = True

    def requirements(self):
        self.requires("osqp/0.6.3")
        self.requires("ceres-solver/1.14.0")