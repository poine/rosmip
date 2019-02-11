from distutils.core import setup
from distutils.extension import Extension
from Cython.Build import cythonize
from Cython.Distutils import build_ext

# https://github.com/longjie/ros_cython_example

##
# workaround this bug:
# http://stackoverflow.com/questions/8106258/cc1plus-warning-command-line-option-wstrict-prototypes-is-valid-for-ada-c-o
import distutils, Cython
class my_build_ext(Cython.Distutils.build_ext):
    def build_extensions(self):
        distutils.sysconfig.customize_compiler(self.compiler)
        try:
            self.compiler.compiler_so.remove("-Wstrict-prototypes")
        except (AttributeError, ValueError):
            pass
        build_ext.build_extensions(self)
##
# extra_compile_args=["-std=c++11", "-Wno-cpp"] is for removing the deprecation warning
#  as defining NPY_NO_DEPRECATED_API to NPY_1_7_API_VERSION breaks compilation



ext = Extension(
    "cpp_rosmip_control",        # name of extension
    ["src/rosmip_control/rosmip_control.pyx"],  # filename of our Pyrex/Cython source
    language="c++",    # this causes Pyrex/Cython to create C++ source
    include_dirs=["/home/poine/work/rosmip.git/rosmip/rosmip_control/include", "/opt/ros/kinetic/include/"],
    extra_compile_args=["-std=c++11", "-Wno-cpp"],
    libraries=["rosmip_controller_legacy"],
    library_dirs=["/opt/ros/kinetic/lib/", "/home/poine/work/overlay_ws/devel/lib/"],
    runtime_library_dirs=[],
)

setup(
    name = 'cpp_rosmip_control',
    ext_modules = [ext],
    cmdclass = {'build_ext': my_build_ext}
)
