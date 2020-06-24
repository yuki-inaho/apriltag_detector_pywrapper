from Cython.Build import cythonize
from Cython.Distutils import build_ext
from setuptools import setup, Extension
import numpy
import sys
import os
import glob
import pkgconfig


opencv_cflags = pkgconfig.cflags('opencv')
cvlibs_string = pkgconfig.libs('opencv')

lib_dirs = []
cvlibs = list()
cvlibs_pkgcfg_list = cvlibs_string.split()
for elem in cvlibs_pkgcfg_list:
    # like u'-L/usr/local/lib'
    if elem.startswith("-L"):
        lib_dirs.append(str(elem))
    # like u'-lopencv_stitching'
    elif elem.startswith("-l"):
        _cvlib = 'opencv_{}'.format(elem.split('-lopencv_')[-1])
        cvlibs.append(_cvlib)
    else:
        pass


sources = ["april_detector_pywrapper.pyx",
           "src/example/april_detector_manager.cpp"] + glob.glob("src/*.cc")

setup(
    name="april_detector_pywrapper",
    version='1.0.0',
    description='Python wrapper of april detector(based on ethz_apriltag2)',
    author='yuki-inaho',
    ext_modules=cythonize(
        [
            Extension("april_detector_pywrapper",
                      sources=sources,
                      include_dirs=[numpy.get_include(),
                                    os.path.join(
                                        sys.prefix, 'include', 'opencv2'),
                                    "/usr/include/eigen3",
                                    os.path.join(
                          os.getcwd(), 'include'),
                      ],
                      library_dirs=lib_dirs,
                      libraries=cvlibs,
                      extra_compile_args=["-std=gnu++11", "-O3", "-fopenmp"],
                      extra_link_args=["-fopenmp"],
                      language="c++",
                      ),

            Extension("opencv_mat",
                      sources=["opencv_mat.pyx"],
                      include_dirs=[numpy.get_include(),
                                    os.path.join(
                          sys.prefix, 'include', 'opencv2'),
                      ],
                      extra_link_args=["-std=gnu++11", "-O3"],
                      library_dirs=lib_dirs,
                      libraries=cvlibs,
                      language="c++"
                      )
        ],
    ),
    cmdclass={'build_ext': build_ext},
)
