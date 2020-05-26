from Cython.Build import cythonize
from Cython.Distutils import build_ext
from setuptools import setup, find_packages, Extension
import numpy
import sys
import os
import glob
import pkgconfig

lib_folder = os.path.join(sys.prefix, 'local', 'lib')
cvlibs = list()

#for file in glob.glob(os.path.join(lib_folder, 'libopencv_*')):
#    cvlibs.append(file.split('.')[0])
#cvlibs = list(set(cvlibs))
#cvlibs = ['opencv_{}'.format(lib.split(os.path.sep)[-1].split('libopencv_')[-1]) for lib in cvlibs]
cvlibs = pkgconfig.libs('opencv')
lib_dirs = [lib_folder]

sources = ["april_detector_pywrapper.pyx", "src/example/april_detector_manager.cpp"] + glob.glob("src/*.cc")

setup(
    name = "april_detector_pywrapper",
    version='1.0.0',
    description='Python wrapper of april detector(based on ethz_apriltag2)',
    author='yuki-inaho',
    ext_modules = cythonize(
                 [
                    Extension("april_detector_pywrapper",
                        sources=sources,
                        include_dirs=[numpy.get_include(),
                                        os.path.join(sys.prefix, 'include', 'opencv2'),
                                        "/usr/include/eigen3",
                                        os.path.join(os.getcwd(), 'include'),
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
                                        os.path.join(sys.prefix, 'include', 'opencv2'),
                                        ],
                        extra_link_args=["-std=gnu++11", "-O3"],
                        library_dirs=lib_dirs,
                        libraries=cvlibs,
                        language="c++"                    
                    )                    
                 ],
    ),
    cmdclass = {'build_ext': build_ext},
)
