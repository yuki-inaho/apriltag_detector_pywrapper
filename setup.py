from Cython.Build import cythonize
from Cython.Distutils import build_ext
from setuptools import setup, Extension
import numpy
import os
import glob

zense_install_dir = os.environ["PICOZENSE_INSTALL_DIR"]
cvlib_folder = os.path.join(
    zense_install_dir,
    'Thirdparty', 'opencv-3.4.1', 'lib'
)

cvlib_include_folder = os.path.join(
    zense_install_dir,
    'Thirdparty', 'opencv-3.4.1', 'include'
)

lib_dirs = [cvlib_folder]

cvlibs = list()
for file in glob.glob(os.path.join(cvlib_folder, 'libopencv_*.so')):
    cvlibs.append(os.path.basename(file).split('.')[0])
cvlibs = list(set(cvlibs))
cvlibs = ['opencv_{}'.format(
    lib.split(os.path.sep)[-1].split('libopencv_')[-1]) for lib in cvlibs]


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
                                    "/usr/include/eigen3",
                                    os.path.join(os.getcwd(), 'include'),
                                    cvlib_include_folder
                      ],
                      library_dirs=lib_dirs,
                      libraries=cvlibs,
                      extra_compile_args=["-std=gnu++11", "-O3", "-fopenmp"],
                      extra_link_args=["-fopenmp"],
                      language="c++",
                      ),

            Extension("opencv_mat",
                      sources=["opencv_mat.pyx"],
                      include_dirs=[
                          numpy.get_include(),
                      ] + cvinclude,
                      extra_link_args=["-std=gnu++11", "-O3"],
                      library_dirs=lib_dirs,
                      libraries=cvlibs,
                      language="c++"
                      )
        ],
    ),
    cmdclass={'build_ext': build_ext},
)