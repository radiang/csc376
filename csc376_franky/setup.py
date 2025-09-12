from setuptools import setup, Extension
import pybind11
import numpy

ext_modules = [
    Extension(
        'csc376_franky',
        [
            'python/bind_franky.cpp',
            'src/franka_joint_trajectory_controller.cpp',
            'src/gripper.cpp',
        ],
        include_dirs=[
            'include/',
            pybind11.get_include(),
            numpy.get_include(),
            '/usr/include/eigen3',
            '/usr/local/include/eigen3',
        ],
        libraries=['franka'],
        library_dirs=['/usr/lib', '/usr/local/lib'],
        language='c++',
        extra_compile_args=['-std=c++17', '-O3'],
    ),
]

setup(
    name='csc376_franky',
    ext_modules=ext_modules,
    zip_safe=False,
)
