cmake_minimum_required(VERSION 3.5)
project(csc376_franky)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# Find required packages
find_package(Franka 0.9.2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(pybind11 REQUIRED)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
)

# Create the main library (your C++ trajectory controller)
add_library(csc376_franky_lib SHARED
    src/franka_joint_trajectory_controller.cpp
    src/gripper.cpp
)

target_link_libraries(csc376_franky_lib
    Franka::Franka
    Eigen3::Eigen
    Threads::Threads
)

target_include_directories(csc376_franky_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Create the Python module
pybind11_add_module(csc376_franky_py 
    python/bind_franky.cpp
)

# Link the Python module to your C++ library
target_link_libraries(csc376_franky_py PRIVATE
    csc376_franky_lib
    Franka::Franka
    Eigen3::Eigen
    Threads::Threads
)

# Compiler-specific options for the Python module
target_compile_definitions(csc376_franky_py PRIVATE VERSION_INFO=${EXAMPLE_VERSION_INFO})

# Optional: Set properties for the Python module
set_target_properties(csc376_franky_py PROPERTIES
    CXX_VISIBILITY_PRESET "hidden"
    VISIBILITY_INLINES_HIDDEN ON
)

# Optional: Install targets
install(TARGETS csc376_franky_lib
    EXPORT csc376_franky_libTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(DIRECTORY include/
    DESTINATION include
)

# Install Python module (optional - usually done via pip install)
# install(TARGETS csc376_franky_py
#     COMPONENT python
#     LIBRARY DESTINATION ${PYTHON_LIBRARY_DIR}
# )
