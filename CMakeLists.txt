# On Ubuntu 18.04, get the latest CMake from https://apt.kitware.com/.
cmake_minimum_required(VERSION 3.18)

project(Open3DCMakeFindPackage LANGUAGES C CXX)

# Find installed Open3D, which exports Open3D::Open3D
set(Open3D_ROOT "/z/liuxs/open3d_install")
find_package(Open3D REQUIRED)

find_package(Armadillo REQUIRED)

add_executable(Draw)
target_sources(Draw PRIVATE Draw.cpp)
target_link_libraries(Draw PRIVATE Open3D::Open3D ${ARMADILLO_LIBRARIES})

