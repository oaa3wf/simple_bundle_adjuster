cmake_minimum_required(VERSION 3.5)

project(simple_ba)

set(Eigen3_DIR "C:/Users/dapot/code/simple_bundle_adjuster/third_party/eigen/install_dir/lib/cmake/eigen3/")
message(Eigen3_DIR="${Eigen3_DIR}")
find_package(Eigen3 3.3 REQUIRED)
message(EIGEN_VERSION="${EIGEN3_VERSION_STRING}")
find_package(Ceres REQUIRED)
find_package(pybind11 CONFIG REQUIRED)
find_package(nlohmann_json REQUIRED)
add_compile_definitions(GLOG_NO_ABBREVIATED_SEVERITIES)

set(HEADERS_LIST
        include/simpleBA.h
        include/cost_functors.h
        include/simpleBASerialization.h)

# helloworld
# add_subdirectory(../third_party/pybind11)
message(cmake_source_DIR="${CMAKE_SOURCE_DIR}")
add_executable(simple_ba src/testSimpleBA.cpp src/simpleBA.cpp ${HEADERS_LIST})
target_include_directories(simple_ba PRIVATE "${CMAKE_SOURCE_DIR}/include/")
# source_group("Headers" FILES "${CMAKE_SOURCE_DIR}/include/")
# pybind11_add_module(simple_ba python/pybind.cc)
target_link_libraries(simple_ba Ceres::ceres nlohmann_json::nlohmann_json)