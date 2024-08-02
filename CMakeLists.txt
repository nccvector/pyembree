cmake_minimum_required(VERSION 3.5)

project(pyembree)

find_package(Python COMPONENTS Interpreter Development REQUIRED)
set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 CONFIG REQUIRED)
add_subdirectory(embree)

set(CMAKE_CXX_STANDARD 17)

pybind11_add_module(pyembree main.cpp)
target_link_libraries(pyembree PRIVATE embree)