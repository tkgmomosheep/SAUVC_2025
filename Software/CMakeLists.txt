# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(sauvc-idf-only)

idf_build_set_property(CXX_COMPILE_OPTIONS "-Wmissing-field-initializers" "-O3" APPEND)