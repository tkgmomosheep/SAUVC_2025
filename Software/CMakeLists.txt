# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(sauvc-idf-only)

idf_build_set_property(COMPILE_OPTIONS "-Wno-missing-field-initializers" "-O3" APPEND
                       CXX_COMPILE_OPTIONS "-Wno-missing-field-initializers" APPEND
                      )