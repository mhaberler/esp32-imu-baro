# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

FILE(GLOB_RECURSE app_sources
    ${CMAKE_SOURCE_DIR}/src/app/*.*
    ${CMAKE_SOURCE_DIR}/src/custom-example/*.*
    ${CMAKE_SOURCE_DIR}/src/teleplot/Teleplot.h
)

idf_component_register(SRCS ${app_sources})
