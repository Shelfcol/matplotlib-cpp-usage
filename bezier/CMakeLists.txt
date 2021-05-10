cmake_minimum_required(VERSION 3.7)

add_compile_options(-std=c++11)

find_package(PythonLibs 2.7)

add_executable(gxf_bezier
        main.cpp
)   

target_include_directories(gxf_bezier  PRIVATE  ${PYTHON_INCLUDE_DIRS})
target_link_libraries(gxf_bezier  PRIVATE ${PYTHON_LIBRARIES})