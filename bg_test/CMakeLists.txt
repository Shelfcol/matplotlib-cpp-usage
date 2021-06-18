cmake_minimum_required(VERSION 3.7)

add_compile_options(-std=c++14)

find_package(PythonLibs 2.7)
# include_directories(
#         include
# )

add_executable(main
        main.cpp
)   

target_include_directories(main  PRIVATE  ${PYTHON_INCLUDE_DIRS})
target_link_libraries(main  PRIVATE ${PYTHON_LIBRARIES})

add_executable(bg_test  bg_test.cpp
)   

target_include_directories(bg_test  PRIVATE  ${PYTHON_INCLUDE_DIRS})
target_link_libraries(bg_test  PRIVATE ${PYTHON_LIBRARIES})
