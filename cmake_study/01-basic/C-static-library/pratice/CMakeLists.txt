cmake_minimum_required(VERSION 3.5)

project(pratice)

add_library(project_library STATIC src/hello.cpp)

target_include_directories(project_library PUBLIC ${CMAKE_SOURCE_DIR}/include)

add_executable(hello_pratice src/main.cpp)

target_link_libraries(hello_pratice PRIVATE project_library)