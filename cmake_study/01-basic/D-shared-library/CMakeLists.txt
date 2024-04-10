cmake_minimum_required(VERSION 3.5)

project(hello_library)

add_library(shared_library SHARED src/hello.cc)

add_library(hello::library ALIAS shared_library)

target_include_directories(shared_library PUBLIC ${CMAKE_SOURCE_DIR}/include)

add_executable(hello_shared_library src/main.cc)

target_link_libraries(hello_shared_library PRIVATE hello::library)



