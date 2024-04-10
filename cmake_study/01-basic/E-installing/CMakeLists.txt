cmake_minimum_required(VERSION 3.5)

project(cmake_examples_install)

############################################################
# Create a library
############################################################

#Generate the shared library from the library sources
add_library(cmake_examples_inst SHARED
    src/hello.cpp
)

target_include_directories(cmake_examples_inst
    PUBLIC 
        ${PROJECT_SOURCE_DIR}/include
)

############################################################
# Create an executable
############################################################

# Add an executable with the above sources
add_executable(cmake_examples_inst_bin
    src/main.cpp
)

# link the new hello_library target with the hello_binary target
target_link_libraries( cmake_examples_inst_bin
    PRIVATE 
        cmake_examples_inst
)

############################################################
# Install
############################################################


install(TARGETS cmake_examples_inst_bin DESTINATION ${CMAKE_SOURCE_DIR}/install/bin)

install(TARGETS cmake_examples_inst LIBRARY DESTINATION ${CMAKE_SOURCE_DIR}/install/lib)

install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/ DESTINATION ${CMAKE_SOURCE_DIR}/install/include)

install(FILES cmake-exampls.conf DESTINATION ${CMAKE_SOURCE_DIR}/install/etc)
#/usr/local/bin
install(FILES cmake-exampls.conf DESTINATION  bin)


