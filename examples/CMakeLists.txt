cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(mavlinkcpp_examples)

add_subdirectory(listener)
add_subdirectory(rid_listener)