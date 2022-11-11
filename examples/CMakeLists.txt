cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(mavlinkcpp_examples)

list(APPEND CMAKE_PREFIX_PATH "../../install")

add_subdirectory(listener)