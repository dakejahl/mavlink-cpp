cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(listener VERSION 0.1 LANGUAGES CXX)

find_package(mavlink-cpp CONFIG REQUIRED)

add_executable(listener)

target_sources(listener
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/listener.cpp
)

target_link_libraries(listener
    mavlinkcpp::mavlink-cpp
)