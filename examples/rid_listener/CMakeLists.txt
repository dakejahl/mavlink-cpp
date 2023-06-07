cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(rid_listener VERSION 0.1 LANGUAGES CXX)

find_package(mavlink-cpp CONFIG REQUIRED)

add_executable(rid_listener)

target_sources(rid_listener
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/rid_listener.cpp
)

target_link_libraries(rid_listener
    mavlinkcpp::mavlink-cpp
)