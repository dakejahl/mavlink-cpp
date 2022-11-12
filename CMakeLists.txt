cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(namespace "mavlinkcpp")

project("mavlink-cpp"
    DESCRIPTION "Easily implement mavlink in C++"
    VERSION 0.0.1
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-address-of-packed-member")

find_package(Threads REQUIRED)
find_package(Python COMPONENTS Interpreter)

if (NOT CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    message(STATUS "This project has a top-level one called [${CMAKE_PROJECT_NAME}]")
else()
    message(STATUS "This project is a top-level one")
endif()

add_library(${PROJECT_NAME} STATIC)

target_sources(${PROJECT_NAME}
PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Connection.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/UdpConnection.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/SerialConnection.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Mavlink.cpp
)

execute_process(COMMAND astyle --quiet --options=astylerc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp,*.hpp,*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h,*.hpp
)

##### Build mavlink from source
set(MAVLINK_DIALECT "development") # standard, development, etc
set(MAVLINK_GIT_DIR "${CMAKE_CURRENT_LIST_DIR}/lib/mavlink")
set(MAVLINK_LIBRARY_DIR "${CMAKE_BINARY_DIR}/lib/mavlink")

execute_process(
    COMMAND ${Python_EXECUTABLE} ${MAVLINK_GIT_DIR}/pymavlink/tools/mavgen.py
        --lang C --wire-protocol 2.0
        #--no-validate
        #--strict-units
        --output ${MAVLINK_LIBRARY_DIR}
        ${MAVLINK_GIT_DIR}/message_definitions/v1.0/${MAVLINK_DIALECT}.xml
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

add_library(mavlink_c INTERFACE)
target_sources(mavlink_c INTERFACE ${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT}/${MAVLINK_DIALECT}.h)
set_source_files_properties(${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT}/${MAVLINK_DIALECT}.h PROPERTIES GENERATED true)
target_include_directories(mavlink_c INTERFACE ${MAVLINK_LIBRARY_DIR} ${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT})
target_compile_options(mavlink_c INTERFACE -Wno-address-of-packed-member -Wno-cast-align)

add_dependencies(${PROJECT_NAME} mavlink_c)

target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})

##########################################################
# The below is taken from:
# https://decovar.dev/blog/2021/03/08/cmake-cpp-library/
##########################################################

# Define headers for this library. PUBLIC headers are used for
# compiling the library, and will be added to consumers' build
# paths.
target_include_directories(${PROJECT_NAME}
    PRIVATE
        # where the library itself will look for its internal headers
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/include

    PUBLIC
        # where top-level project will look for the library's public headers
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT}>
        # where external projects will look for the library's public headers
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
        $<INSTALL_INTERFACE:build/lib/mavlink/${MAVLINK_DIALECT}>
)

# note that it is not CMAKE_INSTALL_PREFIX we are checking here
if(DEFINED CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    message(
        STATUS
        "CMAKE_INSTALL_PREFIX is not set\n"
        "Default value: ${CMAKE_INSTALL_PREFIX}\n"
        "Will set it to ${CMAKE_SOURCE_DIR}/install"
    )
    set(CMAKE_INSTALL_PREFIX
        "${CMAKE_SOURCE_DIR}/install"
        CACHE PATH "Where the library will be installed to" FORCE
    )
else()
    message(
        STATUS
        "CMAKE_INSTALL_PREFIX was already set\n"
        "Current value: ${CMAKE_INSTALL_PREFIX}"
    )
endif()

# Set public header list, this is the list of headers to be installed
set(public_headers
   ${CMAKE_CURRENT_SOURCE_DIR}/include/Mavlink.hpp
   ${CMAKE_CURRENT_SOURCE_DIR}/include/ThreadSafeQueue.hpp
   ${CMAKE_CURRENT_SOURCE_DIR}/include/helpers.hpp
)

# Copy mavlink headers into install
file(GLOB_RECURSE mavlink_headers ${CMAKE_CURRENT_SOURCE_DIR}/build/lib/mavlink/*.h)
list(APPEND public_headers ${mavlink_headers})

set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "${public_headers}")

# for CMAKE_INSTALL_INCLUDEDIR definition
include(GNUInstallDirs)

# the variant with PUBLIC_HEADER property unfortunately does not preserve the above folder structure
# so instead we iterate through public headers and install them "manually"
foreach(header ${public_headers})
    file(RELATIVE_PATH header_file_path "${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}" "${header}")
    get_filename_component(header_directory_path "${header_file_path}" DIRECTORY)
    install(
        FILES ${header}
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${header_directory_path}"
    )
endforeach()

# append suffix 'd' for debug builds
set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "d")

# install the target and create export-set
install(TARGETS ${PROJECT_NAME}
    EXPORT "${PROJECT_NAME}Targets"
    # these get default values from GNUInstallDirs, no need to set them
    #RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} # bin
    #LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} # lib
    #ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} # lib
    # except for public headers, as we want them to be inside a library folder
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# generate and install export file
install(EXPORT "${PROJECT_NAME}Targets"
    FILE "${PROJECT_NAME}Targets.cmake"
    NAMESPACE ${namespace}::
    DESTINATION cmake
)

include(CMakePackageConfigHelpers)

# generate the version file for the config file
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    VERSION "${version}"
    COMPATIBILITY AnyNewerVersion
)
# create config file
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    INSTALL_DESTINATION cmake
)
# install config files
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    DESTINATION cmake
)
# generate the export targets for the build tree
export(EXPORT "${PROJECT_NAME}Targets"
    FILE "${CMAKE_CURRENT_BINARY_DIR}/cmake/${PROJECT_NAME}Targets.cmake"
    NAMESPACE ${namespace}::
)
