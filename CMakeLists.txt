cmake_minimum_required(VERSION 3.24)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(namespace "mavlinkcpp")

project("mavlink-cpp"
    DESCRIPTION "Easily implement mavlink in C++"
    VERSION 0.0.1
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-address-of-packed-member")

if (NOT CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    message(STATUS "This project has a top-level one called [${CMAKE_PROJECT_NAME}]")
else()
    message(STATUS "This project is a top-level one")
endif()

add_library(${PROJECT_NAME} STATIC)

target_sources(${PROJECT_NAME}
PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Mavlink.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/UdpConnection.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/SerialConnection.cpp
)

#### Fix formatting
execute_process(COMMAND astyle --quiet --options=astylerc src/*.cpp,*.hpp)

##### Build mavlink
set(MAVLINK_DIALECT "development") # standard, development, etc
set(MAVLINK_GIT_DIR "${CMAKE_CURRENT_LIST_DIR}/lib/mavlink")
set(MAVLINK_LIBRARY_DIR "${CMAKE_BINARY_DIR}/lib/mavlink")

add_custom_target(mavlink_c_generate
        COMMAND ${PYTHON_EXECUTABLE} ${MAVLINK_GIT_DIR}/pymavlink/tools/mavgen.py
            --lang C --wire-protocol 2.0
            #--no-validate
            #--strict-units
            --output ${MAVLINK_LIBRARY_DIR}
            ${MAVLINK_GIT_DIR}/message_definitions/v1.0/${MAVLINK_DIALECT}.xml)

add_library(mavlink_c INTERFACE)
add_dependencies(mavlink_c mavlink_c_generate)
target_sources(mavlink_c INTERFACE ${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT}/${MAVLINK_DIALECT}.h)
set_source_files_properties(${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT}/${MAVLINK_DIALECT}.h PROPERTIES GENERATED true)
target_include_directories(mavlink_c INTERFACE ${MAVLINK_LIBRARY_DIR} ${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT})
target_compile_options(mavlink_c INTERFACE -Wno-address-of-packed-member -Wno-cast-align)

add_dependencies(${PROJECT_NAME} mavlink_c)

# Define headers for this library. PUBLIC headers are used for
# compiling the library, and will be added to consumers' build
# paths.
target_include_directories(${PROJECT_NAME}
    PRIVATE
        # where the library itself will look for its internal headers
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT}

    PUBLIC
        # where top-level project will look for the library's public headers
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT}>
        # where external projects will look for the library's public headers
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
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

# without it public headers won't get installed
set(public_headers
   ${CMAKE_CURRENT_SOURCE_DIR}/include/Mavlink.hpp
)
# note that ${public_headers} has to be in quotes
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
