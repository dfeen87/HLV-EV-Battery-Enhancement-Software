cmake_minimum_required(VERSION 3.15)

project(
    HLV_Battery_Enhancement
    VERSION 1.1.1
    DESCRIPTION "HLV Theory Battery Management & BMS Middleware"
    LANGUAGES CXX
)

# ============================================================================
# Default Build Type
# ============================================================================

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# ============================================================================
# Project Configuration
# ============================================================================

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Build options
option(HLV_BUILD_EXAMPLES "Build example applications" ON)
option(HLV_BUILD_TESTS "Build unit tests" ON)
option(HLV_BUILD_BENCHMARKS "Build performance benchmarks" ON)
option(HLV_USE_GPU "Enable GPU acceleration (CUDA)" OFF)
option(HLV_INSTALL "Generate install target" ON)

# ============================================================================
# Header-Only Library Target
# ============================================================================

add_library(hlv_battery INTERFACE)

target_include_directories(hlv_battery INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(hlv_battery INTERFACE cxx_std_17)

# Public compile definitions (available to consumers)
target_compile_definitions(hlv_battery INTERFACE
    HLV_BMS_MIDDLEWARE_ENABLED
    HLV_HEADER_ONLY
)

# Warnings (applied per-target, not globally)
if(MSVC)
    target_compile_options(hlv_battery INTERFACE /W4)
else()
    target_compile_options(hlv_battery INTERFACE -Wall -Wextra -Wpedantic)
endif()

# ============================================================================
# GPU Support (Optional)
# ============================================================================

if(HLV_USE_GPU)
    find_package(CUDAToolkit QUIET)
    if(CUDAToolkit_FOUND)
        message(STATUS "CUDA toolkit found - enabling GPU support")
        target_compile_definitions(hlv_battery INTERFACE HLV_USE_GPU)
    else()
        message(WARNING "CUDA requested but not found - continuing without GPU")
    endif()
endif()

# ============================================================================
# Example Applications
# ============================================================================

if(HLV_BUILD_EXAMPLES)
    message(STATUS "Building examples")

    add_executable(basic_integration examples/basic_integration.cpp)
    target_link_libraries(basic_integration PRIVATE hlv_battery)

    # Note: multi_cell_pack.cpp currently has API mismatches and is disabled
    # add_executable(multi_cell_pack examples/multi_cell_pack.cpp)
    # target_link_libraries(multi_cell_pack PRIVATE hlv_battery)

    install(
        TARGETS basic_integration
        RUNTIME DESTINATION bin/examples
    )
endif()

# ============================================================================
# Unit Tests
# ============================================================================

if(HLV_BUILD_TESTS)
    message(STATUS "Building tests")
    enable_testing()

    add_executable(test_core tests/test_core.cpp)
    target_link_libraries(test_core PRIVATE hlv_battery)
    add_test(NAME test_core COMMAND test_core)

    add_executable(test_advanced tests/test_advanced.cpp)
    target_link_libraries(test_advanced PRIVATE hlv_battery)
    add_test(NAME test_advanced COMMAND test_advanced)

    add_executable(test_middleware tests/test_middleware.cpp)
    target_link_libraries(test_middleware PRIVATE hlv_battery)
    add_test(NAME test_middleware COMMAND test_middleware)
endif()

# ============================================================================
# Benchmarks
# ============================================================================

if(HLV_BUILD_BENCHMARKS)
    message(STATUS "Building benchmarks")

    add_executable(benchmark_update benchmarks/benchmark_update.cpp)
    target_link_libraries(benchmark_update PRIVATE hlv_battery)

    add_executable(benchmark_multicell benchmarks/benchmark_multicell.cpp)
    target_link_libraries(benchmark_multicell PRIVATE hlv_battery)
endif()

# ============================================================================
# Installation & Package Export
# ============================================================================

if(HLV_INSTALL)
    include(GNUInstallDirs)

    install(
        DIRECTORY include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    )

    install(
        TARGETS hlv_battery
        EXPORT hlv_battery-targets
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    )

    install(
        EXPORT hlv_battery-targets
        FILE hlv_battery-targets.cmake
        NAMESPACE hlv::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/hlv_battery
    )

    include(CMakePackageConfigHelpers)

    write_basic_package_version_file(
        "${CMAKE_CURRENT_BINARY_DIR}/hlv_battery-config-version.cmake"
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY SameMajorVersion
    )

    configure_package_config_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/hlv_battery-config.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/hlv_battery-config.cmake"
        INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/hlv_battery
    )

    install(
        FILES
            "${CMAKE_CURRENT_BINARY_DIR}/hlv_battery-config.cmake"
            "${CMAKE_CURRENT_BINARY_DIR}/hlv_battery-config-version.cmake"
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/hlv_battery
    )
endif()

# ============================================================================
# Packaging
# ============================================================================

set(CPACK_PACKAGE_NAME "HLV_Battery_Enhancement")
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "HLV Battery Management & BMS Middleware")
set(CPACK_PACKAGE_VENDOR "Don Michael Feeney Jr. & Marcel Krüger")
set(CPACK_PACKAGE_CONTACT "dfeen87@gmail.com")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
set(CPACK_RESOURCE_FILE_README "${CMAKE_CURRENT_SOURCE_DIR}/README.md")

include(CPack)

# ============================================================================
# Status Summary
# ============================================================================

message(STATUS "")
message(STATUS "=======================================")
message(STATUS "HLV Battery Enhancement Build Summary")
message(STATUS "=======================================")
message(STATUS "Version:        ${PROJECT_VERSION}")
message(STATUS "Build Type:     ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard:   ${CMAKE_CXX_STANDARD}")
message(STATUS "")
message(STATUS "Options:")
message(STATUS "  Examples:     ${HLV_BUILD_EXAMPLES}")
message(STATUS "  Tests:        ${HLV_BUILD_TESTS}")
message(STATUS "  Benchmarks:   ${HLV_BUILD_BENCHMARKS}")
message(STATUS "  GPU Support:  ${HLV_USE_GPU}")
message(STATUS "  Install:      ${HLV_INSTALL}")
message(STATUS "")
message(STATUS "Install Prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "=======================================")
message(STATUS "")
