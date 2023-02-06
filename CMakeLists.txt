### BEGIN ARCHIBATE CMAKE TEMPLATE
cmake_minimum_required(VERSION 3.15)

if (DEFINED ENV{VCPKG_ROOT} AND NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    message(STATUS "VCPKG_ROOT: $ENV{VCPKG_ROOT}")
    set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake" CACHE STRING "")
endif()

if (DEFINED ENV{VCPKG_TRIPLET} AND NOT DEFINED VCPKG_TARGET_TRIPLET)
    message(STATUS "VCPKG_TRIPLET: $ENV{VCPKG_TRIPLET}")
    set(VCPKG_TARGET_TRIPLET "$ENV{VCPKG_TRIPLET}" CACHE STRING "")
endif()

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(NBaby LANGUAGES CXX)

if (PROJECT_BINARY_DIR STREQUAL PROJECT_SOURCE_DIR)
    message(FATAL_ERROR "The binary directory of CMake cannot be the same as source directory!"
        "Please consider use 'cmake -B build' to specify a different binary directory.")
endif()

message(STATUS "Build directory: ${PROJECT_BINARY_DIR}")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

if (WIN32)
    # get rid of 'min', 'max' macros; get back 'M_PI', 'M_E'
    add_definitions(-DNOMINMAX -D_USE_MATH_DEFINES)
endif()

if (NOT MSVC)  # speed up compilation by cache (https://ccache.dev)
    find_program(CCACHE_PROGRAM ccache)
    if (CCACHE_PROGRAM)
        message(STATUS "Found CCache: ${CCACHE_PROGRAM}")
        set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ${CCACHE_PROGRAM})
        set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ${CCACHE_PROGRAM})
    endif()
endif()
### END ARCHIBATE CMAKE TEMPLATE

file(GLOB source CONFIGURE_DEPENDS *.h *.cpp)
add_executable(${PROJECT_NAME} ${source})

find_package(OpenMP)
if (TARGET OpenMP::OpenMP_CXX)
    message(STATUS "Found OpenMP")
    target_link_libraries(${PROJECT_NAME} PRIVATE OpenMP::OpenMP_CXX)
    if (MSVC)
        include(CheckCXXCompilerFlag)
        check_cxx_compiler_flag("/openmp:experimental" MSVC_SUPPORT_OPENMP_EXPERIMENTAL)
        if (MSVC_SUPPORT_OPENMP_EXPERIMENTAL)
            target_compile_options(${PROJECT_NAME} PRIVATE $<IF:$<COMPILE_LANGUAGE:CXX>,/openmp:experimental,>)
        endif()
        check_cxx_compiler_flag("/openmp:llvm" MSVC_SUPPORT_OPENMP_LLVM)
        if (MSVC_SUPPORT_OPENMP_LLVM)
            target_compile_options(${PROJECT_NAME} PRIVATE $<IF:$<COMPILE_LANGUAGE:CXX>,/openmp:llvm,>)
        endif()
    endif()
endif()
