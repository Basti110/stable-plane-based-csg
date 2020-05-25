cmake_minimum_required(VERSION 3.8)
project(PlaneOctreeCSG)

# ===============================================
# Global settings

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set(BUILD_SHARED_LIBS ON CACHE BOOL "" FORCE) # required by some dependencies (e.g. gtest)

#option(CSG_USE_GUROBI "Enable gurobi for testing" Off)

# ==============================================================================
# Set bin dir
if(MSVC)
    set(BIN_DIR ${CMAKE_SOURCE_DIR}/bin)
elseif(CMAKE_BUILD_TYPE STREQUAL "")
    set(BIN_DIR ${CMAKE_SOURCE_DIR}/bin/Default)
else()
    set(BIN_DIR ${CMAKE_SOURCE_DIR}/bin/${CMAKE_BUILD_TYPE})
endif()
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BIN_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${BIN_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO ${BIN_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${BIN_DIR})
set(GLOW_BIN_DIR ${CMAKE_SOURCE_DIR}/bin)

# ===============================================
# Dependencies

# std forward declarations
#set(STDFWD_FORCE_INCLUDE ON CACHE BOOL "" FORCE)
#add_subdirectory(extern/cpp-std-fwd)

# Add GLFW lib (with disabled spam)
option(GLFW_BUILD_EXAMPLES "" OFF)
option(GLFW_BUILD_TESTS "" OFF)
option(GLFW_BUILD_DOCS "" OFF)
option(GLFW_INSTALL "" OFF)
add_subdirectory(extern/glfw)

# lean standardlibrary replacement
add_subdirectory(extern/clean-core)

# math library
add_subdirectory(extern/typed-geometry)

# graphics library
add_subdirectory(extern/glow)

# UI lib
add_subdirectory(extern/imgui-lean)

# mesh library
add_subdirectory(extern/polymesh)

# graphics extras library
add_subdirectory(extern/glow-extras)

# nexus
add_subdirectory(extern/nexus)

# ===============================================
# Configure executable

file(GLOB_RECURSE SOURCES
    "src/*.cc"
    "include/*.hh"
    "src/*.cpp"
    "include/*.h"
    "data/*.*sh"
    "data/*.glsl"
)

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${SOURCES})

add_executable(${PROJECT_NAME} ${SOURCES})

target_link_libraries(${PROJECT_NAME} PUBLIC
    clean-core
    glow
    glow-extras
    typed-geometry
    polymesh
    imgui
    nexus
)

target_include_directories(${PROJECT_NAME} PUBLIC "include")

# Add gurobi if necessary


get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "dir='${dir}'")
endforeach()

# ===============================================
# Compile flags

if (MSVC)
    target_compile_options(${PROJECT_NAME} PUBLIC
        /MP
    )
else()
    target_compile_options(${PROJECT_NAME} PUBLIC
        -Wall
        -Wno-unused-variable
        -march=native
    )
    target_link_libraries(${PROJECT_NAME} PUBLIC
        -lstdc++fs
        -lgmp
        -lz
        -lgmpxx
        -lpthread
    )
endif()

if(NOT MSVC)
# Enable GMP for Soplex
target_compile_definitions(${PROJECT_NAME} PRIVATE SOPLEX_WITH_GMP=1)
endif()

# ===============================================
# Folder grouping

#include(cmake/SourceGroup.cmake)
#arcana_source_group(SOURCES)

foreach(TARGET_NAME
    glfw
    aion
    glow
    polymesh
    typed-geometry-src
    imgui
)
    if (TARGET ${TARGET_NAME})
        set_property(TARGET ${TARGET_NAME} PROPERTY FOLDER "Extern")
    endif()
endforeach()
