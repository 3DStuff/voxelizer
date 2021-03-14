cmake_minimum_required(VERSION 3.12)
project(VoxelMagick LANGUAGES CXX)

#find_package(OpenCV)
find_package(GTest)
find_package(OpenMP)

set(BUILD_COMPILE_FLAGS "-Wextra -Wall")
add_definitions(${BUILD_COMPILE_FLAGS})

if(TARGET OpenMP::OpenMP_CXX)
    set(OMP_COMPILE_FLAGS "-fopenmp -DCMAKE_OMP_FOUND")
    add_definitions(${OMP_COMPILE_FLAGS})
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
endif()

set(pugixml_lib
    pugixml/src/pugixml.cpp
    pugixml/src/pugixml.hpp
    pugixml/src/pugiconfig.hpp
)
add_library(pugixml STATIC ${pugixml_lib})

include_directories(
    glm
    pugixml/src
)

set(voxel_lib
    polyhedron/glm_ext/glm_extensions.h
    polyhedron/mesh/bbox.h
    polyhedron/mesh/polyhedron.h
    polyhedron/mesh/polyhedron.tpp
    polyhedron/io/stl/types.h
    polyhedron/io/stl/stl_import.h
    polyhedron/io/stl/stl_import.cpp
    polyhedron/io/obj/obj_import.h
    polyhedron/io/obj/obj_import.cpp
    oqtree/oqtree.h
    rle/rle.h
    rle/rle_io.h
    voxelizer.hpp
    voxelizer.ipp
    enums.h
    rasterizer_fast.h
    rasterizer_hollow.h
    rasterizer_safe.h
    raster_results.h
    index_buffer.h
    buffer.h
    timer.h
    rules.h
    checks.h
    xml_config.h
    vox_file.h
    merge.h
    main.cpp
)

find_package(Git QUIET)
if(GIT_FOUND AND EXISTS "${PROJECT_SOURCE_DIR}/.git")
    # Update submodules as needed
    option(GIT_SUBMODULE "Check submodules during build" ON)
    if(GIT_SUBMODULE)
        message(STATUS "Submodule update")
        execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
                        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                        RESULT_VARIABLE GIT_SUBMOD_RESULT)

        if(NOT GIT_SUBMOD_RESULT EQUAL "0")
            message(FATAL_ERROR "git submodule update --init failed with ${GIT_SUBMOD_RESULT}, please checkout submodules")
        endif()
    endif()
endif()

add_subdirectory("polyhedron/tinyobjloader")

add_executable(VoxelMagick main.cpp ${voxel_lib})
target_link_libraries(VoxelMagick pugixml tinyobjloader) 
if(TARGET OpenMP::OpenMP_CXX)
    message(STATUS "OpenMP::OpenMP_CXX: " ${OpenMP_CXX_FLAGS})
    target_link_libraries(VoxelMagick ${OpenMP_CXX_FLAGS}) 
endif()

# Copy single files
macro(copy_ressource src trg)
    foreach(file ${src})
        message(STATUS "Copying resource ${file}")
        file(COPY ${file} DESTINATION ${trg})
    endforeach()
endmacro()

# Copy full directories
macro(copy_ressource src trg)
    foreach(dir ${src})
        # Replace / at the end of the path (copy dir content VS copy dir)
        string(REGEX REPLACE "/+$" "" dirclean "${dir}")
        message(STATUS "Copying resource ${dirclean}")
        file(COPY ${dirclean} DESTINATION ${trg})
    endforeach()
endmacro()

# copy ressources right into the build directory for playing
copy_ressource(examples/david ".")
copy_ressource(examples/cube ".")
copy_ressource(examples/eiffel ".")
copy_ressource(examples/ensemble ".")

if(GTEST_FOUND)
    message(STATUS "Google test found")
    ################################
    # Unit Tests
    ################################
    # Add test cpp file
    add_executable( runUnitTests rle/test_rle.cpp )
    # Link test executable against gtest & gtest_main
    target_link_libraries(runUnitTests gtest gtest_main)
    add_test( runUnitTests runUnitTests )
endif()
