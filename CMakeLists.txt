cmake_minimum_required(VERSION 3.5)
project(VoxelMagick LANGUAGES CXX)

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

find_package( OpenCV REQUIRED )
find_package(OpenMP)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
endif()

include_directories(
    glm
    pugixml/src
)

set(voxel_lib
    polyhedron/glm_ext/glm_extensions.h
    polyhedron/mesh/polyhedron.h
    polyhedron/mesh/polyhedron.tpp
    polyhedron/stl/stl_import.h
    polyhedron/stl/stl_import.cpp
    oqtree/oqtree.h
    voxelizer.hpp
    voxelizer.ipp
    enums.h
    rasterizer.h
    buffer.h
    timer.h
    rules.h
    checks.h
    xml_config.h
    vox_file.h
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

add_subdirectory(pugixml)
link_directories(pugixml)


add_executable(VoxelMagick main.cpp ${voxel_lib})
if(OpenMP_CXX_FOUND)
    target_link_libraries(VoxelMagick pugixml OpenMP::OpenMP_CXX ${OpenCV_LIBS})
else()
    target_link_libraries(VoxelMagick pugixml ${OpenCV_LIBS})
endif()
