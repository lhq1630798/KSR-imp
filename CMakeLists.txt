cmake_minimum_required(VERSION 3.17)
project(KSR_imp LANGUAGES CXX)

find_package(CGAL CONFIG REQUIRED)
add_subdirectory("deps/imgui/")
add_subdirectory("deps/nfd/")
add_subdirectory("deps/fmt-7.1.3/")

set(KSR_SOURCES 
        "src/math/vec3.h"
        "src/log.h"
        "src/camera.h"
        "src/main.cpp"
        "src/Manager.h"
        "src/Manager.cpp"
        "src/app.h"
        "src/app.cpp"
        "src/gl_object.h"
        "src/cgal_object.h"
        "src/cgal_object.cpp"
        "src/intersection.cpp"
        "src/kinetic.cpp"
        "src/kinetic.h"
        "src/platform.h"
        "src/platform.cpp"
        "src/ransac.h"
        "src/ransac.cpp"
        "src/region_growing.h"
        "src/region_growing.cpp"
        "src/get_convex.cpp"
        "src/extract_surface/face_graph.cpp"
        "src/extract_surface/face_graph.h"
        "src/extract_surface/extract_surface.cpp"
        "src/extract_surface/extract_surface.h"
        "src/extract_surface/min-cut/block.h"
        "src/extract_surface/min-cut/graph.cpp"
        "src/extract_surface/min-cut/graph.h"
        "src/extract_surface/min-cut/maxflow.cpp"
        "src/extract_surface/min-cut/instances.inc"
        "src/extract_surface/build_map.h"
        "src/extract_surface/build_map.cpp"
        "src/extract_surface/label_polyhedron.h"
        "src/extract_surface/label_polyhedron.cpp"
        )
        
add_executable(KSR_imp ${KSR_SOURCES})
set_target_properties(KSR_imp PROPERTIES
                      CXX_STANDARD 17
                      CXX_EXTENSIONS OFF)
if(MSVC)
    target_compile_options(KSR_imp PRIVATE /W4 /wd4201 /wd4840 /wd4100 /fp:fast /MP )
    set_target_properties(KSR_imp PROPERTIES LINK_FLAGS /STACK:"100000000")

else()
    target_compile_options(KSR_imp PRIVATE -Wall -Wextra -Wno-reorder -Wno-unused-parameter )
    set_target_properties(KSR_imp PROPERTIES LINK_FLAGS -Wl,--stack, 100000000)
endif()
target_link_libraries(KSR_imp 
    PRIVATE CGAL::CGAL imgui nfd fmt::fmt
)
target_include_directories(KSR_imp 
    PRIVATE "deps/"
    PRIVATE "src/"
    PRIVATE "src/extract_surface/"
)