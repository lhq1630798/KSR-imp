cmake_minimum_required(VERSION 3.17)
project(KSR_imp LANGUAGES CXX)

find_package(CGAL CONFIG REQUIRED)
find_package(tomlplusplus CONFIG REQUIRED)
add_subdirectory("deps/imgui/")
add_subdirectory("deps/nfd/")
add_subdirectory("deps/fmt-7.1.3/")

set(KSR_SOURCES 
        "config.toml"

        "src/main.cpp"
        "src/Manager.h"
        "src/Manager.cpp"

        "src/util/convex.h"
        "src/util/convex.cpp"
        "src/util/log.h"
        "src/util/config.h"
        "src/util/config.cpp"

        "src/gui/camera.h"
        "src/gui/app.h"
        "src/gui/app.cpp"
        "src/gui/gl_object.h"
        "src/gui/platform.h"
        "src/gui/platform.cpp"

        "src/cgal/cgal_object.h"
        "src/cgal/cgal_object.cpp"

        # "src/partition/kinetic.cpp"
        # "src/partition/kinetic.h"
        "src/partition/BSP.cpp"
        "src/partition/BSP.h"

        # "src/detect_shape/ransac.h"
        # "src/detect_shape/ransac.cpp"
        "src/detect_shape/region_growing.h"
        "src/detect_shape/region_growing.cpp"
        "src/detect_shape/detect_shape.cpp"
        "src/detect_shape/detect_shape.h"
        "src/detect_shape/hierarchical.h"
        "src/detect_shape/hierarchical.cpp"
        "src/detect_shape/shape_diameter.cpp"
        "src/detect_shape/shape_diameter.h"
        "src/detect_shape/structure_detect.cpp"
        "src/detect_shape/structure_detect.h"
        
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
    PRIVATE CGAL::CGAL imgui nfd fmt::fmt tomlplusplus::tomlplusplus
)
target_include_directories(KSR_imp 
    PRIVATE "deps/"
    PRIVATE "src/"
)