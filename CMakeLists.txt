cmake_minimum_required(VERSION 3.17)
project(KSR_imp LANGUAGES CXX)

# find_package(CGAL CONFIG REQUIRED OPTIONAL_COMPONENTS Qt5)
find_package(CGAL CONFIG REQUIRED)
add_subdirectory("deps/imgui/")

set(KSR_SOURCES 
        "src/math/vec3.h"
        "src/camera.h"
        "src/main.cpp"
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
        "src/extract_surface.cpp"
        "src/extract_surface.h"
        "src/min-cut/block.h"
        "src/min-cut/graph.cpp"
        "src/min-cut/graph.h"
        "src/min-cut/maxflow.cpp"
        "src/min-cut/instances.inc"
        )
        
add_executable(KSR_imp ${KSR_SOURCES})
set_target_properties(KSR_imp PROPERTIES
                      CXX_STANDARD 17
                      CXX_EXTENSIONS OFF)
if(MSVC)
    target_compile_options(KSR_imp PRIVATE /W4 /wd4201 /wd4840 /wd4100 /fp:fast /MP)
else()
    target_compile_options(KSR_imp PRIVATE -Wall -Wextra -Wno-reorder -Wno-unused-parameter)
endif()
target_link_libraries(KSR_imp 
    PRIVATE CGAL::CGAL imgui
)
target_include_directories(KSR_imp 
    PRIVATE "deps/"
)
