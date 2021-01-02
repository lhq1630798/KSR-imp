cmake_minimum_required(VERSION 3.17)

project(imgui LANGUAGES CXX)

file(GLOB SOURCES_IMGUI ${PROJECT_SOURCE_DIR}/*.cpp)
add_library(imgui STATIC ${SOURCES_IMGUI})

find_package(glfw3 CONFIG REQUIRED)
add_subdirectory("glad/")
target_link_libraries(imgui 
    PUBLIC glfw glad
)
target_include_directories(imgui
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
)

