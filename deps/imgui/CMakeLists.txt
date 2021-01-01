cmake_minimum_required(VERSION 3.17)

project(imgui LANGUAGES CXX)

file(GLOB SOURCES_IMGUI ${PROJECT_SOURCE_DIR}/*.cpp)
add_library(imgui STATIC ${SOURCES_IMGUI})

find_package(glfw3 CONFIG REQUIRED)
find_package(GLEW REQUIRED)
target_link_libraries(imgui 
    PUBLIC glfw GLEW::GLEW
)
target_include_directories(imgui
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
)

