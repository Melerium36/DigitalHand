cmake_minimum_required(VERSION 3.20)
project(DigitalHandProjectiom LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


add_library(glad STATIC
    external/glad/src/glad.c
)
target_include_directories(glad PUBLIC
    external/glad/include
)

add_executable(app
    src/main.cpp
    src/utility/ShaderLoader.cpp
    src/opengl_api/build_shader.cpp
    src/imu_reader/Imu_reader.cpp
    src/MadgwickAlg/MadgwickAHRS.cpp
    src/DrawObjects/Handpart.cpp
    src/callibration/callibrator.cpp
)

find_package(glm REQUIRED) # freiwillig, da Headers schon da sind und keine Binärdatei dahinter

find_package(glfw3 REQUIRED)

find_package(OpenGL REQUIRED)

target_link_libraries(app PRIVATE
    glad
    glfw
    OpenGL::GL
    dl
)
