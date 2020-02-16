cmake_minimum_required(VERSION 3.8)

project(app CXX)

# Some basics, set accordingly to your needs.
set(CMAKE_CXX_STANDARD 17)

find_package(Magnum REQUIRED
    GL
    MeshTools
    Primitives
    SceneGraph
    Shaders
    Sdl2Application)

set_directory_properties(PROPERTIES CORRADE_USE_PEDANTIC_FLAGS ON)

# Your application target.
add_executable(anttrack main.cpp track.cpp rider.cpp)
target_link_libraries(anttrack PRIVATE
    Magnum::Application
    Magnum::GL
    Magnum::Magnum
    Magnum::MeshTools
    Magnum::Primitives
    Magnum::SceneGraph
    Magnum::Shaders)
