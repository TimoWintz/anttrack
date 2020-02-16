find_package(Magnum REQUIRED GL Sdl2Application
    MeshTools
    Primitives
    SceneGraph
    Shaders)

set_directory_properties(PROPERTIES CORRADE_USE_PEDANTIC_FLAGS ON)

add_executable(AntTrack main.cpp track.cpp rider.cpp)
target_link_libraries(AntTrack PRIVATE
    Magnum::Application
    Magnum::GL
    Magnum::Magnum
    Magnum::MeshTools
    Magnum::Primitives
    Magnum::SceneGraph
    Magnum::Shaders)

# Make the executable a default target to build & run in Visual Studio
set_property(DIRECTORY ${PROJECT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT AntTrack)
