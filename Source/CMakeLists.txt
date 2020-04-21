find_package(Magnum REQUIRED GL Sdl2Application
    MeshTools
    Primitives
    SceneGraph
    Shaders)

find_package(MagnumIntegration REQUIRED Bullet)


set_directory_properties(PROPERTIES CORRADE_USE_PEDANTIC_FLAGS ON)
include_directories(../toml11)
include_directories(../mio/single_include)
include_directories(../json/include)

add_executable(TrackSim TrackSim.cpp Track.cpp Rider.cpp Common.cpp)
target_link_libraries(TrackSim PRIVATE
    Magnum::Application
    Magnum::GL
    Magnum::Magnum
    Magnum::MeshTools
    Magnum::Primitives
    Magnum::SceneGraph
    Magnum::Shaders
    MagnumIntegration::Bullet) 

# Make the executable a default target to build & run in Visual Studio
set_property(DIRECTORY ${PROJECT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT TrackSim)
