find_package(Magnum REQUIRED GL Sdl2Application
    MeshTools
    Primitives
    SceneGraph
    Shaders)
find_package(MagnumIntegration REQUIRED Eigen)
find_package(cpprestsdk REQUIRED)

set_directory_properties(PROPERTIES CORRADE_USE_PEDANTIC_FLAGS ON)
include_directories(../toml11)
include_directories(../mio/single_include)
include_directories(../json/include)

add_executable(CycloTron utils.cpp main.cpp gamelogic.cpp track.cpp rider.cpp race.cpp)
target_link_libraries(CycloTron PRIVATE
    Magnum::Application
    Magnum::GL
    Magnum::Magnum
    Magnum::MeshTools
    Magnum::Primitives
    Magnum::SceneGraph
    Magnum::Shaders) 

# Make the executable a default target to build & run in Visual Studio
set_property(DIRECTORY ${PROJECT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT CycloTron)
