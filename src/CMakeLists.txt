find_package(Magnum REQUIRED GL Sdl2Application
    MeshTools
    Primitives
    SceneGraph
    Shaders)
find_package(MagnumIntegration REQUIRED ImGui Eigen)
find_package(imgui REQUIRED Sources)

set_directory_properties(PROPERTIES CORRADE_USE_PEDANTIC_FLAGS ON)
include_directories(../toml11)

add_executable(CycloTron main.cpp track.cpp rider.cpp race.cpp)
target_link_libraries(CycloTron PRIVATE
    pthread
    Magnum::Application
    Magnum::GL
    Magnum::Magnum
    Magnum::MeshTools
    Magnum::Primitives
    Magnum::SceneGraph
    Magnum::Shaders
    MagnumIntegration::ImGui
    MagnumIntegration::Eigen)

# Make the executable a default target to build & run in Visual Studio
set_property(DIRECTORY ${PROJECT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT CycloTron)
