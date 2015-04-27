rock_find_cmake(Boost REQUIRED COMPONENTS system)

rock_library(vizkit3d_world
    SOURCES Vizkit3dWorld.cpp
    HEADERS Vizkit3dWorld.hpp
    LIBS ${Boost_SYSTEM_LIBRARY}
    DEPS_PKGCONFIG base-types sdformat vizkit3d robot_model-viz)

rock_executable(vizkit3d_world_bin Main.cpp
    DEPS vizkit3d_world)

