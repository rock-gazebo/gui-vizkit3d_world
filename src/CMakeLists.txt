rock_library(vizkit3d_world
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(vizkit3d_world_bin Main.cpp
    DEPS vizkit3d_world)

