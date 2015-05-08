rock_find_cmake(Boost REQUIRED COMPONENTS system thread date_time chrono)

rock_library(vizkit3d_world
    SOURCES 
        Vizkit3dWorld.cpp

    MOC 
        Vizkit3dWorld.hpp

    HEADERS
        Vizkit3dWorld.hpp

    LIBS
        ${Boost_THREAD_LIBRARY}
        ${Boost_SYSTEM_LIBRARY}
        ${Boost_DATE_TIME_LIBRARY}
        ${Boost_CHRONO_LIBRARY}
        
    DEPS_PKGCONFIG
        base-types
        sdformat
        vizkit3d
        robot_model-viz)

rock_executable(vizkit3d_world_bin Main.cpp
    DEPS vizkit3d_world)

