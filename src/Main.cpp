#include <iostream>
#include <sys/stat.h>
#include <vizkit3d/RobotModel.h>
#include <osgViewer/Viewer>
#include <QApplication>
#include <vizkit3d/Vizkit3DWidget.hpp>
#include <vizkit3d_world/Vizkit3dWorld.hpp>
#include <locale.h>


int main(int argc, char** argv) {

    if (argc < 2) {
        std::cerr << "error: the number of parameters is invalid." << std::endl;
        return 1;
    }

    vizkit3d_world::Vizkit3dWorld world(argv[1]);
    world.initialize();
    world.wait();
    world.deinitialize();


    return 0;

}

