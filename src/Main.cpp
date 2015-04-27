#include <iostream>
#include <sys/stat.h>
#include <vizkit3d/RobotModel.h>
#include <osgViewer/Viewer>
#include <QApplication>
#include <vizkit3d/Vizkit3DWidget.hpp>
#include <vizkit3d_world/Vizkit3dWorld.hpp>

std::string getEnv(std::string varname) {
    char *varvalue = NULL;

    if ((varvalue = getenv(varname.c_str())) == NULL) {
        return "";
    }

    return std::string(varvalue);
}

//gui/robot_model is dependency of vizkit3d_model
//include gui/robot_model/test_data in the GAZEBO_MODEL_PATH variable
void loadRobotModelTestDataPath() {

    std::string rootpath = getEnv("AUTOPROJ_CURRENT_ROOT");
    std::string modelpath = getEnv("GAZEBO_MODEL_PATH");
    std::string testdatapath = rootpath + "/gui/robot_model/test_data";

    modelpath += ":" + testdatapath;

    setenv("GAZEBO_MODEL_PATH", modelpath.c_str(), 1);
}

int main(int argc, char** argv) {

    if (argc < 2) {
        std::cerr << "error: the number of parameters is invalid." << std::endl;
        return 1;
    }

    loadRobotModelTestDataPath();

    vizkit3d_world::Vizkit3dWorld world(argv[1]);
    world.start();

    while (world.isRunning()){
        usleep(200);
    }


    return 0;

}

