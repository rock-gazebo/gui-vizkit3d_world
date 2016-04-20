#include <iostream>
#include <sys/stat.h>
#include <vizkit3d/RobotModel.h>
#include <osgViewer/Viewer>
#include <QApplication>
#include <vizkit3d/Vizkit3DWidget.hpp>
#include <vizkit3d_world/Vizkit3dWorld.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <frame_helper/FrameHelper.h>

vizkit3d_world::Vizkit3dWorld *g_world = NULL;

int main(int argc, char** argv) {

    if (argc < 2) {
        std::cerr << "error: the number of parameters is invalid." << std::endl;
        return 1;
    }

    vizkit3d_world::Vizkit3dWorld g_world(argv[1], std::vector<std::string>());
    g_world.enableGrabbing();

    base::samples::frame::Frame* frame = new base::samples::frame::Frame();
    g_world.grabFrame(*frame);
    cv::Mat mat = frame_helper::FrameHelper::convertToCvMat(*frame);
    cv::imwrite("cvfile.png", mat);

    return 0;

}

