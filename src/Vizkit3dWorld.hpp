/*
 * Vizkit3dWorld.h
 *
 *  Created on: Apr 20, 2015
 *      Author: gustavoneves
 */

#ifndef GUI_VIZKIT3D_WORLD_SRC_VIZKIT3DWORLD_HPP_
#define GUI_VIZKIT3D_WORLD_SRC_VIZKIT3DWORLD_HPP_

#include <string>
#include <sdf/sdf.hh>
#include <vizkit3d/Vizkit3DWidget.hpp>
#include <vizkit3d/RobotVisualization.hpp>
#include <vizkit3d/QtThreadedWidget.hpp>
#include <map>

namespace vizkit3d_world {

typedef std::map<std::string, vizkit3d::RobotVisualization*> RobotVizMap;

/**
 * Vizkit3dWorld
 * set up vizkit3d instance from SDF
 */
class Vizkit3dWorld {

public:
    Vizkit3dWorld(std::string path = std::string(""), std::vector<std::string> modelPaths = std::vector<std::string>());
    virtual ~Vizkit3dWorld();

    //each model is stored in a map indexed by model name
    RobotVizMap getRobotVizMap();

    /*
        Start QtThreadedWidget
    */
    void start();

    void stop();

    bool isRunning();

    void updateModel(std::string modelName,
            base::samples::RigidBodyState pose,
            base::samples::Joints joints);

protected:

    void loadFromFile(QString filename);

    void loadFromString(const std::string xml);

    void loadGazeboModelPaths(std::vector<std::string> modelPaths = std::vector<std::string>());

    void attachPlugins();

    vizkit3d::RobotVisualization* robotVizFromSdfModel(sdf::ElementPtr sdf_model);

    void makeWorld(sdf::ElementPtr sdf);

    RobotVizMap robotVizMap;
    QtThreadedWidget<vizkit3d::Vizkit3DWidget> vizkit3dThread;
    std::map<std::string, int> robotVizCountMap;
    QString worldPath;
};

}

#endif /* GUI_VIZKIT3D_WORLD_SRC_VIZKIT3DWORLD_HPP_ */
