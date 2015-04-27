/*
 * Vizkit3dWorld.cpp
 *
 *  Created on: Apr 20, 2015
 *      Author: gustavoneves
 */

#include <iostream>
#include <vizkit3d_world/Vizkit3dWorld.hpp>
#include <QString>

namespace vizkit3d_world {

Vizkit3dWorld::Vizkit3dWorld(std::string path, std::vector<std::string> modelPaths) {
    this->worldPath = QString(path.c_str());
    loadGazeboModelPaths(modelPaths);
}

Vizkit3dWorld::~Vizkit3dWorld() {
}

void Vizkit3dWorld::loadFromFile(QString path) {
    std::ifstream file(path.toStdString().c_str());
    std::string str((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    loadFromString(str);
}

void Vizkit3dWorld::loadFromString(const std::string xml) {
    sdf::SDFPtr sdf(new sdf::SDF);
    if (!sdf::init(sdf)) {
        throw std::runtime_error("unable to initialize sdf");
    }

    if (!sdf::readString(xml, sdf)) {
        throw std::invalid_argument("unable to load sdf from string " + xml + "\n");
    }

    if (!sdf->root->HasElement("world")) {
        throw std::invalid_argument("the SDF doesn't have a <world> tag\n");
    }

    makeWorld(sdf->root->GetElement("world"));
}

void Vizkit3dWorld::loadGazeboModelPaths(std::vector<std::string> modelPaths) {

    for (std::vector<std::string>::iterator it = modelPaths.begin(); it != modelPaths.end(); it++){
        sdf::addURIPath("model://", *it);
    }

    std::string home = std::string(getenv("HOME"));

    sdf::addURIPath("model://", home + "/.gazebo/models");

    char *pathCStr = getenv("GAZEBO_MODEL_PATH");

    if (!pathCStr || *pathCStr == '\0') {
        return;
    }

    std::string delim(":");
    std::string path = pathCStr;

    size_t pos1 = 0;
    size_t pos2 = path.find(delim);
    std::string substr;
    while (pos2 != std::string::npos) {
        substr = path.substr(pos1, pos2 - pos1);

        if (!substr.empty()) {
            sdf::addURIPath("model://", substr);
        }

        pos1 = pos2 + 1;
        pos2 = path.find(delim, pos2 + 1);
    }

    substr = path.substr(pos1, path.size() - pos1);

    if (!substr.empty()) {
        sdf::addURIPath("model://", substr);
    }
}

void Vizkit3dWorld::makeWorld(sdf::ElementPtr sdf) {

    if (sdf->HasElement("model")) {
        sdf::ElementPtr modelElem = sdf->GetElement("model");

        while (modelElem) {

            std::string modelName = modelElem->Get<std::string>("name");

            if (robotVizMap.find(modelName) == robotVizMap.end()){
                robotVizCountMap.insert(std::make_pair<std::string, int>(modelName, 0));
            }
            else {
                char buf[64];
                sprintf(buf, "%s_%d", modelName.c_str(), robotVizCountMap[modelName]++);
                modelName = buf;
            }

            vizkit3d::RobotVisualization* robotViz = robotVizFromSdfModel(modelElem);

            robotViz->setPluginName(modelName.c_str());
            robotVizMap.insert(std::make_pair<std::string, vizkit3d::RobotVisualization*>(modelName, robotViz));

            modelElem = modelElem->GetNextElement("model");

        }
    }
}

vizkit3d::RobotVisualization* Vizkit3dWorld::robotVizFromSdfModel(sdf::ElementPtr sdf_model) {
    sdf::Pose pose =  sdf_model->GetElement("pose")->Get<sdf::Pose>();
    base::samples::RigidBodyState rbs;
    rbs.position = base::Position(pose.pos.x, pose.pos.y, pose.pos.z);
    rbs.orientation = base::Orientation(pose.rot.w, pose.rot.x, pose.rot.y, pose.rot.z);

    vizkit3d::RobotVisualization* robotViz = new vizkit3d::RobotVisualization();

    std::string prefix;
    std::string modelstr = "<sdf version ='1.4'>" + sdf_model->ToString(prefix) + "</sdf>";
    sdf::SDF sdf;
    sdf.SetFromString(modelstr);

    robotViz->updateData(rbs);
    robotViz->loadFromString(QString(sdf.ToString().c_str()), QString("sdf"));

    return robotViz;
}

RobotVizMap Vizkit3dWorld::getRobotVizMap() {
    return robotVizMap;
}

void Vizkit3dWorld::attachPlugins()
{
    for (RobotVizMap::iterator it = robotVizMap.begin(); it != robotVizMap.end(); it++){
        vizkit3dThread.getWidget()->addPlugin(it->second);
    }
}

void Vizkit3dWorld::start(){
    loadFromFile(worldPath);
    vizkit3dThread.start();
    attachPlugins();
    vizkit3dThread.getWidget()->show();
}

void Vizkit3dWorld::stop(){
    vizkit3dThread.stop();
}

bool Vizkit3dWorld::isRunning(){
    return vizkit3dThread.isRunning();
}

void Vizkit3dWorld::updateModel(std::string modelName,
        base::samples::RigidBodyState pose,
        base::samples::Joints joints){

    RobotVizMap::iterator robotviz_it = robotVizMap.find(modelName);

    if (robotviz_it != robotVizMap.end()){
        robotviz_it->second->updateData(pose);

        if (joints.size() > 0){
            robotviz_it->second->updateData(joints);
        }
    }

}


}
