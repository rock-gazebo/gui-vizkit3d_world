/*
 * Vizkit3dWorld.cpp
 *
 *  Created on: Apr 20, 2015
 *      Author: gustavoneves
 */

#include <iostream>
#include <QString>
#include <QtGui>
#include <QtCore>

#include <boost/algorithm/string.hpp>
#include <vizkit3d_world/Vizkit3dWorld.hpp>
#include <osgViewer/View>
#include <base-logging/Logging.hpp>
#include "Utils.hpp"
#include <kdl_parser/RobotModelFormat.hpp>
#include <locale.h>     /* struct lconv, setlocale, localeconv */

namespace vizkit3d_world {

Vizkit3dWorld::Vizkit3dWorld(std::string path, 
                            std::vector<std::string> modelPaths,
                            std::vector<std::string> ignoredModels,
                            int cameraWidth, int cameraHeight, 
                            double horizontalFov, 
                            double zNear, double zFar, int number_of_widgets)
    : worldPath(path)
    , modelPaths(modelPaths)
    , app(NULL)
    , cameraWidth((cameraWidth <= 0) ? 800 : cameraWidth)
    , cameraHeight((cameraHeight <= 0) ? 600 : cameraHeight)
    , zNear(zNear)
    , zFar(zFar)
    , horizontalFov(horizontalFov)
    , number_of_widgets(number_of_widgets)
{
    
    loadGazeboModelPaths(modelPaths);

    int argc = 1;
    char const*argv[] = { "vizkit3d_world" };
    app = new QApplication(argc, const_cast<char**>(argv));
    //Qt application changes the locale information, what crashes the sdf initialization
    setlocale(LC_ALL, "C");

    //main widget to store the plugins and performs the GUI events
    widget.reserve(number_of_widgets);

    for (int i = 0; i < number_of_widgets; ++i){
    
        widget[i] = new vizkit3d::Vizkit3DWidget(NULL, cameraWidth, cameraHeight, "world_osg", false);

        widget[i]->setFixedSize(cameraWidth, cameraHeight); //set the window size
        widget[i]->getView(0)->getCamera()->
            setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
        widget[i]->setAxes(false);
        widget[i]->setAxesLabels(false);
        widget[i]->getPropertyWidget()->hide(); //hide the right property widget
        applyCameraParams(i);
    }

    this->ignoredModels = ignoredModels;
    //load the world sdf file and create the vizkit3d::RobotVisualization models
    //It is necessary to create the vizkit3d plugins in the same thread of QApplication
    loadFromFile(worldPath);
    attachPlugins();

    //apply the tranformations in each model
    for (int i = 0; i < number_of_widgets; ++i){
        applyTransformations(i);
        widget[i]->setCameraManipulator(vizkit3d::NO_MANIPULATOR);
    }
}

Vizkit3dWorld::~Vizkit3dWorld()
{
    app->closeAllWindows();
    app->quit();

    for (int i = 0; i < number_of_widgets; ++i){
        delete widget[i];
    }

    delete app;
    toSdfElement.clear();
    robotVizMap.clear();
}

const int Vizkit3dWorld::getNumberOfWidgets()
{
    return number_of_widgets;
}

void Vizkit3dWorld::loadFromFile(std::string path) {
    std::pair<std::string, int> sdf_string = getRobotModelString(
        path, kdl_parser::ROBOT_MODEL_FORMAT::ROBOT_MODEL_AUTO);
    loadFromString(sdf_string.first);
}

void Vizkit3dWorld::loadFromString(const std::string xml) {
    sdf::SDFPtr sdf(new sdf::SDF);
    if (!sdf::init(sdf)) {
        throw std::runtime_error("unable to initialize sdf");
    }

    if (!sdf::readString(xml, sdf)) {
        throw std::invalid_argument("unable to load sdf from string " + xml + "\n");
    }

    if (!sdf->Root()->HasElement("world")) {
        throw std::invalid_argument("the SDF doesn't have a <world> tag\n");
    }

    makeWorld(sdf->Root()->GetElement("world"), sdf->Version());
}

void Vizkit3dWorld::loadGazeboModelPaths(std::vector<std::string> modelPaths) {

    for (std::vector<std::string>::iterator it = modelPaths.begin(); it != modelPaths.end(); it++){
        sdf::addURIPath("model://", *it);
    }

    std::string home = getEnv("HOME");

    sdf::addURIPath("model://", home + "/.gazebo/models");

    std::string path = getEnv("GAZEBO_MODEL_PATH") + std::string(":") + getEnv("PATH");

    std::vector<std::string> vec;
    boost::algorithm::split(vec, path, boost::algorithm::is_any_of(":"), boost::algorithm::token_compress_on);

    for (std::vector<std::string>::iterator it = vec.begin(); it != vec.end(); it++){
        if (!(*it).empty()){
            sdf::addURIPath("model://", *it);
        }
    }
}

void Vizkit3dWorld::makeWorld(sdf::ElementPtr sdf, std::string version) {
    // Transformations reported by rock-gazebo now always use "world" for the world,
    // regardless of the actual world name. Insert such a frame, but keep a frame
    // with the actual world name in case someone is feeding data that uses it.
    std::string worldName = sdf->Get<std::string>("name");
    if (worldName != "world") {
        for (int i = 0; i< number_of_widgets; ++i)
            applyTransformation(worldName, "world", QVector3D(), QQuaternion(), i);
    }
    
    if (sdf->HasElement("model")) {

        std::map<std::string, int> robotVizCountMap;

        sdf::ElementPtr modelElem = sdf->GetElement("model");

        while (modelElem) {

            std::string modelName = modelElem->Get<std::string>("name");

            /**
             * this code is used to add the models with the same name
             * but it is necessary to change control/kdl_parser
             * and control/sdf_ruby to change the base segment name
             */
            if (robotVizMap.find(modelName) == robotVizMap.end()){
                robotVizCountMap.insert(std::make_pair(modelName, 0));
            }
            else {
                std::ostringstream buf;
                buf << modelName << "_" << (robotVizCountMap[modelName]++);
                modelName = buf.str();
            }

            if(std::find(ignoredModels.begin(), ignoredModels.end(), modelName) == ignoredModels.end()){
                vizkit3d::RobotVisualization* robotViz = robotVizFromSdfModel(modelElem, modelName, version);
                robotVizMap.insert(std::make_pair(modelName, robotViz));
            }

            modelElem = modelElem->GetNextElement("model");

        }
    }
}

vizkit3d::RobotVisualization* Vizkit3dWorld::robotVizFromSdfModel(sdf::ElementPtr sdf_model, std::string modelName, std::string version) {

    //vizkit3d plugin with model defined in the sdf model
    vizkit3d::RobotVisualization* robotViz = new vizkit3d::RobotVisualization();

    std::string prefix;
    std::string modelstr = "<sdf version='" +  version + "'>" + sdf_model->ToString(prefix) + "</sdf>";

    sdf::SDF sdf;
    sdf.SetFromString(modelstr);
    robotViz->loadFromString(QString(sdf.ToString().c_str()), QString("sdf"));
    robotViz->setPluginName(modelName.c_str());
    robotViz->relocateRoot(modelName);

    toSdfElement.insert(std::make_pair(modelName, sdf_model));


    return robotViz;
}

RobotVizMap Vizkit3dWorld::getRobotVizMap() {
    return robotVizMap;
}

void Vizkit3dWorld::attachPlugins()
{
    for (RobotVizMap::iterator it = robotVizMap.begin(); it != robotVizMap.end(); it++){
        for (int i = 0; i < number_of_widgets; ++i){
            widget[i]->addPlugin(it->second);
            it->second->setParent(widget[i]);
            //it is necessary to add to widget first and set the parent widget
            it->second->setVisualizationFrame(it->first.c_str());
        }
    }
}

vizkit3d::RobotVisualization* Vizkit3dWorld::getRobotViz(std::string name)
{
    RobotVizMap::iterator robotviz_it = robotVizMap.find(name);
    return (robotviz_it == robotVizMap.end()) ? NULL : robotviz_it->second;
}

void Vizkit3dWorld::setJoints(std::string modelName, base::samples::Joints joints) {
    vizkit3d::RobotVisualization *viz;
    if ((viz = getRobotViz(modelName)) != NULL) {
        viz->updateData(joints);
    }
}

sdf::ElementPtr Vizkit3dWorld::getSdfElement(std::string name) {
    std::map<std::string, sdf::ElementPtr>::iterator it = toSdfElement.find(name);
    return (it == toSdfElement.end()) ? sdf::ElementPtr() : it->second;
}

void Vizkit3dWorld::applyTransformations(int widget_number) {

    for (RobotVizMap::iterator it = robotVizMap.begin();
            it != robotVizMap.end(); it++){

        sdf::ElementPtr sdfModel = getSdfElement(it->first);

        ignition::math::Pose3d pose =  sdfModel->GetElement("pose")->Get<ignition::math::Pose3d>();

        applyTransformation(
            "world", it->first,
            QVector3D(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z()),
            QQuaternion(pose.Rot().W(), pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z()),
            widget_number);

    }
}

void Vizkit3dWorld::applyTransformation(base::samples::RigidBodyState rbs, int widget_number) {
    applyTransformation(rbs.targetFrame,
                        rbs.sourceFrame,
                        rbs.position,
                        rbs.orientation,
                        widget_number);
}

void Vizkit3dWorld::applyTransformation(
    std::string targetFrame, 
    std::string sourceFrame, 
    base::Position position, 
    base::Orientation orientation, 
    int widget_number) 
{
    applyTransformation(
        targetFrame, sourceFrame,
        QVector3D(position.x(), position.y(), position.z()),
        QQuaternion(orientation.w(), orientation.x(), orientation.y(), orientation.z()),
        widget_number);
}

void Vizkit3dWorld::applyTransformation(
    std::string targetFrame, 
    std::string sourceFrame, 
    QVector3D position, 
    QQuaternion orientation,
    int widget_number) 
{

    if (widget[widget_number]) {
        if (!targetFrame.empty() && !sourceFrame.empty()){
            widget[widget_number]->setTransformation(QString::fromStdString(targetFrame),
                                      QString::fromStdString(sourceFrame),
                                      position,
                                      orientation);

            widget[widget_number]->setTransformer(false);
        }
        else {
            LOG_WARN("it is necessary to inform the target and source frames.");
        }
    }
}



void Vizkit3dWorld::setTransformation(base::samples::RigidBodyState rbs, int widget_number) {
    applyTransformation(rbs, widget_number);
}

void Vizkit3dWorld::setCameraPose(base::samples::RigidBodyState pose, int widget_number) {
    Eigen::Vector3d look_at = pose.position + pose.orientation * Eigen::Vector3d::UnitX();
    Eigen::Vector3d up      = pose.orientation * Eigen::Vector3d::UnitZ();
    Eigen::Vector3d eye     = pose.position;

    /**
     * Set the new camera position
     */
    widget[widget_number]->setCameraEye(eye.x(), eye.y(), eye.z());
    widget[widget_number]->setCameraLookAt(look_at.x(), look_at.y(), look_at.z());
    widget[widget_number]->setCameraUp(up.x(), up.y(), up.z());
}


//internal enable and disable grabbing
void Vizkit3dWorld::enableGrabbing(int widget_number)
{
    widget[widget_number]->enableGrabbing();
}

void Vizkit3dWorld::disableGrabbing(int widget_number)
{
    widget[widget_number]->disableGrabbing();
}

QImage Vizkit3dWorld::grabImage(int widget_number)
{
    return widget[widget_number]->grab();
}

//grab frame
//convert QImage to base::samples::frame::Frame
void Vizkit3dWorld::grabFrame(base::samples::frame::Frame& frame, int widget_number)
{
    QImage image = grabImage(widget_number);
    cvtQImageToFrame(image, frame, 
        (widget[widget_number]->isVisible() && !widget[widget_number]->isMinimized()));
}

void Vizkit3dWorld::setCameraParams(int cameraWidth, int cameraHeight, 
    double horizontalFov, double zNear, double zFar, int widget_number) 
{
    this->cameraWidth = cameraWidth;
    this->cameraHeight = cameraHeight;
    this->horizontalFov = horizontalFov;
    this->zNear = zNear;
    this->zFar = zFar;
    applyCameraParams(widget_number);
}

void Vizkit3dWorld::applyCameraParams(int widget_number) {
    double aspectRatio = cameraWidth/cameraHeight;
    double fovy =  osg::DegreesToRadians(horizontalFov) / aspectRatio;
    widget[widget_number]->getView(0)->getCamera()->setProjectionMatrixAsPerspective(osg::RadiansToDegrees(fovy), aspectRatio, zNear, zFar);
}

}
