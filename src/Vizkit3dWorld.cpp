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
#include <QEvent>
#include <csignal>

#include <boost/algorithm/string.hpp>
#include <vizkit3d_world/Vizkit3dWorld.hpp>

namespace vizkit3d_world {

void CustomEventReceiver::customEvent(QEvent *evt)
{
    world->customEvent(evt);
}

Vizkit3dWorld::Vizkit3dWorld(std::string path, std::vector<std::string> modelPaths, bool showGui)
    : worldPath(path), modelPaths(modelPaths), showGui(showGui), widget(NULL), running(false)
{

    loadGazeboModelPaths(modelPaths);
}

Vizkit3dWorld::~Vizkit3dWorld()
{
    deinitialize();
}

void Vizkit3dWorld::initialize() {

    if (running) {
        return;
    }

    guiThread = boost::thread( boost::bind( &Vizkit3dWorld::run, this ) );
    boost::mutex::scoped_lock lock(mut);
    cond.wait(lock);
}

void Vizkit3dWorld::deinitialize() {

    if (running){
        //close all openned windows
        qApp->closeAllWindows();
        //the correct way is to use guiThread.join()
        //but if showGui is "false" then the QApplication::exec is frozen
        //to solve this problem join the thread for 500
        guiThread.try_join_for(boost::chrono::milliseconds(500));
    }
}

void Vizkit3dWorld::wait(){
    /**
     * wait until Qt event loop thread stop
     */
    boost::mutex::scoped_lock lock(mut);
    cond.wait(lock);
}

/**
 * Thread procedure with Qt event loop thread
 */
void Vizkit3dWorld::run() {

    int argc = 1;
    char *argv[] = { "vizkit3d_world" };

    QApplication app(argc, argv);

    //intercept the custom events
    customEventReceiver = new CustomEventReceiver(this);

    //main widget to store the plugins and performs the GUI events
    widget = new vizkit3d::Vizkit3DWidget();

    //remove the close button from window title
    widget->setWindowFlags(widget->windowFlags() & ~Qt::WindowCloseButtonHint);

    if (showGui)  widget->show();

    //load the world sdf file and created the vizkit3d::RobotVisualization models
    //It is necessary to create the vizkit3d plugins in the same thread of QApplication
    loadFromFile(worldPath);
    attachPlugins();
    //apply the tranformations in each model
    applyTransformations();

    running = true;
    cond.notify_one();

    app.exec();
    app.exit(0);

    running = false;

    cond.notify_one();

    delete widget;
    widget = NULL;
    toSdfElement.clear();
    robotVizMap.clear();
}

void Vizkit3dWorld::loadFromFile(std::string path) {
    std::ifstream file(path.c_str());
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

    makeWorld(sdf->root->GetElement("world"), sdf->version);
}

void Vizkit3dWorld::loadGazeboModelPaths(std::vector<std::string> modelPaths) {

    for (std::vector<std::string>::iterator it = modelPaths.begin(); it != modelPaths.end(); it++){
        sdf::addURIPath("model://", *it);
    }

    std::string home = std::string(getenv("HOME"));

    sdf::addURIPath("model://", home + "/.gazebo/models");

    std::string path = getenv("GAZEBO_MODEL_PATH") + std::string(":") + getenv("PATH");

    std::vector<std::string> vec;
    boost::algorithm::split(vec, path, boost::algorithm::is_any_of(":"), boost::algorithm::token_compress_on);

    for (std::vector<std::string>::iterator it = vec.begin(); it != vec.end(); it++){
        if (!(*it).empty()){
            sdf::addURIPath("model://", *it);
        }
    }
}

void Vizkit3dWorld::makeWorld(sdf::ElementPtr sdf, std::string version) {

    if (sdf->HasElement("model")) {
        worldName = sdf->Get<std::string>("name");

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

            vizkit3d::RobotVisualization* robotViz = robotVizFromSdfModel(modelElem, modelName, version);
            robotVizMap.insert(std::make_pair(modelName, robotViz));

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
        widget->addPlugin(it->second);
        it->second->setParent(widget);
        //it is necessary to add to widget first and set the parent widget
        it->second->setVisualizationFrame(it->first.c_str());
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

void Vizkit3dWorld::applyTransformations() {

    for (RobotVizMap::iterator it = robotVizMap.begin();
            it != robotVizMap.end(); it++){

        sdf::ElementPtr sdfModel = getSdfElement(it->first);

        sdf::Pose pose =  sdfModel->GetElement("pose")->Get<sdf::Pose>();

        applyTransformation(worldName, it->first,
                            QVector3D(pose.pos.x, pose.pos.y, pose.pos.z),
                            QQuaternion(pose.rot.w, pose.rot.x, pose.rot.y, pose.rot.z));

    }
}

void Vizkit3dWorld::applyTransformation(base::samples::RigidBodyState rbs) {
    applyTransformation(rbs.targetFrame,
                        rbs.sourceFrame,
                        rbs.position,
                        rbs.orientation);
}

void Vizkit3dWorld::applyTransformation(std::string targetFrame, std::string sourceFrame, base::Position position, base::Orientation orientation) {
    applyTransformation(targetFrame ,sourceFrame,
                        QVector3D(position.x(), position.y(), position.z()),
                        QQuaternion(orientation.w(), orientation.x(), orientation.y(), orientation.z()));
}

void Vizkit3dWorld::applyTransformation(std::string targetFrame, std::string sourceFrame, QVector3D position, QQuaternion orientation) {

    if (widget) {
        widget->setTransformation(targetFrame.c_str(),
                                  sourceFrame.c_str(),
                                  position,
                                  orientation);
    }
}

void Vizkit3dWorld::customEvent(QEvent *e){
    //intercept custom events
    if (e->type() == TransformationEvent::ID){
        TransformationEvent *te = (TransformationEvent*)e;
        applyTransformation(te->pose);
    }
}

void Vizkit3dWorld::setTransformation(base::samples::RigidBodyState rbs) {
    //the function setTransformation must be called from Qt event loop thread
    //if setTransformation was called in another thread the application breaks
    QEvent *evt = new TransformationEvent(rbs);
    QCoreApplication::postEvent(customEventReceiver, evt);
}

}
