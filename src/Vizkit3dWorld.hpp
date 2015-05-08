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
#include <kdl_parser/kdl_parser.hpp>
#include <vizkit3d/Vizkit3DWidget.hpp>
#include <vizkit3d/RobotVisualization.hpp>
#include <vizkit3d/QtThreadedWidget.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <map>

namespace vizkit3d_world {

typedef std::map<std::string, vizkit3d::RobotVisualization*> RobotVizMap;

class Vizkit3dWorld;

/**
 * This event is used to call setTransformation in the event loop thread
 */
class TransformationEvent : public QEvent {

public:

    static const int ID = 2000;

    TransformationEvent(base::samples::RigidBodyState pose) :
        QEvent(QEvent::Type(ID)),
        pose(pose)  {}

    base::samples::RigidBodyState pose;
};

/**
 * Receive a Custom Event
 * The custom event is used to send commands to event loop thread
 */
class CustomEventReceiver : public QObject
{
    Q_OBJECT

public:

    CustomEventReceiver(Vizkit3dWorld *world) : world(world)
    {
    }

protected:

    /**
     * Intercept Qt custom events
     */
    void customEvent(QEvent *evt);
    Vizkit3dWorld *world;
};

/**
 * Vizkit3dWorld
 * set up vizkit3d instance from SDF
 */
class Vizkit3dWorld : public QObject {
    Q_OBJECT

public:

    /**
     * Vizkit3dWorld constructor
     *
     * @param path the string with the path to the sdf world file
     * @param modelPaths list with paths to models
     * @param showGui show or not the user interface
     */
    Vizkit3dWorld(std::string path = std::string(""),
                  std::vector<std::string> modelPaths = std::vector<std::string>(),
                  bool showGui = false);

    /**
     * Vizkit3dWorld destructor
     */
    virtual ~Vizkit3dWorld();

    /**
     * Initialize the vizkit3d world and start the Qt event loop thread
     */
    void initialize();

    /**
     * Stop the event loop thread
     */
    void deinitialize();

    /**
     * Wait until the event loop thread stopped
     */
    void wait();

    RobotVizMap getRobotVizMap();

    /**
     * set joints states
     *
     * @param modelName the model name
     * @param joints the vector with joints states
     */
    void setJoints(std::string modelName, base::samples::Joints joints);

    /**
     * set models transformations using vizkit3d setTransformation
     *
     * @param pose the pose with transformation. The transformation is
     * relative to the target frame and the source frame
     */
    void setTransformation(base::samples::RigidBodyState pose);

protected:

    /**
     * Thread procedure
     * This functions has the Qt event loop thread
     */
    void run();

    /**
     * Load world sdf from file
     */
    void loadFromFile(std::string filename);

    /**
     * Load world sdf from string
     */
    void loadFromString(const std::string xml);

    /**
     * Add gazebo models paths
     * @params list with paths to models
     */
    void loadGazeboModelPaths(std::vector<std::string> modelPaths = std::vector<std::string>());

    /**
     * attach vizkit3d::RobotVisualization plugins to Vizkit3dWidget
     */
    void attachPlugins();


    /**
     * Return the RobotVisualization by name
     *
     * @param model name
     * @return vizkit3d::RobotVisualization with the model
     */
    vizkit3d::RobotVisualization* getRobotViz(std::string name);

    /**
     * Create RobotVisualization using sdf model
     *
     * @param sdf_model the model structure with the model definition
     * @param modelName the model model name
     * @param version the version of sdf file
     * @return vizkit3d::RobotVisualization created from sdf model
     */
    vizkit3d::RobotVisualization* robotVizFromSdfModel(sdf::ElementPtr sdf_model, std::string modelName, std::string version);

    /**
     * Create the scene contains every models defined in the sdf world file
     */
    void makeWorld(sdf::ElementPtr sdf, std::string version);

    /**
     * Get sdf model element by the model name
     *
     * @param name the model name
     * @return sdf::ElementPtr with the model sdf element
     */
    sdf::ElementPtr getSdfElement(std::string name);

    /**
     * Apply transformation in the each model of the scene
     */
    void applyTransformations();

    /**
     * Apply transformation in a model
     *
     * @param pose with transformation
     */
    void applyTransformation(base::samples::RigidBodyState pose);

    /**
     * Apply transformation using rock types
     *
     * @param sourceFrame the source frame
     * @param targetFrame the target frame
     * @param position the position that will applied in the transformation
     * @param orientation the orientation that will applied in the transformation
     *
     */
    void applyTransformation(std::string sourceFrame, std::string targetFrame, base::Position position, base::Orientation orientation);

    /**
     * Apply transformation using Qt types
     *
     * @param sourceFrame the source frame
     * @param targetFrame the target frame
     * @param position the position that will applied in the transformation
     * @param orientation the orientation that will applied in the transformation
     *
     */
    void applyTransformation(std::string sourceFrame, std::string targetFrame, QVector3D position, QQuaternion orientation);

    /**
     * Intercept the Qt custom events
     */
    void customEvent(QEvent *event);

    std::string worldPath; //path to sdf file that describe the scene
    std::string worldName; //stores the world name

    bool showGui; //used to show or not the user interface

    RobotVizMap robotVizMap; //stores the vizkit3d::RobotVisualization and uses the model name as key
    vizkit3d::Vizkit3DWidget *widget; //this widget stores and manage the robot models plugins

    std::vector<std::string> modelPaths; //stores paths with gazebo models

    std::map<std::string, sdf::ElementPtr> toSdfElement; //map sdf element using model name

    boost::mutex mut; //mutex used to sincronize the event loop thread and other threads
    boost::condition_variable cond; //condition used to sincronize the event loop thread and other threads
    boost::thread guiThread; //event loop thread

    bool running; //check if event loop is running

    CustomEventReceiver *customEventReceiver;
    friend class CustomEventReceiver;
};

}

#endif /* GUI_VIZKIT3D_WORLD_SRC_VIZKIT3DWORLD_HPP_ */
