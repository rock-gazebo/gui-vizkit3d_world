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
#include <base/samples/frame.h>
#include <map>

#include "Events.hpp"

namespace vizkit3d_world {

typedef std::map<std::string, vizkit3d::RobotVisualization*> RobotVizMap;

/**
 * Event listener used to intercept the gui creation and destruction inside the event loop thread
 * This listener is useful, when we need to create and destroy Qt objects (e.g: vizkit3d plugins) inside the same thread of the main thread
 */
class EventListener {
public:
    virtual void onCreateWorld() = 0;
    virtual void onDestroyWorld() = 0;
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
     * @param path: the string with the path to the sdf world file
     * @param modelPaths: list with paths to models
     * @param showGui: show or not the user interface
     */
    Vizkit3dWorld(std::string path = std::string(""),
                  std::vector<std::string> modelPaths = std::vector<std::string>(),
                  bool showGui = false, int cameraWidth = 800, int cameraHeight = 600,
                  double horizontalFov = 60.0,
                  double zNear = 0.01, double zFar = 1000.0);

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


    /***
     * set camera position
     *
     * @param pose: the camera position
     */
    void setCameraPose(base::samples::RigidBodyState pose);


    /**
     * send a event signal to qt event loop to enable grabbing
     */
    void postEnableGrabbing();


    /**
     * send a event signal to qt event loop to disable grabbing
     */
    void postDisableGrabbing();

    /**
     * @return vizkit3d::Vizkit3DWidget: render the scene
     */
    vizkit3d::Vizkit3DWidget *getWidget() { return widget; }


    /**
     * send event signal to qt event loop to grab render image
     */
    void postGrabImage();


    /**
     * this function notifies the event loop to process events
     * the event loop thread is blocked until receive a signal to process the events
     */
    void notifyEvents();


    /**
     * returns if the qt event loop thread is running
     *
     * @return bool: true if thread is running
     */
    bool isRunning() { return running; }


    /**
     * grab image from vizkit3d
     *
     * @return QImage: returns a image rendered by vizkit3d
     */
    QImage grabImage();

    /**
     * grab image from vizkit3d
     *
     * @return base::samples::frame::Frame* : returns a frame rendered by vizkit3d
     */
    void grabFrame(base::samples::frame::Frame& frame);


     void setCameraParams(int cameraWidth, int cameraHeight, double horizontalFov, double zNear, double zFar);

     /**
      * set the event listener
      */
     void setEventListener(EventListener *listener);

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
     * @param sourceFrame: the source frame
     * @param targetFrame: the target frame
     * @param position: the position that will applied in the transformation
     * @param orientation: the orientation that will applied in the transformation
     *
     */
    void applyTransformation(std::string sourceFrame, std::string targetFrame, base::Position position, base::Orientation orientation);

    /**
     * Apply transformation using Qt types
     *
     * @param sourceFrame: the source frame
     * @param targetFrame: the target frame
     * @param position: the position that will applied in the transformation
     * @param orientation: the orientation that will applied in the transformation
     *
     */
    void applyTransformation(std::string sourceFrame, std::string targetFrame, QVector3D position, QQuaternion orientation);


    void applyCameraParams();


    /**
     * internal function used to enable or disable grabbing
     *
     * @param value: enable grabbing if true, otherwise, disable grabbing
     */
    void enableGrabbing(bool value);

    QImage grabbedImage; //image grabbed

    /**
     * Intercept the Qt custom events
     */
    void customEvent(QEvent *event);

    std::string worldPath; //path to sdf file that describe the scene
    std::string worldName; //stores the world name

    bool showGui; //used to show or not the user interfaces

    RobotVizMap robotVizMap; //stores the vizkit3d::RobotVisualization and uses the model name as key
    vizkit3d::Vizkit3DWidget *widget; //this widget stores and manage the robot models plugins

    std::vector<std::string> modelPaths; //stores paths with gazebo models

    std::map<std::string, sdf::ElementPtr> toSdfElement; //map sdf element using model name

    /**
     * synchronize the qt event loop thread and another threads
     */
    boost::mutex mutex;
    boost::condition_variable cond;

    boost::thread guiThread; //event loop thread

    bool appQuit; //stop event loop thread if is true
    bool running; //check if event loop is running

    /**
     * Used to receive the custom events and sends to Vizkit3dWorld
     */
    events::CustomEventReceiver *customEventReceiver;
    friend class events::CustomEventReceiver;

    /**
     * Used in qt event loop to manager the Qt windows and events
     */
    QApplication *app;

    /**
     * Camera parameters
     */
    int cameraWidth;
    int cameraHeight;
    double zNear;
    double zFar;
    double horizontalFov;

    /**
     * interface used to intercept the the GUI creation and destruction in the event loop thread
     */
    EventListener *listener;

};

}

#endif /* GUI_VIZKIT3D_WORLD_SRC_VIZKIT3DWORLD_HPP_ */
