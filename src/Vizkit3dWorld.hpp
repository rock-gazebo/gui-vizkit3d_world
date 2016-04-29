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
#include <base/samples/Frame.hpp>
#include <map>

namespace vizkit3d_world {

typedef std::map<std::string, vizkit3d::RobotVisualization*> RobotVizMap;

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
     */
    Vizkit3dWorld(std::string path = std::string(""),
                  std::vector<std::string> modelPaths = std::vector<std::string>(),
                  std::vector<std::string> ignoredModels = std::vector<std::string>(),
                  int cameraWidth = 800, int cameraHeight = 600,
                  double horizontalFov = 60.0,
                  double zNear = 0.01, double zFar = 1000.0);

    /**
     * Vizkit3dWorld destructor
     */
    virtual ~Vizkit3dWorld();

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
     * Enable grabbing
     *
     * @param value: enable grabbing if true, otherwise, disable grabbing
     */
    void enableGrabbing();

    /**
     * Disable grabbing
     *
     * @param value: enable grabbing if true, otherwise, disable grabbing
     */
    void disableGrabbing();

    /**
     * @return vizkit3d::Vizkit3DWidget: render the scene
     */
    vizkit3d::Vizkit3DWidget *getWidget() { return widget; }

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


    QImage grabbedImage; //image grabbed

    std::string worldPath; //path to sdf file that describe the scene
    std::string worldName; //stores the world name

    RobotVizMap robotVizMap; //stores the vizkit3d::RobotVisualization and uses the model name as key
    vizkit3d::Vizkit3DWidget *widget; //this widget stores and manage the robot models plugins

    std::vector<std::string> modelPaths; //stores paths with gazebo models

    std::vector<std::string> ignoredModels; //list of sdf that will be ignored by the robot visualization

    std::map<std::string, sdf::ElementPtr> toSdfElement; //map sdf element using model name

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

};

}

#endif /* GUI_VIZKIT3D_WORLD_SRC_VIZKIT3DWORLD_HPP_ */
