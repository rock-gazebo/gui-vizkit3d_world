#ifndef GUI_VIZKIT3D_WORLD_SRC_EVENTS_HPP_
#define GUI_VIZKIT3D_WORLD_SRC_EVENTS_HPP_

#include <base/samples/RigidBodyState.hpp>
#include <QtCore/QEvent>
#include <QtCore/QObject>
#include <QtCore/QString>

namespace vizkit3d_world {

/**
 * Vizkit3dWorld forward class
 */
class Vizkit3dWorld;

namespace events {

enum CustomEventIdentifiers {
    CustomEventIdentifiersFirst = 2000,
    //events identifiers
    TransformationEventId, //event used to set transformation
    GrabbingEventId, //event used to enable or disable grabbing
    GrabEventId, //event used to grab render image
    //events identifiers
    CustomEventIdentifiersLast,
};

/**
 * This event is used to call setTransformation in the event loop thread
 */
class TransformationEvent : public QEvent {

public:

    TransformationEvent(base::samples::RigidBodyState pose) :
        QEvent(QEvent::Type(TransformationEventId)),
        pose(pose)  {}

    //event data
    base::samples::RigidBodyState pose;
};

/**
 * This event is used to enable the grabbing image
 * The function enableGrabbing is just called from event loop thread
 */
class GrabbingEvent : public QEvent {

public:

    GrabbingEvent(bool enableGrabbing) :
        QEvent(QEvent::Type(GrabbingEventId)),
        enableGrabbing(enableGrabbing) {}

    //enable or disable grabbing
    //It is used by vizkit3d to enable to grab render image by OpenSceneGraph
    bool enableGrabbing;
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

} //end events namespace

} //end vizkit3d_world namespace



#endif /* GUI_VIZKIT3D_WORLD_SRC_EVENTS_HPP_ */
