#include "Vizkit3dWorld.hpp"
#include "Events.hpp"

namespace vizkit3d_world {

namespace events {

void CustomEventReceiver::customEvent(QEvent *evt)
{
    world->customEvent(evt);
}

} //end events namespace

} //end vizkit3d_world namespace





