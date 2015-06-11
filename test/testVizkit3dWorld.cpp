#include <boost/test/unit_test.hpp>
#include <vizkit3d_world/Vizkit3dWorld.hpp>
#include <QString>

using namespace vizkit3d_world;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    vizkit3d_world::Vizkit3dWorld vizkit3d_world;
}
