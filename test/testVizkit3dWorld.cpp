#include <boost/test/unit_test.hpp>
#include <vizkit3d_world/Vizkit3dWorld.hpp>
#include <QString>

using namespace vizkit3d_world;

BOOST_AUTO_TEST_CASE(it_loads_correctly_a_sdf_world)
{
    vizkit3d_world::Vizkit3dWorld vizkit3d_world("test_data/simple.world");
    RobotVizMap robot_viz = vizkit3d_world.getRobotVizMap();
    BOOST_TEST(robot_viz.size(),1);
    BOOST_CHECK_EQUAL(robot_viz.begin()->first,"simple_model");
}
