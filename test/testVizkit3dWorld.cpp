#include <boost/test/unit_test.hpp>
#include <vizkit3d_world/Vizkit3dWorld.hpp>
#include <QString>
#include <string>

using namespace vizkit3d_world;

BOOST_AUTO_TEST_CASE(it_loads_correctly_a_sdf_world)
{
    vizkit3d_world::Vizkit3dWorld vizkit3d_world("test_data/simple.world");
    RobotVizMap robot_viz = vizkit3d_world.getRobotVizMap();
    BOOST_TEST(robot_viz.size(),1);
    BOOST_CHECK_EQUAL(robot_viz.begin()->first,"simple_model");
}

BOOST_AUTO_TEST_CASE(it_creates_two_widgets)
{
    vizkit3d_world::Vizkit3dWorld *vizkit3dWorld;
    vizkit3dWorld = new vizkit3d_world::Vizkit3dWorld(
        "test_data/simple.world",
        std::vector<std::string>(),
        std::vector<std::string>(),
        800, 600,
        60.0,
        0.01, 1000.0,
        2);
    BOOST_TEST(vizkit3dWorld->getWidget(0)->isWindow(),1);
    BOOST_TEST(vizkit3dWorld->getWidget(1)->isWindow(),1);
}
