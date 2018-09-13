#include <boost/test/unit_test.hpp>
#include <vizkit3d_world/Vizkit3dWorld.hpp>
#include <QString>

using namespace vizkit3d_world;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    vizkit3d_world::Vizkit3dWorld vizkit3d_world;
}

BOOST_AUTO_TEST_CASE(it_starts_sdf)
{   
    std::string path = "test_data/simple.world";
    std::pair<std::string, int> sdf_string = getRobotModelString(
        path, kdl_parser::ROBOT_MODEL_FORMAT::ROBOT_MODEL_AUTO);
    sdf::SDFPtr sdf(new sdf::SDF);
    sdf::init(sdf);
}


//BOOST_AUTO_TEST_CASE(it_loads_correctly_a_sdf_world)
//{
//    vizkit3d_world::Vizkit3dWorld vizkit3d_world("test_data/simple.world");
//}