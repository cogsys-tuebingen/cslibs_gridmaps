
#include <cslibs_math/random/random.hpp>

#include <cslibs_gridmaps/static_maps/conversion/convert_binary_gridmap.hpp>
#include <cslibs_gridmaps/static_maps/binary_gridmap.h>

#include <cslibs_time/time.hpp>
#include <cslibs_time/duration.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>

void mapOnly()
{

    nav_msgs::OccupancyGrid grid;
    grid.header.stamp = ros::Time::now();
    grid.info.height = 2500;
    grid.info.width = 2500;
    grid.info.resolution = 0.05;
    grid.data.resize(2500 * 2500, 0);
    grid.info.origin.orientation = tf::createQuaternionMsgFromYaw(0.0);

    cslibs_gridmaps::static_maps::BinaryGridmap<double>::Ptr binary;
    cslibs_gridmaps::static_maps::conversion::from<double>(grid, binary);

    cslibs_math_2d::Point2d start(62.5, 62.5);

    const double angle_incr = M_PI / 100.0;
    const double radius = 10.0;
    double angle = 0.0;
    double range = 0.0;
    cslibs_time::Time now = cslibs_time::Time::now();
    const std::size_t iterations = 100000;
    for(std::size_t i = 0 ; i < iterations ; ++i) {
        cslibs_math_2d::Point2d end = start + cslibs_math_2d::Vector2d(std::cos(angle) * radius,
                                                                             std::sin(angle) * radius);
        range = binary->getRange(start, end);
        angle += angle_incr;
    }
    std::cout << "took : " << (cslibs_time::Time::now() - now).milliseconds() / iterations << "\n";

    cslibs_math::random::Uniform<double, 1>uniform(-10.0, 10.0);
    now = cslibs_time::Time::now();
    double val = 0.0;
    for(std::size_t i = 9 ; i < iterations ; ++i) {
        val = tfSqrt(uniform.get());
    }
    std::cout << "took : " << (cslibs_time::Time::now() - now).milliseconds() / iterations << "\n";

    now = cslibs_time::Time::now();
    for(std::size_t i = 9 ; i < iterations ; ++i) {
        val = std::sqrt(uniform.get());
    }
    std::cout << "took : " << (cslibs_time::Time::now() - now).milliseconds() / iterations << "\n";

    cslibs_math_2d::Vector2d v1(0.0,0.0);
    cslibs_math_2d::Vector2d v2(1.0,2.0);
    now = cslibs_time::Time::now();
    double l = 0.0;
    for(std::size_t i = 0 ; i < iterations * iterations * iterations ; ++i) {
        l = (v1 - v2).length();
    }
    std::cout << "(v1 - v2).length() took :             " << (cslibs_time::Time::now() - now).milliseconds() << "\n";

    now = cslibs_time::Time::now();
    l = 0.0;
    for(std::size_t i = 0 ; i < iterations * iterations * iterations ; ++i) {
        l = std::sqrt((v1 - v2).length2());
    }
    std::cout << "std::sqrt((v1 - v2).length2()) took : " << (cslibs_time::Time::now() - now).milliseconds() << "\n";


    now = cslibs_time::Time::now();
    l = 0.0;
    for(std::size_t i = 0 ; i < iterations * iterations * iterations ; ++i) {
        l = (v1 - v2).length();
    }
    std::cout << "v1.distance(v2) took :                " << (cslibs_time::Time::now() - now).milliseconds() << "\n";


    tf::Vector3 v1_tf(0.0,0.0,0.0);
    tf::Vector3 v2_tf(1.0,2.0,0.0);
    now = cslibs_time::Time::now();
    l = 0.0;
    for(std::size_t i = 0 ; i < iterations * iterations * iterations ; ++i) {
        l = v1_tf.distance(v2_tf);
    }
    std::cout << "v1_tf.distance(v2_tf) took :          " << (cslibs_time::Time::now() - now).milliseconds() << "\n";
}

int main(int argc, char *argv[])
{
    mapOnly();

    return 0;
}
