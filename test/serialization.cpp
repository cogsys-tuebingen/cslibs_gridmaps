#include <gtest/gtest.h>

#include <cslibs_gridmaps/serialization/dynamic_maps/gridmap.hpp>
#include <cslibs_gridmaps/serialization/dynamic_maps/probability_gridmap.hpp>

#include <cslibs_gridmaps/serialization/static_maps/binary_gridmap.hpp>
#include <cslibs_gridmaps/serialization/static_maps/distance_gridmap.hpp>
#include <cslibs_gridmaps/serialization/static_maps/gridmap.hpp>
#include <cslibs_gridmaps/serialization/static_maps/likelihood_field_gridmap.hpp>
#include <cslibs_gridmaps/serialization/static_maps/probability_gridmap.hpp>

#include <cslibs_math/random/random.hpp>

const std::size_t MIN_NUM_SAMPLES = 100;
const std::size_t MAX_NUM_SAMPLES = 1000;

template <std::size_t Dim>
using rng_t = typename cslibs_math::random::Uniform<Dim>;

TEST(Test_cslibs_gridmaps, testDynamicGridmapSerialization)
{
    using map_t = cslibs_gridmaps::dynamic_maps::Gridmap<float>;
    rng_t<1> rng_prob(0.0, 1.0);
    rng_t<1> rng_coord(-100.0, 100.0);

    // fill map
    cslibs_math_2d::Transform2d origin(rng_coord.get(), rng_coord.get(), rng_t<1>(-M_PI, M_PI).get());
    const double resolution       = rng_t<1>(0.05, 1.0).get();
    const double chunk_resolution = rng_t<1>(5.0, 10.0).get();
    typename map_t::Ptr map(new map_t(origin, resolution, chunk_resolution, rng_t<1>(0.2, 0.8).get()));
    const int num_samples = static_cast<int>(rng_t<1>(MIN_NUM_SAMPLES, MAX_NUM_SAMPLES).get());
        for (int i = 0 ; i < num_samples ; ++ i) {
        const cslibs_math_2d::Point2d p(rng_coord.get(), rng_coord.get());
        map->set(p, rng_prob.get());
    }

    // serialization
    YAML::Node n(map);

    // de-serialization
    const typename map_t::Ptr & map_converted = n.as<typename map_t::Ptr>();
    EXPECT_NE(map_converted, nullptr);

    // tests
    EXPECT_EQ(map->getMinChunkIndex()[0], map_converted->getMinChunkIndex()[0]);
    EXPECT_EQ(map->getMinChunkIndex()[1], map_converted->getMinChunkIndex()[1]);
    EXPECT_EQ(map->getMaxChunkIndex()[0], map_converted->getMaxChunkIndex()[0]);
    EXPECT_EQ(map->getMaxChunkIndex()[1], map_converted->getMaxChunkIndex()[1]);
    EXPECT_EQ(map->getChunkSize(),        map_converted->getChunkSize());
    EXPECT_EQ(map->getHeight(),           map_converted->getHeight());
    EXPECT_EQ(map->getWidth(),            map_converted->getWidth());

    EXPECT_NEAR(map->getMin()(0),              map_converted->getMin()(0),              1e-9);
    EXPECT_NEAR(map->getMin()(1),              map_converted->getMin()(1),              1e-9);
    EXPECT_NEAR(map->getMax()(0),              map_converted->getMax()(0),              1e-9);
    EXPECT_NEAR(map->getMax()(1),              map_converted->getMax()(1),              1e-9);
    EXPECT_NEAR(map->getResolution(),          map_converted->getResolution(),          1e-9);
    EXPECT_NEAR(map->getOrigin().tx(),         map_converted->getOrigin().tx(),         1e-9);
    EXPECT_NEAR(map->getOrigin().ty(),         map_converted->getOrigin().ty(),         1e-9);
    EXPECT_NEAR(map->getOrigin().yaw(),        map_converted->getOrigin().yaw(),        1e-9);
    EXPECT_NEAR(map->getInitialOrigin().tx(),  map_converted->getInitialOrigin().tx(),  1e-9);
    EXPECT_NEAR(map->getInitialOrigin().ty(),  map_converted->getInitialOrigin().ty(),  1e-9);
    EXPECT_NEAR(map->getInitialOrigin().yaw(), map_converted->getInitialOrigin().yaw(), 1e-9);

    std::array<int, 2> min_chunk_index = map->getMinChunkIndex();
    std::array<int, 2> max_chunk_index = map->getMaxChunkIndex();

    for (int idx = min_chunk_index[0] ; idx <= max_chunk_index[0] ; ++ idx) {
        for (int idy = min_chunk_index[1] ; idy <= max_chunk_index[1] ; ++ idy) {
            std::array<int, 2> index({idx, idy});
            if (typename cslibs_gridmaps::dynamic_maps::Chunk<float>::handle_t c = map->getChunk(index)) {
                const typename cslibs_gridmaps::dynamic_maps::Chunk<float>::handle_t cc = map_converted->getChunk(index);
                EXPECT_NE(cc, nullptr);
                EXPECT_EQ(c->getData().size(), cc->getData().size());
                for (std::size_t i = 0 ; i < c->getData().size() ; ++ i)
                    EXPECT_NEAR(c->getData()[i], cc->getData()[i], 1e-3);
            } else
                EXPECT_EQ(map_converted->getChunk(index), nullptr);
        }
    }
}

TEST(Test_cslibs_gridmaps, testDynamicProbabilityGridmapSerialization)
{
    using map_t = cslibs_gridmaps::dynamic_maps::ProbabilityGridmap;
    rng_t<1> rng_prob(0.0, 1.0);
    rng_t<1> rng_coord(-100.0, 100.0);

    // fill map
    cslibs_math_2d::Transform2d origin(rng_coord.get(), rng_coord.get(), rng_t<1>(-M_PI, M_PI).get());
    typename map_t::Ptr map(new map_t(origin, rng_t<1>(0.05, 1.0).get(), rng_t<1>(5.0, 10.0).get()));
    const int num_samples = static_cast<int>(rng_t<1>(MIN_NUM_SAMPLES, MAX_NUM_SAMPLES).get());
    for (int i = 0 ; i < num_samples ; ++ i) {
        cslibs_math_2d::Point2d p(rng_coord.get(), rng_coord.get());
        map->set(p, rng_prob.get());
    }

    // serialization
    YAML::Node n(map);

    // de-serialization
    const typename map_t::Ptr & map_converted = n.as<typename map_t::Ptr>();

    // tests
    EXPECT_EQ(map->getMinChunkIndex()[0], map_converted->getMinChunkIndex()[0]);
    EXPECT_EQ(map->getMinChunkIndex()[1], map_converted->getMinChunkIndex()[1]);
    EXPECT_EQ(map->getMaxChunkIndex()[0], map_converted->getMaxChunkIndex()[0]);
    EXPECT_EQ(map->getMaxChunkIndex()[1], map_converted->getMaxChunkIndex()[1]);
    EXPECT_EQ(map->getChunkSize(),        map_converted->getChunkSize());
    EXPECT_EQ(map->getHeight(),           map_converted->getHeight());
    EXPECT_EQ(map->getWidth(),            map_converted->getWidth());

    EXPECT_NEAR(map->getMin()(0),              map_converted->getMin()(0),              1e-9);
    EXPECT_NEAR(map->getMin()(1),              map_converted->getMin()(1),              1e-9);
    EXPECT_NEAR(map->getMax()(0),              map_converted->getMax()(0),              1e-9);
    EXPECT_NEAR(map->getMax()(1),              map_converted->getMax()(1),              1e-9);
    EXPECT_NEAR(map->getResolution(),          map_converted->getResolution(),          1e-9);
    EXPECT_NEAR(map->getOrigin().tx(),         map_converted->getOrigin().tx(),         1e-9);
    EXPECT_NEAR(map->getOrigin().ty(),         map_converted->getOrigin().ty(),         1e-9);
    EXPECT_NEAR(map->getOrigin().yaw(),        map_converted->getOrigin().yaw(),        1e-9);
    EXPECT_NEAR(map->getInitialOrigin().tx(),  map_converted->getInitialOrigin().tx(),  1e-9);
    EXPECT_NEAR(map->getInitialOrigin().ty(),  map_converted->getInitialOrigin().ty(),  1e-9);
    EXPECT_NEAR(map->getInitialOrigin().yaw(), map_converted->getInitialOrigin().yaw(), 1e-9);

    std::array<int, 2> min_chunk_index = map->getMinChunkIndex();
    std::array<int, 2> max_chunk_index = map->getMaxChunkIndex();

    for (int idx = min_chunk_index[0] ; idx <= max_chunk_index[0] ; ++ idx) {
        for (int idy = min_chunk_index[1] ; idy <= max_chunk_index[1] ; ++ idy) {
            std::array<int, 2> index({idx, idy});
            if (typename cslibs_gridmaps::dynamic_maps::Chunk<double>::handle_t c = map->getChunk(index)) {
                const typename cslibs_gridmaps::dynamic_maps::Chunk<double>::handle_t cc = map_converted->getChunk(index);
                EXPECT_NE(cc, nullptr);
                EXPECT_EQ(c->getData().size(), cc->getData().size());
                for (std::size_t i = 0 ; i < c->getData().size() ; ++ i)
                    EXPECT_NEAR(c->getData()[i], cc->getData()[i], 1e-3);
            } else
                EXPECT_EQ(map_converted->getChunk(index), nullptr);
        }
    }
}

TEST(Test_cslibs_gridmaps, testStaticGridmapSerialization)
{
    using map_t = cslibs_gridmaps::static_maps::Gridmap<float>;
    rng_t<1> rng_prob(0.0, 1.0);
    rng_t<1> rng_coord(-100.0, 100.0);
    rng_t<1> rng_size(200, 500);

    // fill map
    cslibs_math_2d::Transform2d origin(rng_coord.get(), rng_coord.get(), rng_t<1>(-M_PI, M_PI).get());
    const std::size_t height = static_cast<std::size_t>(rng_size.get());
    const std::size_t width  = static_cast<std::size_t>(rng_size.get());
    typename map_t::Ptr map(new map_t(origin, rng_t<1>(0.05, 1.0).get(), height, width, -1.0));
    for (std::size_t i = 0 ; i < width ; ++ i)
        for (std::size_t j = 0 ; j < height ; ++ j)
            map->at(i, j) = rng_prob.get();

    // serialization
    YAML::Node n(map);

    // de-serialization
    const typename map_t::Ptr & map_converted = n.as<typename map_t::Ptr>();

    // tests
    EXPECT_EQ(map->getHeight(), map_converted->getHeight());
    EXPECT_EQ(map->getWidth(),  map_converted->getWidth());
    EXPECT_EQ(map->getHeight(), height);
    EXPECT_EQ(map->getWidth(),  width);

    EXPECT_NEAR(map->getMin()(0),       map_converted->getMin()(0),       1e-9);
    EXPECT_NEAR(map->getMin()(1),       map_converted->getMin()(1),       1e-9);
    EXPECT_NEAR(map->getMax()(0),       map_converted->getMax()(0),       1e-9);
    EXPECT_NEAR(map->getMax()(1),       map_converted->getMax()(1),       1e-9);
    EXPECT_NEAR(map->getResolution(),   map_converted->getResolution(),   1e-9);
    EXPECT_NEAR(map->getOrigin().tx(),  map_converted->getOrigin().tx(),  1e-9);
    EXPECT_NEAR(map->getOrigin().ty(),  map_converted->getOrigin().ty(),  1e-9);
    EXPECT_NEAR(map->getOrigin().yaw(), map_converted->getOrigin().yaw(), 1e-9);

    for (std::size_t i = 0 ; i < width ; ++ i)
        for (std::size_t j = 0 ; j < height ; ++ j)
            EXPECT_NEAR(map->at(i, j), map_converted->at(i, j), 1e-3);
}

TEST(Test_cslibs_gridmaps, testStaticBinaryGridmapSerialization)
{
    using map_t = cslibs_gridmaps::static_maps::BinaryGridmap;
    rng_t<1> rng_prob(0.0, 1.0);
    rng_t<1> rng_coord(-100.0, 100.0);
    rng_t<1> rng_size(200, 500);

    // fill map
    cslibs_math_2d::Transform2d origin(rng_coord.get(), rng_coord.get(), rng_t<1>(-M_PI, M_PI).get());
    const std::size_t height = static_cast<std::size_t>(rng_size.get());
    const std::size_t width  = static_cast<std::size_t>(rng_size.get());
    typename map_t::Ptr map(new map_t(origin, rng_t<1>(0.05, 1.0).get(), height, width));
    for (std::size_t i = 0 ; i < width ; ++ i)
        for (std::size_t j = 0 ; j < height ; ++ j)
            map->at(i, j) = static_cast<int>(std::round(rng_prob.get()));

    // serialization
    YAML::Node n(map);

    // de-serialization
    const typename map_t::Ptr & map_converted = n.as<typename map_t::Ptr>();

    // tests
    EXPECT_EQ(map->getHeight(), map_converted->getHeight());
    EXPECT_EQ(map->getWidth(),  map_converted->getWidth());
    EXPECT_EQ(map->getHeight(), height);
    EXPECT_EQ(map->getWidth(),  width);

    EXPECT_NEAR(map->getMin()(0),       map_converted->getMin()(0),       1e-9);
    EXPECT_NEAR(map->getMin()(1),       map_converted->getMin()(1),       1e-9);
    EXPECT_NEAR(map->getMax()(0),       map_converted->getMax()(0),       1e-9);
    EXPECT_NEAR(map->getMax()(1),       map_converted->getMax()(1),       1e-9);
    EXPECT_NEAR(map->getResolution(),   map_converted->getResolution(),   1e-9);
    EXPECT_NEAR(map->getOrigin().tx(),  map_converted->getOrigin().tx(),  1e-9);
    EXPECT_NEAR(map->getOrigin().ty(),  map_converted->getOrigin().ty(),  1e-9);
    EXPECT_NEAR(map->getOrigin().yaw(), map_converted->getOrigin().yaw(), 1e-9);

    for (std::size_t i = 0 ; i < width ; ++ i) {
        for (std::size_t j = 0 ; j < height ; ++ j) {
            EXPECT_EQ(map->at(i, j), map_converted->at(i, j));
            EXPECT_TRUE(map->at(i, j) == map_t::FREE || map->at(i, j) == map_t::OCCUPIED);
        }
    }
}

TEST(Test_cslibs_gridmaps, testStaticDistanceGridmapSerialization)
{    
    using map_t = cslibs_gridmaps::static_maps::DistanceGridmap;
    rng_t<1> rng_dist(0.0, 100.0);
    rng_t<1> rng_coord(-100.0, 100.0);
    rng_t<1> rng_size(200, 500);

    // fill map
    cslibs_math_2d::Transform2d origin(rng_coord.get(), rng_coord.get(), rng_t<1>(-M_PI, M_PI).get());
    const std::size_t height  = static_cast<std::size_t>(rng_size.get());
    const std::size_t width   = static_cast<std::size_t>(rng_size.get());
    const double max_distance = rng_t<1>(10.0, 30.0).get();
    typename map_t::Ptr map(new map_t(origin, rng_t<1>(0.05, 1.0).get(), max_distance, height, width));
    for (std::size_t i = 0 ; i < width ; ++ i)
        for (std::size_t j = 0 ; j < height ; ++ j)
            map->at(i, j) = rng_dist.get();

    // serialization
    YAML::Node n(map);

    // de-serialization
    const typename map_t::Ptr & map_converted = n.as<typename map_t::Ptr>();

    // tests
    EXPECT_EQ(map->getHeight(), map_converted->getHeight());
    EXPECT_EQ(map->getWidth(),  map_converted->getWidth());
    EXPECT_EQ(map->getHeight(), height);
    EXPECT_EQ(map->getWidth(),  width);

    EXPECT_NEAR(map->getMin()(0),       map_converted->getMin()(0),       1e-9);
    EXPECT_NEAR(map->getMin()(1),       map_converted->getMin()(1),       1e-9);
    EXPECT_NEAR(map->getMax()(0),       map_converted->getMax()(0),       1e-9);
    EXPECT_NEAR(map->getMax()(1),       map_converted->getMax()(1),       1e-9);
    EXPECT_NEAR(map->getResolution(),   map_converted->getResolution(),   1e-9);
    EXPECT_NEAR(map->getOrigin().tx(),  map_converted->getOrigin().tx(),  1e-9);
    EXPECT_NEAR(map->getOrigin().ty(),  map_converted->getOrigin().ty(),  1e-9);
    EXPECT_NEAR(map->getOrigin().yaw(), map_converted->getOrigin().yaw(), 1e-9);

    for (std::size_t i = 0 ; i < width ; ++ i)
        for (std::size_t j = 0 ; j < height ; ++ j)
            EXPECT_NEAR(map->at(i, j), map_converted->at(i, j), 1e-3);
}

TEST(Test_cslibs_gridmaps, testStaticLikelihoodFieldGridmapSerialization)
{    
    using map_t = cslibs_gridmaps::static_maps::LikelihoodFieldGridmap;
    rng_t<1> rng_prob(0.0, 1.0);
    rng_t<1> rng_coord(-100.0, 100.0);
    rng_t<1> rng_size(200, 500);

    // fill map
    cslibs_math_2d::Transform2d origin(rng_coord.get(), rng_coord.get(), rng_t<1>(-M_PI, M_PI).get());
    const std::size_t height  = static_cast<std::size_t>(rng_size.get());
    const std::size_t width   = static_cast<std::size_t>(rng_size.get());
    const double max_distance = rng_t<1>(10.0, 30.0).get();
    typename map_t::Ptr map(new map_t(origin, rng_t<1>(0.05, 1.0).get(), height, width, max_distance, rng_prob.get()));
    for (std::size_t i = 0 ; i < width ; ++ i)
        for (std::size_t j = 0 ; j < height ; ++ j)
            map->at(i, j) = rng_prob.get();

    // serialization
    YAML::Node n(map);

    // de-serialization
    const typename map_t::Ptr & map_converted = n.as<typename map_t::Ptr>();

    // tests
    EXPECT_EQ(map->getHeight(), map_converted->getHeight());
    EXPECT_EQ(map->getWidth(),  map_converted->getWidth());
    EXPECT_EQ(map->getHeight(), height);
    EXPECT_EQ(map->getWidth(),  width);

    EXPECT_NEAR(map->getMin()(0),       map_converted->getMin()(0),       1e-9);
    EXPECT_NEAR(map->getMin()(1),       map_converted->getMin()(1),       1e-9);
    EXPECT_NEAR(map->getMax()(0),       map_converted->getMax()(0),       1e-9);
    EXPECT_NEAR(map->getMax()(1),       map_converted->getMax()(1),       1e-9);
    EXPECT_NEAR(map->getResolution(),   map_converted->getResolution(),   1e-9);
    EXPECT_NEAR(map->getOrigin().tx(),  map_converted->getOrigin().tx(),  1e-9);
    EXPECT_NEAR(map->getOrigin().ty(),  map_converted->getOrigin().ty(),  1e-9);
    EXPECT_NEAR(map->getOrigin().yaw(), map_converted->getOrigin().yaw(), 1e-9);

    for (std::size_t i = 0 ; i < width ; ++ i)
        for (std::size_t j = 0 ; j < height ; ++ j)
            EXPECT_NEAR(map->at(i, j), map_converted->at(i, j), 1e-3);
}

TEST(Test_cslibs_gridmaps, testStaticProbabilityGridmapSerialization)
{
    using map_t = cslibs_gridmaps::static_maps::ProbabilityGridmap;
    rng_t<1> rng_prob(0.0, 1.0);
    rng_t<1> rng_coord(-100.0, 100.0);
    rng_t<1> rng_size(200, 500);

    // fill map
    cslibs_math_2d::Transform2d origin(rng_coord.get(), rng_coord.get(), rng_t<1>(-M_PI, M_PI).get());
    const std::size_t height  = static_cast<std::size_t>(rng_size.get());
    const std::size_t width   = static_cast<std::size_t>(rng_size.get());
    typename map_t::Ptr map(new map_t(origin, rng_t<1>(0.05, 1.0).get(), height, width));
    for (std::size_t i = 0 ; i < width ; ++ i)
        for (std::size_t j = 0 ; j < height ; ++ j)
            map->at(i, j) = rng_prob.get();

    // serialization
    YAML::Node n(map);

    // de-serialization
    const typename map_t::Ptr & map_converted = n.as<typename map_t::Ptr>();

    // tests
    EXPECT_EQ(map->getHeight(), map_converted->getHeight());
    EXPECT_EQ(map->getWidth(),  map_converted->getWidth());
    EXPECT_EQ(map->getHeight(), height);
    EXPECT_EQ(map->getWidth(),  width);

    EXPECT_NEAR(map->getMin()(0),       map_converted->getMin()(0),       1e-9);
    EXPECT_NEAR(map->getMin()(1),       map_converted->getMin()(1),       1e-9);
    EXPECT_NEAR(map->getMax()(0),       map_converted->getMax()(0),       1e-9);
    EXPECT_NEAR(map->getMax()(1),       map_converted->getMax()(1),       1e-9);
    EXPECT_NEAR(map->getResolution(),   map_converted->getResolution(),   1e-9);
    EXPECT_NEAR(map->getOrigin().tx(),  map_converted->getOrigin().tx(),  1e-9);
    EXPECT_NEAR(map->getOrigin().ty(),  map_converted->getOrigin().ty(),  1e-9);
    EXPECT_NEAR(map->getOrigin().yaw(), map_converted->getOrigin().yaw(), 1e-9);

    for (std::size_t i = 0 ; i < width ; ++ i)
        for (std::size_t j = 0 ; j < height ; ++ j)
            EXPECT_NEAR(map->at(i, j), map_converted->at(i, j), 1e-3);
}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
