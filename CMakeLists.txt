cmake_minimum_required(VERSION 2.8.3)
project(cslibs_gridmaps)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

include(cmake/cslibs_gridmaps_enable_c++11.cmake)
include(cmake/cslibs_gridmaps_extras.cmake)
include(cmake/cslibs_gridmaps_show_headers.cmake)

if(NOT ${PROJECT_NAME}_BUILD_EXAMPLES)
    set(${PROJECT_NAME}_BUILD_EXAMPLES False)
endif()

find_package(catkin REQUIRED COMPONENTS
  cslibs_indexed_storage
  cslibs_math_2d
  cslibs_utility
)

catkin_package(
    INCLUDE_DIRS    include
    CATKIN_DEPENDS  cslibs_math_2d cslibs_indexed_storage cslibs_utility
    CFG_EXTRAS      cslibs_gridmaps_enable_c++11.cmake
                    cslibs_gridmaps_extras.cmake
                    cslibs_gridmaps_show_headers.cmake
                    cslibs_gridmaps_add_unit_test_gtest.cmake
    LIBRARIES       ${PROJECT_NAME}_static_maps
                    ${PROJECT_NAME}_dynamic_maps
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

cslibs_gridmaps_show_headers()

add_library(${PROJECT_NAME}_static_maps SHARED
    src/static_maps/binary_gridmap.cpp
    src/static_maps/distance_gridmap.cpp
    src/static_maps/probability_gridmap.cpp
    src/static_maps/likelihood_field_gridmap.cpp
)
target_link_libraries(${PROJECT_NAME}_static_maps
    ${catkin_LIBRARIES}
)

add_library(${PROJECT_NAME}_dynamic_maps SHARED
    src/dynamic_maps/probability_gridmap.cpp
    src/dynamic_maps/min_heightmap.cpp
    src/dynamic_maps/distribution_heightmap.cpp
)
target_link_libraries(${PROJECT_NAME}_dynamic_maps
    ${catkin_LIBRARIES}
)

if(${PROJECT_NAME}_BUILD_EXAMPLES)
    find_package(OpenCV      REQUIRED)
    find_package(cslibs_time REQUIRED)
    find_package(tf          REQUIRED)
    find_package(nav_msgs    REQUIRED)

    include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${cslibs_time_INCLUDE_DIRS}
        ${tf_INCLUDE_DIRS}
        ${nav_msgs_INCLUDE_DIRS}
    )

    add_executable(${PROJECT_NAME}_example_get_range
        test/example_get_range.cpp
    )

    target_link_libraries(${PROJECT_NAME}_example_get_range
        ${catkin_LIBRARIES}
        ${tf_LIBRARIES}
        ${nav_msgs_LIBRARIES}
        ${PROJECT_NAME}_static_maps
        ${OpenCV_LIBRARIES}
    )
    add_executable(${PROJECT_NAME}_example_line_iterator
        test/example_line_iterator.cpp
    )

    target_link_libraries(${PROJECT_NAME}_example_line_iterator
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}_static_maps
        ${tf_LIBRARIES}
        ${nav_msgs_LIBRARIES}
        ${OpenCV_LIBRARIES}
    )
    add_executable(${PROJECT_NAME}_example_distance_transform
        test/example_distance_transform.cpp
    )

    target_link_libraries(${PROJECT_NAME}_example_distance_transform
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
    )
endif()

cslibs_gridmaps_add_unit_test_gtest(serialization
    SRCS
    src/dynamic_maps/probability_gridmap.cpp
    src/static_maps/binary_gridmap.cpp
    src/static_maps/distance_gridmap.cpp
    src/static_maps/likelihood_field_gridmap.cpp
    src/static_maps/probability_gridmap.cpp
    test/serialization.cpp
)
target_link_libraries(serialization
    yaml-cpp
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}_static_maps
                ${PROJECT_NAME}_dynamic_maps
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

