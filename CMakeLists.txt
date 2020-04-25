cmake_minimum_required(VERSION 2.8.3)
project(cslibs_gridmaps)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

include(cmake/cslibs_gridmaps_enable_c++17.cmake)

if(NOT ${PROJECT_NAME}_BUILD_EXAMPLES)
    set(${PROJECT_NAME}_BUILD_EXAMPLES False)
endif()

find_package(catkin REQUIRED COMPONENTS
  cslibs_indexed_storage
  cslibs_math_2d
  cslibs_utility
  cslibs_time
)

find_package(yaml-cpp REQUIRED)

catkin_package(
    INCLUDE_DIRS
        include
    CATKIN_DEPENDS
        cslibs_math_2d
        cslibs_indexed_storage
        cslibs_utility
    CFG_EXTRAS
        cslibs_gridmaps_enable_c++17.cmake
    LIBRARIES
)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME}
    INTERFACE
        include/
        ${catkin_INCLUDE_DIRS}
)

if(NOT ${CMAKE_BUILD_TYPE} STREQUAL Debug)
    set(TARGET_COMPILE_OPTIONS
        -Wall -Werror -Ofast -ffast-math
    )
    # enabling -march=native my result in problems on some machines,
    # my try -mtune=native

    message(STATUS "[${PROJECT_NAME}]: Compiling with optimization!")
endif()

if(${PROJECT_NAME}_BUILD_EXAMPLES)
    find_package(OpenCV      REQUIRED)
    find_package(cslibs_time REQUIRED)
    find_package(tf          REQUIRED)
    find_package(nav_msgs    REQUIRED)




    set(TARGET_INCLUDE_DIRS
        include/
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${cslibs_time_INCLUDE_DIRS}
        ${tf_INCLUDE_DIRS}
        ${nav_msgs_INCLUDE_DIRS}

    )
    set(TARGET_LINK_LIBRARIES
        ${catkin_LIBRARIES}
        ${tf_LIBRARIES}
        ${nav_msgs_LIBRARIES}
        ${OpenCV_LIBRARIES}
    )

    ### Range Example ###
    add_executable(${PROJECT_NAME}_example_get_range
        test/example_get_range.cpp
    )

    target_include_directories(${PROJECT_NAME}_example_get_range
        PRIVATE
            ${TARGET_INCLUDE_DIRS}
    )

    target_link_libraries(${PROJECT_NAME}_example_get_range
        PRIVATE
            ${TARGET_LINK_LIBRARIES}
    )

    target_compile_options(${PROJECT_NAME}_example_get_range
        PRIVATE
            ${TARGET_COMPILE_OPTIONS}
    )

    ### Iterator Example ###
    add_executable(${PROJECT_NAME}_example_line_iterator
        test/example_line_iterator.cpp
    )

    target_link_libraries(${PROJECT_NAME}_example_line_iterator
        PRIVATE
            ${TARGET_LINK_LIBRARIES}
    )

    target_include_directories(${PROJECT_NAME}_example_line_iterator
        PRIVATE
            ${TARGET_INCLUDE_DIRS}
    )

    target_compile_options(${PROJECT_NAME}_example_line_iterator
        PRIVATE
            ${TARGET_COMPILE_OPTIONS}
    )

    ### Distance Transform Example
    add_executable(${PROJECT_NAME}_example_distance_transform
        test/example_distance_transform.cpp
    )

    target_include_directories(${PROJECT_NAME}_example_distance_transform
        PRIVATE
            ${TARGET_INCLUDE_DIRS}
    )

    target_link_libraries(${PROJECT_NAME}_example_distance_transform
        PRIVATE
            ${catkin_LIBRARIES}
            ${OpenCV_LIBRARIES}
    )

    target_compile_options(${PROJECT_NAME}_example_distance_transform
        PRIVATE
            ${TARGET_COMPILE_OPTIONS}
    )

endif()

cslibs_gridmaps_add_unit_test_gtest(serialization
    INCLUDE_DIRS
        include/
        ${catkin_INCLUDE_DIRS}
        ${YAML_CPP_INCLUDE_DIR}
    SOURCE_FILES
        test/serialization.cpp
    LINK_LIBRARIES
        ${YAML_CPP_LIBRARIES}
    COMPILE_OPTIONS
        ${TARGET_COMPILE_OPTIONS}
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
