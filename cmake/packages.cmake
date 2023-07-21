list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        geometry_msgs
        message_generation
        eigen_conversions
        )

add_message_files(
        FILES
        LanePoint.msg
        Lane.msg
        LaneList.msg
        )

generate_messages(
        DEPENDENCIES
        geometry_msgs
)
catkin_package()

include_directories(
        ${catkin_INCLUDE_DIRS}
)
