cmake_minimum_required(VERSION 2.8.3)
project(robonomics_control)

find_package(catkin REQUIRED
    COMPONENTS
        rospy
        message_generation
)

catkin_python_setup()

generate_messages()

catkin_package()

catkin_install_python(
    PROGRAMS
        scripts/distribution_node
    DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    DIRECTORY
        abi
        launch
        models
    DESTINATION
        ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
