cmake_minimum_required(VERSION 2.8.3)
project(robonomics_liability)

find_package(catkin REQUIRED
    COMPONENTS
        rospy
        message_generation
)

catkin_python_setup()

add_message_files(
    FILES
        Liability.msg
)

generate_messages()

catkin_package()

catkin_install_python(
    PROGRAMS
        scripts/listener_node
        scripts/executor_node
    DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    FILES
        launch/liability.launch
    DESTINATION
        ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(
    FILES
        abi/liability.json
    DESTINATION
        ${CATKIN_PACKAGE_SHARE_DESTINATION}/abi
)
