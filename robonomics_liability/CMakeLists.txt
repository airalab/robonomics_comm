cmake_minimum_required(VERSION 2.8.3)
project(robonomics_liability)

find_package(catkin REQUIRED
    COMPONENTS
        rospy
        message_generation
        std_srvs
)

catkin_python_setup()

add_message_files(
    FILES
        Liability.msg
)

add_service_files(
    FILES
        SetLiabilityResult.srv
)

generate_messages()

catkin_install_python(
    PROGRAMS
        scripts/listener_node
        scripts/executor_node
    DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION}
)
