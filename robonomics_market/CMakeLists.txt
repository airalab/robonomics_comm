cmake_minimum_required(VERSION 2.8.3)
project(robonomics_market)

find_package(catkin REQUIRED
    COMPONENTS
        rospy
        message_generation
)

catkin_python_setup()

add_message_files(
    FILES
        Ask.msg
        Bid.msg
)

generate_messages()

catkin_install_python(
    PROGRAMS
        scripts/market_node
        scripts/distribution_node
    DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION}
)
