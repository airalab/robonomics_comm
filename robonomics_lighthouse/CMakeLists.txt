cmake_minimum_required(VERSION 2.8.3)
project(robonomics_lighthouse)

find_package(catkin REQUIRED
    COMPONENTS
        rospy
        message_generation
        ethereum_common
)

catkin_python_setup()

add_message_files(
    FILES
        Ask.msg
        Bid.msg
        Result.msg
)

generate_messages()

catkin_package()

catkin_install_python(
    PROGRAMS
        scripts/infochan_node
        scripts/signer_node
        scripts/matcher_node
        scripts/reporter_node
    DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    FILES
        launch/infochan.launch
        launch/lighthouse.launch
    DESTINATION
        ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(
    FILES
        abi/factory.json
        abi/liability.json
        abi/lighthouse.json
    DESTINATION
        ${CATKIN_PACKAGE_SHARE_DESTINATION}/abi
)
