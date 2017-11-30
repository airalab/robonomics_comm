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

add_service_files(
    FILES
        AsksGenerator.srv
        BidsGenerator.srv
)

generate_messages()

catkin_package()

catkin_install_python(
    PROGRAMS
        scripts/market_node
        scripts/signer_node
        scripts/matcher_node
        scripts/generator_node
    DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    FILES
        launch/market.launch
        launch/matcher.launch
    DESTINATION
        ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(
    FILES
        abi/builder.json
    DESTINATION
        ${CATKIN_PACKAGE_SHARE_DESTINATION}/abi
)
