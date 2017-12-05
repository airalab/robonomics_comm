cmake_minimum_required(VERSION 2.8.3)
project(robonomics_comm)
find_package(catkin REQUIRED
    COMPONENTS
        robonomics_market
        robonomics_control
        robonomics_liability
    )
catkin_metapackage()
