cmake_minimum_required(VERSION 3.0.2)
project(rm_auto_aim_test_jig)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        rm_msgs
        rm_common
        urdf
        )


catkin_package(
        INCLUDE_DIRS
        include
        CATKIN_DEPENDS
        roscpp
        rm_msgs
        rm_common
        urdf
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
        src/main.cpp
        src/test_jig.cpp
        )

target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
 )
