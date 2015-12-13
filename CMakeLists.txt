# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(ardrone_thinc)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs message_runtime std_msgs uga_tum_ardrone message_generation ardrone_autonomy)

add_service_files(
  FILES
  Waypoint.srv
)

add_message_files(FILES pose.msg)


## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES 
    std_msgs
)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
catkin_package(
    DEPENDS  # TODO
    CATKIN_DEPENDS roscpp sensor_msgs message_runtime std_msgs uga_tum_ardrone message_generation ardrone_autonomy
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

include_directories( ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})


add_executable(thinc_smart src/SmartMain.cpp src/ArdroneThinc.cpp)
target_link_libraries(thinc_smart ${catkin_LIBRARIES})
add_dependencies(thinc_smart ${catkin_EXPORTED_TARGETS})
add_dependencies(thinc_smart ${PROJECT_NAME}_generate_messages_cpp)

add_executable(thinc_sock src/SocketMain.cpp)
target_link_libraries(thinc_sock ${catkin_LIBRARIES})
add_dependencies(thinc_sock ${catkin_EXPORTED_TARGETS})
add_dependencies(thinc_sock ${PROJECT_NAME}_generate_messages_cpp)

add_executable(sock_stub src/SocketStub.cpp)
target_link_libraries(sock_stub ${catkin_LIBRARIES})
add_dependencies(sock_stub ${catkin_EXPORTED_TARGETS})
add_dependencies(sock_stub ${PROJECT_NAME}_generate_messages_cpp)

install(PROGRAMS scripts/drone_teleop.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
