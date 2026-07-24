cmake_minimum_required(VERSION 3.0.2)
project(chassis_interface)

# C++14 for make_unique, shared_timed_mutex
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pluginlib
  nav_msgs
  geometry_msgs
  tf
  diagnostic_updater
  std_srvs
)

find_package(Boost REQUIRED COMPONENTS system thread)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES chassis_interface
  CATKIN_DEPENDS
    roscpp
    pluginlib
    nav_msgs
    geometry_msgs
    diagnostic_updater
    std_srvs
  DEPENDS Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# ============================================================
# 共享库 — 基类实现 + 协议层 + 所有驱动插件
# ============================================================
add_library(chassis_interface
  src/chassis_driver.cpp
  src/drivers/lepu/lepu_driver.cpp
  src/drivers/lepu/lepu_protocol.cpp
  src/drivers/template_driver.cpp
)
target_link_libraries(chassis_interface
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# ============================================================
# 可执行文件 — 桥接节点
# ============================================================
add_executable(chassis_bridge_node
  src/chassis_bridge.cpp
  src/chassis_bridge_node.cpp
)
target_link_libraries(chassis_bridge_node
  chassis_interface
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(lepu_serial_link_test
    test/lepu_serial_link_test.cpp
  )
  if(TARGET lepu_serial_link_test)
    target_link_libraries(lepu_serial_link_test
      chassis_interface
      ${catkin_LIBRARIES}
      util
    )
  endif()
endif()

# ============================================================
# Install
# ============================================================
install(TARGETS chassis_interface chassis_bridge_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES plugins/chassis_interface_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/plugins
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
