cmake_minimum_required(VERSION 3.0.2)
project(lpms_imu_driver)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  message_generation
)

# ============================================================
# 核心共享库 — LPMS IG1 串口通信库
# ============================================================
set(CORE_SOURCES
  src/LpmsIG1.cpp
  src/LpMatrix.c
  src/LpUtil.cpp
  src/MicroMeasure.cpp
  src/SerialPortLinux.cpp
)

add_library(LpmsIG1_OpenSourceLib SHARED ${CORE_SOURCES})
target_include_directories(LpmsIG1_OpenSourceLib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include/lpsensor
)
target_link_libraries(LpmsIG1_OpenSourceLib pthread rt udev)

# ============================================================
# catkin 包声明
# ============================================================
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES LpmsIG1_OpenSourceLib
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs geometry_msgs
)

include_directories(
  include
  include/lpsensor
  ${catkin_INCLUDE_DIRS}
)

# ============================================================
# ROS 节点
# ============================================================
# lpms_ig1_node (USB)
add_executable(lpms_ig1_node src/lpms_ig1_node.cpp)
target_link_libraries(lpms_ig1_node ${catkin_LIBRARIES} LpmsIG1_OpenSourceLib)
add_dependencies(lpms_ig1_node ${catkin_EXPORTED_TARGETS})

# lpms_ig1_rs485_node (RS485)
add_executable(lpms_ig1_rs485_node src/lpms_ig1_rs485_node.cpp)
target_link_libraries(lpms_ig1_rs485_node ${catkin_LIBRARIES} LpmsIG1_OpenSourceLib)
add_dependencies(lpms_ig1_rs485_node ${catkin_EXPORTED_TARGETS})

# imudata_rad_to_deg_node
add_executable(imudata_rad_to_deg_node src/imudata_rad_to_deg_node.cpp)
target_link_libraries(imudata_rad_to_deg_node ${catkin_LIBRARIES})
add_dependencies(imudata_rad_to_deg_node ${catkin_EXPORTED_TARGETS})

# ============================================================
# Install
# ============================================================
install(TARGETS LpmsIG1_OpenSourceLib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(TARGETS lpms_ig1_node lpms_ig1_rs485_node imudata_rad_to_deg_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/lpsensor/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/lpsensor
)
install(DIRECTORY launch/ config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
