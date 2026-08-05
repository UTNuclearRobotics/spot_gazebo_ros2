cmake_minimum_required(VERSION 3.5)

# Following directive defines the project name.
project(spot_gazebo)


# Following directives find required packages and load their configuration.
# The 'set' directive defines a variable (e.g. 'GZ_PLUGIN_VER').
# Such variables can be used lateron in the CMakeLists.txt file.
find_package(ament_cmake REQUIRED)
find_package(spot_description_sim REQUIRED)

# Gazebo Ignition Fortress (ign-gazebo6, pairs with ROS 2 Humble) —
# required for the custom system plugins below.
find_package(ignition-cmake2 REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)


# Following 'install' directive ensures that the 'worlds' subfolder
# will be copied to the 'share/spot_gazebo/worlds'
# subfolder of the installation directory.
install(
  DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

install(
  DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

# ===================== custom_gazebo_plugins =====================
# Custom gz-sim (Ignition Fortress) system plugins. Installed to
# lib/${PROJECT_NAME}/, which the hooks/*.dsv.in environment hook
# prepends to IGN_GAZEBO_SYSTEM_PLUGIN_PATH — so after
# 'source install/setup.bash', SDF can reference them by filename
# (e.g. filename="RfidPdReader") with no extra exports.

add_library(RfidPdReader SHARED
  custom_gazebo_plugins/RfidPdReader.cc
)
set_property(TARGET RfidPdReader PROPERTY CXX_STANDARD 17)
target_link_libraries(RfidPdReader
  ignition-gazebo6::ignition-gazebo6
  ignition-plugin1::ignition-plugin1
)

# Multi-port reader (AdvanReader-style): N antennas on arbitrary links,
# incl. articulated ones (arm end effector), TDM port multiplexing.
add_library(RfidReader SHARED
  custom_gazebo_plugins/RfidReader.cc
)
set_property(TARGET RfidReader PROPERTY CXX_STANDARD 17)
target_link_libraries(RfidReader
  ignition-gazebo6::ignition-gazebo6
  ignition-plugin1::ignition-plugin1
)

# Zebra Symbol LS2208 1-D laser barcode scanner. Coexists with the RFID
# readers above rather than replacing them: the RFID plugins stay buildable so
# server_rack_rfid.sdf and rfid_calibration.sdf keep working, and so the two
# sensing modalities can be compared. See the header comment in
# BarcodeScanner.cc for why its geometry gates look nothing like RfidReader's.
add_library(BarcodeScanner SHARED
  custom_gazebo_plugins/BarcodeScanner.cc
)
set_property(TARGET BarcodeScanner PROPERTY CXX_STANDARD 17)
target_link_libraries(BarcodeScanner
  ignition-gazebo6::ignition-gazebo6
  ignition-plugin1::ignition-plugin1
)

# Kinematic joint-state teleport at spawn (arm stow pose) — see the header
# comment in JointStateInitializer.cc for why the JTC initial_position
# parameter is not enough.
add_library(JointStateInitializer SHARED
  custom_gazebo_plugins/JointStateInitializer.cc
)
set_property(TARGET JointStateInitializer PROPERTY CXX_STANDARD 17)
target_link_libraries(JointStateInitializer
  ignition-gazebo6::ignition-gazebo6
  ignition-plugin1::ignition-plugin1
)

install(
  TARGETS RfidPdReader RfidReader BarcodeScanner JointStateInitializer
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

# Following directives are used when testing.
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Following hooks are used to ensure that the correct environment variables
# will be set by exectuting 'sourece install/setup.bash' after compilation.
# When using this template for your project, change the filenames of the
# files in the 'hooks' folder, to correspond to your project name.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

# Following directive configures ament based on the previous directives, and should
# typically be the last directive in the CMakeLists.txt file.
ament_package()