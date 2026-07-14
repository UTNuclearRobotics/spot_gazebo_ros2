cmake_minimum_required(VERSION 3.5)

project(spot_description_sim)

find_package(ament_cmake REQUIRED)
find_program(XACRO_EXECUTABLE xacro REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ---------------------------------------------------------------------------
# Generate models/spot/model.urdf and models/spot/model.sdf from xacro.
# The install destinations are identical to the paths the launch files and
# world <include> URIs already use, so nothing downstream changes.
#
# Note: $(find spot_description) inside spot.urdf.xacro requires
# spot_description to be built and in the ament environment; colcon provides
# this via the <build_depend> in package.xml.
# ---------------------------------------------------------------------------
set(GENERATED_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
file(MAKE_DIRECTORY ${GENERATED_DIR})

file(GLOB_RECURSE XACRO_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/urdf/*.xacro
  ${CMAKE_CURRENT_SOURCE_DIR}/sdf/*.xacro
)

add_custom_command(
  OUTPUT ${GENERATED_DIR}/model.urdf
  COMMAND ${XACRO_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/urdf/spot.urdf.xacro
          -o ${GENERATED_DIR}/model.urdf
  DEPENDS ${XACRO_SOURCES}
  COMMENT "Generating spot model.urdf from urdf/spot.urdf.xacro"
  VERBATIM
)

add_custom_command(
  OUTPUT ${GENERATED_DIR}/model.sdf
  COMMAND ${XACRO_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/sdf/model.sdf.xacro
          -o ${GENERATED_DIR}/model.sdf
  DEPENDS ${XACRO_SOURCES}
  COMMENT "Generating spot model.sdf from sdf/model.sdf.xacro"
  VERBATIM
)

add_custom_target(generate_spot_model ALL
  DEPENDS ${GENERATED_DIR}/model.urdf ${GENERATED_DIR}/model.sdf
)

# Static content: meshes + model.config (model.urdf/model.sdf are no longer
# checked in; delete them from models/spot/ in the source tree).
install(
  DIRECTORY
    models/
  DESTINATION share/${PROJECT_NAME}/models
)

# Generated artifacts land exactly where the hand-written files used to be.
install(
  FILES
    ${GENERATED_DIR}/model.urdf
    ${GENERATED_DIR}/model.sdf
  DESTINATION share/${PROJECT_NAME}/models/spot
)

# Ship the xacro sources so other packages can compose against them.
install(
  DIRECTORY
    urdf
    sdf
    scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

ament_package()
