cmake_minimum_required(VERSION 3.8)
project(plan_bookstore)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_monitor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(easynav_interfaces QUIET)
if(easynav_interfaces_FOUND)
  add_compile_definitions(HAS_EASYNAV)
else()
  message(WARNING
    "easynav_interfaces NOT found - building Move BT plugin with fake_navigation "
    "fallback. Real navigation will be unavailable at runtime. Make sure "
    "easynav_interfaces is built before plan_bookstore (it is listed as a "
    "dependency in package.xml, so 'colcon build' from the workspace root "
    "should order it correctly).")
endif()
find_package(nav_msgs REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)
find_package(plansys2_bt_actions REQUIRED)
find_package(plansys2_monitor REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yolo_msgs QUIET)
find_package(cv_bridge QUIET)
find_package(OpenCV QUIET)


add_library(move_bt_node SHARED src/bt_nodes/Move.cpp)
list(APPEND plugin_libs move_bt_node)

add_library(approach_object_bt_node SHARED src/bt_nodes/ApproachObject.cpp)
list(APPEND plugin_libs approach_object_bt_node)

add_library(pick_book_bt_node SHARED src/bt_nodes/PickBook.cpp)
list(APPEND plugin_libs pick_book_bt_node)

add_library(place_book_bt_node SHARED src/bt_nodes/PlaceBook.cpp)
list(APPEND plugin_libs place_book_bt_node)

add_library(check_book_present_bt_node SHARED src/bt_nodes/CheckBookPresent.cpp)
list(APPEND plugin_libs check_book_present_bt_node)

foreach(bt_plugin ${plugin_libs})
  target_include_directories(${bt_plugin} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  )

  target_link_libraries(${bt_plugin}
    PUBLIC
    behaviortree_cpp::behaviortree_cpp
    ${geometry_msgs_TARGETS}
    $<$<BOOL:${easynav_interfaces_FOUND}>:${easynav_interfaces_TARGETS}>
    ${nav_msgs_TARGETS}
    PRIVATE
    ${tf2_geometry_msgs_TARGETS}
    ${std_msgs_TARGETS}
    nlohmann_json::nlohmann_json
    rclcpp_lifecycle::rclcpp_lifecycle
    rclcpp::rclcpp
    rclcpp_action::rclcpp_action
  )

  if(TARGET plansys2_bt_actions::plansys2_bt_actions)
    target_link_libraries(${bt_plugin}
      PUBLIC
      plansys2_bt_actions::plansys2_bt_actions
    )
  else()
    ament_target_dependencies(${bt_plugin}
      PUBLIC
      plansys2_bt_actions
    )
  endif()

  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(reception_controller_node src/reception_controller_node.cpp)
target_include_directories(reception_controller_node PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

if(plansys2_msgs_VERSION VERSION_GREATER_EQUAL "3.0.0")
  target_compile_definitions(reception_controller_node PRIVATE PLANSYS2_HAS_RESULT_ENUM)
endif()

target_link_libraries(reception_controller_node
  plansys2_monitor::plansys2_monitor
  ${plansys2_msgs_TARGETS}
  nlohmann_json::nlohmann_json
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_lifecycle::rclcpp_lifecycle
  ${std_msgs_TARGETS}
  ${tf2_TARGETS}
)

if(TARGET plansys2_domain_expert::plansys2_domain_expert)
  target_link_libraries(reception_controller_node
    plansys2_domain_expert::plansys2_domain_expert
    plansys2_executor::plansys2_executor
    plansys2_planner::plansys2_planner
    plansys2_problem_expert::plansys2_problem_expert
    plansys2_pddl_parser::plansys2_pddl_parser
  )
else()
  ament_target_dependencies(reception_controller_node
    plansys2_domain_expert
    plansys2_executor
    plansys2_planner
    plansys2_problem_expert
    plansys2_pddl_parser
  )
endif()

add_executable(perception_sim_node src/perception_sim_node.cpp)
target_link_libraries(perception_sim_node
  PRIVATE
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${plansys2_msgs_TARGETS}
  nlohmann_json::nlohmann_json
)

if(yolo_msgs_FOUND AND cv_bridge_FOUND AND OpenCV_FOUND)
  add_executable(perception_yolo_node src/perception_yolo_node.cpp)
  target_link_libraries(perception_yolo_node
    PRIVATE
    rclcpp::rclcpp
    ${std_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${yolo_msgs_TARGETS}
    cv_bridge::cv_bridge
    ${OpenCV_LIBS}
    nlohmann_json::nlohmann_json
  )
endif()

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch pddl bt_xml config params maps models scripts rviz
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS ${plugin_libs}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS reception_controller_node perception_sim_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(TARGET perception_yolo_node)
  install(TARGETS perception_yolo_node
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME})
ament_export_dependencies(
  behaviortree_cpp
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  plansys2_msgs
  geometry_msgs
  nav_msgs
  plansys2_domain_expert
  plansys2_executor
  plansys2_planner
  plansys2_problem_expert
  plansys2_pddl_parser
  plansys2_bt_actions
  plansys2_monitor
  lifecycle_msgs
  tf2
  tf2_geometry_msgs
)

ament_package()