cmake_minimum_required(VERSION 3.8)
project(rviz_goal_panel)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)

set(CMAKE_AUTOMOC ON)
qt5_wrap_cpp(MOC_FILES
  include/rviz_goal_panel/goal_panel.hpp
)

add_library(goal_panel src/goal_panel.cpp ${MOC_FILES})
target_include_directories(goal_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(goal_panel
  pluginlib
  rviz_common
)
install(TARGETS goal_panel
        EXPORT export_rviz_goal_panel
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(DIRECTORY include/
        DESTINATION include
)
install(FILES rviz_common_plugins.xml
        DESTINATION share/${PROJECT_NAME}
)
# install(FILES icons/classes/DemoPanel.png
#         DESTINATION share/${PROJECT_NAME}/icons/classes
# )
ament_export_include_directories(include)
ament_export_targets(export_rviz_goal_panel)
pluginlib_export_plugin_description_file(rviz_common rviz_common_plugins.xml)

ament_package()
