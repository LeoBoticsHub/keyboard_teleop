cmake_minimum_required(VERSION 3.5)
project(keyboard_teleop)


find_package(catkin QUIET)
find_package(ament_cmake QUIET)

if(${catkin_FOUND})

  catkin_package()

  catkin_install_python(PROGRAMS
    scripts/keyboard_teleop_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  set(SHARE_DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

elseif(${ament_cmake_FOUND})

  ament_package()

  # Install Python executables (this is used to install the node executables)
  install(PROGRAMS
    scripts/ros2_keyboard_teleop.py
    scripts/keyboard_listener.py
    DESTINATION lib/${PROJECT_NAME}
  )

  set(SHARE_DESTINATION share/${PROJECT_NAME})

endif()

# Install directories
set(directories launch config)
foreach(dir IN LISTS directories)
  install(
    DIRECTORY ${dir}
    DESTINATION ${SHARE_DESTINATION}
  )
endforeach()


