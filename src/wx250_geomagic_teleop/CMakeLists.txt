cmake_minimum_required(VERSION 3.8)
project(wx250_geomagic_teleop)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python package (src folder must be python package named like project)
ament_python_install_package(${PROJECT_NAME})

# Console scripts live in lib/<pkg>
install(PROGRAMS
  wx250_geomagic_teleop/geomagic_to_wx250.py
  wx250_geomagic_teleop/virtual_geomagic.py
  DESTINATION lib/${PROJECT_NAME}
)

# Launch files
install(FILES
  wx250_geomagic_teleop.launch.py
  wx250_virtual_geomagic_sim.launch.py
  DESTINATION share/${PROJECT_NAME}
)

# Register package in ament index (this is what `ros2 pkg`/`ros2 launch` needs)
# The marker file must be named exactly like the package.
install(FILES
  resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages
)

# Also install package.xml for completeness
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
