cmake_minimum_required(VERSION 3.8)
project(polaris_ultrasound)

# 找到必要的包
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# 将Python包安装到正确位置
ament_python_install_package(${PROJECT_NAME})

# 安装Python脚本
install(PROGRAMS
  polaris_ultrasound/polaris_reader_node.py
  polaris_ultrasound/ultrasound_reader_node.py
  polaris_ultrasound/calibration_recorder.py
  polaris_ultrasound/phantom_experiment.py
  DESTINATION lib/${PROJECT_NAME}
)

# 安装launch文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Python依赖
ament_package()