cmake_minimum_required(VERSION 3.8)
project(self_check)

find_package(ament_cmake REQUIRED)

# Install the Python script as an executable
install(PROGRAMS
  src/selfcheck.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
