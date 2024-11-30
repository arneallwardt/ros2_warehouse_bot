cmake_minimum_required(VERSION 3.8)
project(more_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)


#-----packages setup---------------------------------------

# package that generates code from msg/srv files
find_package(rosidl_default_generators REQUIRED)

# declare list of messages you want to create
set(msg_files
  "msg/AddressBook.msg"
)

# generate the messages
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

# export message runtime dependency
ament_export_dependencies(rosidl_default_runtime)

#--------------------------------------------------------


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
