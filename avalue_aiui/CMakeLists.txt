cmake_minimum_required(VERSION 3.8)
project(avalue_aiui)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  /usr/include/alsa
  ${CMAKE_CURRENT_LIST_DIR}/include
  ${CMAKE_CURRENT_LIST_DIR}/src/utils
  ${CMAKE_CURRENT_LIST_DIR}/src/jsoncpp
)

install(DIRECTORY launch AIUI
  DESTINATION share/${PROJECT_NAME}
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ollama_ros_msgs REQUIRED)

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

file(GLOB_RECURSE SOURCE_FILE
  "src/*.cpp"
)

# Normalize uname/getconf output (strip trailing newlines/spaces)
execute_process(COMMAND uname -m
  OUTPUT_VARIABLE MACHINES
  OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND getconf LONG_BIT
  OUTPUT_VARIABLE MACHINES_BIT
  OUTPUT_STRIP_TRAILING_WHITESPACE)

message(STATUS "AIUI Machine: ${MACHINES}")
message(STATUS "AIUI Machine Bits: ${MACHINES_BIT}")

# Select correct libaiui.so directory by architecture
if((${MACHINES} MATCHES "x86_64") AND (${MACHINES_BIT} MATCHES "64"))
  # Some projects use x64, some use x86_64. You said both x64 and x86 exist.
  set(AIUI_LIB_DIR "${CMAKE_CURRENT_LIST_DIR}/libs/x64")
elseif((${MACHINES} MATCHES "aarch64") AND (${MACHINES_BIT} MATCHES "64"))
  set(AIUI_LIB_DIR "${CMAKE_CURRENT_LIST_DIR}/libs/arm64")
elseif(${MACHINES} MATCHES "arm")
  # Fallback: if you also have 32-bit arm libs, put them in libs/arm
  set(AIUI_LIB_DIR "${CMAKE_CURRENT_LIST_DIR}/libs/arm")
else()
  message(FATAL_ERROR "Unsupported architecture: ${MACHINES} (${MACHINES_BIT}-bit)")
endif()

# If x64 path doesn't exist, fallback to x86 (as you mentioned libs/x86 also exists)
if(NOT EXISTS "${AIUI_LIB_DIR}/libaiui.so")
  if((${MACHINES} MATCHES "x86_64") AND (${MACHINES_BIT} MATCHES "64") AND EXISTS "${CMAKE_CURRENT_LIST_DIR}/libs/x86/libaiui.so")
    set(AIUI_LIB_DIR "${CMAKE_CURRENT_LIST_DIR}/libs/x86")
  endif()
endif()

if(NOT EXISTS "${AIUI_LIB_DIR}/libaiui.so")
  message(FATAL_ERROR "libaiui.so not found in ${AIUI_LIB_DIR}. Please ensure the correct lib is present under libs/arm64, libs/x64, or libs/x86.")
endif()

message(STATUS "Using AIUI library from: ${AIUI_LIB_DIR}")

link_directories(${AIUI_LIB_DIR})

add_executable(${PROJECT_NAME} ${SOURCE_FILE})
ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs ollama_ros_msgs)
target_link_libraries(${PROJECT_NAME} aiui asound pthread stdc++)

install(TARGETS
  ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install the selected AIUI shared library so runtime can find it
install(FILES "${AIUI_LIB_DIR}/libaiui.so"
  DESTINATION lib
)

ament_package()
