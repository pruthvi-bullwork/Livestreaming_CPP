cmake_minimum_required(VERSION 3.8)
project(livestreaming_cpp)

add_compile_definitions(GST_USE_UNSTABLE_API)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED)

pkg_check_modules(GST REQUIRED gstreamer-1.0>=1.14)
pkg_check_modules(GSTAPP REQUIRED gstreamer-app-1.0)
pkg_check_modules(GSTWEBRTC REQUIRED gstreamer-webrtc-1.0)
pkg_check_modules(GSTSDP REQUIRED gstreamer-sdp-1.0)


# -------------------- Web (npm install) --------------------
option(INSTALL_WEB_NODE_MODULES "Run npm install in web/ and install node_modules" ON)

if(INSTALL_WEB_NODE_MODULES)
  find_program(NPM_EXECUTABLE npm)

  set(WEB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/web)
  set(WEB_STAMP ${CMAKE_CURRENT_BINARY_DIR}/web_npm_install.stamp)

  if(NPM_EXECUTABLE)
    # Prefer npm ci if package-lock.json exists, else npm install
    if(EXISTS "${WEB_DIR}/package-lock.json")
      set(NPM_INSTALL_CMD ${NPM_EXECUTABLE} ci --omit=dev)
    else()
      set(NPM_INSTALL_CMD ${NPM_EXECUTABLE} install --omit=dev)
    endif()

    add_custom_command(
      OUTPUT ${WEB_STAMP}
      COMMAND ${CMAKE_COMMAND} -E echo "Installing npm dependencies in ${WEB_DIR}"
      COMMAND ${NPM_INSTALL_CMD}
      COMMAND ${CMAKE_COMMAND} -E touch ${WEB_STAMP}
      WORKING_DIRECTORY ${WEB_DIR}
      COMMENT "npm install (web)"
      VERBATIM
    )

    add_custom_target(web_npm_install ALL DEPENDS ${WEB_STAMP})
  else()
    message(WARNING "npm not found. web/http_server.js may fail if it requires express. Install nodejs+npm or set INSTALL_WEB_NODE_MODULES=OFF.")
  endif()
endif()

# -------------------- C++ executable --------------------
add_executable(server src/server.cpp)

target_include_directories(server PRIVATE
  ${GST_INCLUDE_DIRS}
  ${GSTAPP_INCLUDE_DIRS}
  ${GSTWEBRTC_INCLUDE_DIRS}
  ${GSTSDP_INCLUDE_DIRS}
)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)



target_link_directories(server PRIVATE
  ${GST_LIBRARY_DIRS}
  ${GSTAPP_LIBRARY_DIRS}
  ${GSTWEBRTC_LIBRARY_DIRS}
  ${GSTSDP_LIBRARY_DIRS}
)

target_link_libraries(server
  ${GST_LIBRARIES}
  ${GSTAPP_LIBRARIES}
  ${GSTWEBRTC_LIBRARIES}
  ${GSTSDP_LIBRARIES}
)

target_link_libraries(server
  ${OpenCV_LIBS}
)

ament_target_dependencies(server
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
)

# Ensure npm install runs before building the server (optional but useful)
if(TARGET web_npm_install)
  add_dependencies(server web_npm_install)
endif()

# -------------------- Install --------------------
install(TARGETS server
  DESTINATION lib/${PROJECT_NAME}
)

# Install web directory (includes node_modules if created in source web/)
install(DIRECTORY web
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
