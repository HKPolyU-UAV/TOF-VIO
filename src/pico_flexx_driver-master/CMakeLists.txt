cmake_minimum_required(VERSION 2.8.3)
project(pico_flexx_driver)

set(additional_cxx_flags "-DBT_USE_DOUBLE_PRECISION")
# Unused warnings
set(additional_cxx_flags "${additional_cxx_flags} -Wuninitialized -Winit-self -Wunused-function -Wunused-label -Wunused-variable -Wunused-but-set-variable -Wunused-but-set-parameter")
# Additional warnings
set(additional_cxx_flags "${additional_cxx_flags} -Warray-bounds -Wtype-limits -Wreturn-type -Wsequence-point -Wparentheses -Wmissing-braces -Wchar-subscripts -Wswitch -Wwrite-strings -Wenum-compare -Wempty-body -Wlogical-op")

# Check for c++11 support
INCLUDE(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
IF(COMPILER_SUPPORTS_CXX11)
SET(additional_cxx_flags "${additional_cxx_flags} -std=c++11")
ELSEIF(COMPILER_SUPPORTS_CXX0X)
SET(additional_cxx_flags "${additional_cxx_flags} -std=c++0x")
ELSE()
  MESSAGE(ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
ENDIF()

# Some versions of libroyale (e.g. 3.1.0.122) needed -march=native to work
# Version 3.4.0.18 works fine without, but adding this should not hurt
CHECK_CXX_COMPILER_FLAG("-march=native" COMPILER_OPT_ARCH_NATIVE_SUPPORTED)
if (COMPILER_OPT_ARCH_NATIVE_SUPPORTED)
  set(additional_cxx_flags "${additional_cxx_flags} -march=native")
endif()


set(CMAKE_MODULE_PATH   ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  nodelet
  roscpp
  rostime
  sensor_msgs
  std_msgs
  dynamic_reconfigure
)

## System dependencies are found with CMake's conventions
find_package(royale REQUIRED)

# combine cmake_cxx_flags and additional_cxx_flags
# plus change the space-separated string, into a list delimited by semicolons
# cxx_flags will be used with target_compile_options (cleaner than changing CMAKE_CXX_FLAGS) 
string(REPLACE " " ";" cxx_flags "${CMAKE_CXX_FLAGS} ${additional_cxx_flags}")


# debugging output
MESSAGE( STATUS "CMAKE_CXX_FLAGS: " ${CMAKE_CXX_FLAGS})
MESSAGE( STATUS "additional_cxx_flags: " ${additional_cxx_flags})
MESSAGE( STATUS "cxx_flags: ${cxx_flags}")
MESSAGE( STATUS "royale version: ${royale_VERSION}")


################################################
## Declare ROS messages, services and actions ##
################################################

generate_dynamic_reconfigure_options(
  cfg/pico_flexx_driver.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS nodelet roscpp rostime sensor_msgs std_msgs dynamic_reconfigure
)

###########
## Build ##
###########

# Display additional files in qtcreator
execute_process(COMMAND find -L ${PROJECT_SOURCE_DIR} -type f ! -path "${PROJECT_SOURCE_DIR}/royale/*" OUTPUT_VARIABLE FILES_STRING)
string(REPLACE "\n" ";" FILES_LIST ${FILES_STRING})
add_custom_target(additional_files_${PROJECT_NAME}
  SOURCES
  ${FILES_LIST}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${royale_INCLUDE_DIRS}
)

link_directories(${royale_LIB_DIR})
set(CMAKE_INSTALL_RPATH ${royale_LIB_DIR})

add_library(pico_flexx_nodelet SHARED src/pico_flexx_driver.cpp)
target_link_libraries(pico_flexx_nodelet
  ${catkin_LIBRARIES}
  ${royale_LIBRARIES}
)
add_dependencies(pico_flexx_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_compile_options(pico_flexx_nodelet PUBLIC ${cxx_flags})
target_compile_definitions(pico_flexx_nodelet PUBLIC royale_VERSION_MAJOR=${royale_VERSION_MAJOR} royale_VERSION_MINOR=${royale_VERSION_MINOR})

add_executable(pico_flexx_driver src/pico_flexx_driver.cpp)
target_link_libraries(pico_flexx_driver
  ${catkin_LIBRARIES}
  ${royale_LIBRARIES}
)
add_dependencies(pico_flexx_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_compile_options(pico_flexx_driver PUBLIC ${cxx_flags})
target_compile_definitions(pico_flexx_driver PUBLIC royale_VERSION_MAJOR=${royale_VERSION_MAJOR} royale_VERSION_MINOR=${royale_VERSION_MINOR})



#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS pico_flexx_driver pico_flexx_nodelet
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  launch
  # myfile2
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES
  nodelet_plugins.xml
  # myfile2
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
