cmake_minimum_required(VERSION 3.13)

project(brl_urdf VERSION 0.1.0)

message(STATUS "=============  !BRL Custom URDF Parser!  =============")

################################################################################
# Options
################################################################################
option(BUILD_TESTS "Building the tests" ON)

################################################################################
# Setup Compilation
################################################################################

message(STATUS "======> Setup Compilation")

set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)
set(CMAKE_DISABLE_SOURCE_CHANGES  ON)

if ("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
  message(SEND_ERROR "In-source builds are not allowed.")
endif ()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_COLOR_MAKEFILE ON)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${PROJECT_SOURCE_DIR}/cmake")
set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};${PROJECT_SOURCE_DIR}/cmake")

set(CMAKE_BUILD_TYPE Release)
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_definitions(-DDEBUG_MODE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -ggdb -Wall \
  -Wextra -Wcast-align -Wdisabled-optimization -Wformat=2 \
  -Winit-self -Wmissing-include-dirs -Woverloaded-virtual \
  -Wshadow -Wsign-promo -Wno-sign-compare -Wno-unused-const-variable \
  -Wno-unused-parameter -Wno-unused-variable -Wno-uninitialized")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3  -ggdb -std=gnu99 -I.")

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.h.in ${CMAKE_CURRENT_BINARY_DIR}/config.h)

################################################################################
# Specify Build Resources
################################################################################
message(STATUS "======>  Setup Build  ")

include_directories(include)
include_directories(${CMAKE_CURRENT_BINARY_DIR})
file(GLOB_RECURSE SRCS "src/*.cpp")

################################################################################
# Setup Build
################################################################################

if(NOT SRCS)
  message(FATAL_ERROR "No source files found. Please add something")
	set(LIBRARY_NAME)
else()
  add_library(${PROJECT_NAME} ${SRCS})

  set(LIBRARY_NAME ${PROJECT_NAME})
  set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE TRUE) 

  install(TARGETS ${PROJECT_NAME}
          EXPORT ${PROJECT_NAME}Targets
          LIBRARY DESTINATION lib
          ARCHIVE DESTINATION lib
          RUNTIME DESTINATION bin
          INCLUDES DESTINATION include)

  install(DIRECTORY include/ DESTINATION include)

  include(CMakePackageConfigHelpers)
  write_basic_package_version_file(
    ${PROJECT_NAME}ConfigVersion.cmake
    VERSION ${PACKAGE_VERSION}
    COMPATIBILITY AnyNewerVersion
  )

  configure_package_config_file(
    ${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in
    ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
  )

  install(EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION lib/cmake/${PROJECT_NAME})

  install(FILES
    ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    ${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    DESTINATION lib/cmake/${PROJECT_NAME}
  )
endif()

if(BUILD_TESTS)
  message(STATUS "======>  Setup Tests  ")
  enable_testing()
  add_subdirectory(tests)
endif()
