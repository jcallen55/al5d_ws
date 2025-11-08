cmake_minimum_required(VERSION 3.16)

project(AL5D_SSC32U_CLI
  VERSION 1.0.0
  DESCRIPTION "Terminal CLI to control Lynxmotion AL5D via SSC-32U on Raspberry Pi"
  LANGUAGES CXX)

option(AL5D_ENABLE_LTO "Enable link-time optimization in Release builds (if supported)" ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Create a library for the SSC-32U serial communication
add_library(ssc32u_serial src/Ssc32uSerial.cpp)
target_include_directories(ssc32u_serial PUBLIC ${CMAKE_SOURCE_DIR}/include/)

add_executable(ssc32u_cli src/ssc32u_cli.cpp)
target_link_libraries(ssc32u_cli PRIVATE ssc32u_serial)

# target_sources (none yet)

include_directories(${CMAKE_SOURCE_DIR}/include)

if(AL5D_ENABLE_LTO AND CMAKE_BUILD_TYPE MATCHES "Release")
  include(CheckIPOSupported)
  check_ipo_supported(RESULT _ipo_supported OUTPUT _ipo_msg)
  if(_ipo_supported)
    set_property(TARGET ssc32u_cli PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
  else()
    message(STATUS "IPO/LTO not supported: ${_ipo_msg}")
  endif()
endif()

include(GNUInstallDirs)
install(TARGETS ssc32u_cli RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

# Print summary
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Compiler:   ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
