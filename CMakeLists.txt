cmake_minimum_required(VERSION 3.16)
project(otto_suite LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(SOEM_BUILD_SAMPLES OFF CACHE BOOL "" FORCE)
add_subdirectory(third_party/SOEM)

add_library(cia402
  src/cia402/state_machine.cpp
)
target_include_directories(cia402 PUBLIC src)

add_library(ethercat
  src/ethercat/soem_master.cpp
)
target_include_directories(ethercat PUBLIC src)
target_link_libraries(ethercat PUBLIC soem)

add_library(zeroerr
  src/zeroerr/zeroerr_actuator.cpp
)
target_include_directories(zeroerr PUBLIC src)
target_link_libraries(zeroerr PUBLIC cia402 ethercat)

add_executable(zeroerr_move tools/zeroerr_move.cpp)
target_link_libraries(zeroerr_move PRIVATE zeroerr)

add_executable(zeroerr_state tools/zeroerr_state.cpp)
target_link_libraries(zeroerr_state PRIVATE zeroerr)
