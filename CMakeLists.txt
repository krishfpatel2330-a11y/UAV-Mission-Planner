cmake_minimum_required(VERSION 3.16)
project(uav_mission_planner LANGUAGES CXX)
 
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
 
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()
 
option(UMP_BUILD_TESTS "Build the unit test suite" ON)
option(UMP_WERROR "Treat warnings as errors" ON)
 
add_library(ump
  src/terrain.cpp
  src/airspace.cpp
  src/energy.cpp
  src/planner.cpp
  src/astar.cpp
  src/rrt_star.cpp
  src/mission.cpp
  src/scenario.cpp
)
target_include_directories(ump PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
 
if(MSVC)
  target_compile_options(ump PRIVATE /W4)
else()
  target_compile_options(ump PRIVATE -Wall -Wextra -Wpedantic -Wshadow -Wconversion)
  if(UMP_WERROR)
    target_compile_options(ump PRIVATE -Werror)
  endif()
endif()
 
add_executable(ump-plan src/main.cpp)
target_link_libraries(ump-plan PRIVATE ump)
 
if(UMP_BUILD_TESTS)
  include(FetchContent)
  FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.15.2
  )
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
 
  enable_testing()
  add_executable(ump-tests
    tests/test_airspace.cpp
    tests/test_energy.cpp
    tests/test_planners.cpp
    tests/test_mission.cpp
  )
  target_link_libraries(ump-tests PRIVATE ump GTest::gtest_main)
  include(GoogleTest)
  gtest_discover_tests(ump-tests)
endif()
