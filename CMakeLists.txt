project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++17)

# FunctionalPlus
include(ExternalProject)
ExternalProject_Add(functional_plus
  GIT_REPOSITORY https://github.com/Dobiasd/FunctionalPlus.git
  GIT_TAG master

  SOURCE_DIR "${CMAKE_BINARY_DIR}/thirdparty/fplus"

  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND ""

  LOG_DOWNLOAD ON
  LOG_BUILD ON
)
set(FPLUS_INCLUDE_DIR ${CMAKE_BINARY_DIR}/thirdparty/fplus/include)
include_directories(${FPLUS_INCLUDE_DIR})


set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/map.cpp src/state_machine.cpp src/vehicle.cpp)



if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")


add_executable(path_planning src/main.cpp ${sources})
add_dependencies(path_planning functional_plus)

target_link_libraries(path_planning z ssl uv uWS)

# TEST CONFIG
enable_testing()
include_directories(tests)
include_directories(src)
set(test_sources tests/test_main.cpp tests/state_machine_test.cpp tests/vehicle_test.cpp)
add_executable(test_main ${sources} ${test_sources})
add_dependencies(test_main functional_plus)
add_test(MainTest test_main)
