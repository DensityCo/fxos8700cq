cmake_minimum_required(VERSION 3.10)
project(fxos8700cq)

include_directories ("${PROJECT_SOURCE_DIR}/source")
link_directories(BEFORE ${CMAKE_PREFIX_PATH}/lib)
include_directories(BEFORE ${CMAKE_PREFIX_PATH}/include)
ADD_DEFINITIONS ( -DPQXX_HIDE_EXP_OPTIONAL )
set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CXX_FLAGS_RELEASE "-std=c++11 -O0")
set(CMAKE_CXX_FLAGS_DEBUG "-std=c++11 -O0 -g")

add_library(fxos8700cq SHARED fxos8700cq.cpp)
add_executable(fxos-test fxos-test.cpp)

target_link_libraries(fxos-test fxos8700cq)
install(TARGETS fxos8700cq DESTINATION lib)
install(TARGETS fxos-test DESTINATION bin)
install(FILES fxos8700cq.h DESTINATION include)

