cmake_minimum_required(VERSION 3.0.0)
project(hand-eye VERSION 0.1.0)

include(CTest)
enable_testing()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin/)

include_directories(${PROJECT_SOURCE_DIR}/src/)
link_directories(${PROJECT_SOURCE_DIR}/lib/)

find_package(OpenCV 4.5 REQUIRED)

find_package(Boost REQUIRED filesystem)
include_directories(${OpenCV_INCLUDE_DIRS})

add_subdirectory(${PROJECT_SOURCE_DIR}/src/)

add_executable(hand-eye main.cpp)
target_link_libraries(hand-eye PUBLIC ${OpenCV_LIBS} ${Boost_LIBRARIES} file_manip calib_intrinsic)

add_executable(he_calib handeye-calib.cpp)
target_link_libraries(he_calib PUBLIC ${OpenCV_LIBS} ${Boost_LIBRARIES} file_manip calib_intrinsic)

add_executable(cb_calib chessboard_test.cpp)
target_link_libraries(cb_calib PUBLIC ${OpenCV_LIBS} ${Boost_LIBRARIES} file_manip calib_intrinsic)

add_executable(veri veri.cpp)

add_executable(pop pop_ideas.cpp)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
