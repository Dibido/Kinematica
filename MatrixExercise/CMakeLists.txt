cmake_minimum_required(VERSION 2.8)

project (kinematics)
find_package( Boost COMPONENTS system filesystem unit_test_framework REQUIRED )
find_package(OpenCV 3.2.0 REQUIRED)

include_directories( ${Boost_INCLUDE_DIR} )


add_executable(kinematics
Main.cpp
Matrix_test.cpp
DetectColor.cpp
DetectShapes.cpp
Shapedetector.cpp
)

target_link_libraries(kinematics ${Boost_FILESYSTEM_LIBRARY}
                      ${Boost_SYSTEM_LIBRARY}
                      ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

target_link_libraries(kinematics ${OpenCV_LIBS})