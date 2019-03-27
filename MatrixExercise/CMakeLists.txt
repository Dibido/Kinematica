cmake_minimum_required(VERSION 2.8)

project (matrix_test)
find_package( Boost COMPONENTS system filesystem unit_test_framework REQUIRED )

include_directories( ${Boost_INCLUDE_DIR} )


add_executable(matrix_test
Main.cpp
Matrix_test.cpp
)

target_link_libraries(matrix_test ${Boost_FILESYSTEM_LIBRARY}
                      ${Boost_SYSTEM_LIBRARY}
                      ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})