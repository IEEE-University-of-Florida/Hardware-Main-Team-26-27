cmake_minimum_required(VERSION 3.20)
project(otos_simulator LANGUAGES CXX)

enable_testing()
add_executable(otos_tests
    src/otos.cpp
    test/test_otos.cpp
    ../src/otos_localization.cpp
    ../src/otos_odometry.cpp
)
target_include_directories(otos_tests PRIVATE src test ../include)
target_compile_features(otos_tests PRIVATE cxx_std_11)
add_test(NAME otos_simulator COMMAND otos_tests)
