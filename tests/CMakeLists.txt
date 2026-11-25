cmake_minimum_required(VERSION 3.20)
project(VorotWifiTests LANGUAGES CXX)

find_package(Threads REQUIRED)
find_path(FFTW3_INCLUDE_DIR fftw3.h REQUIRED)
find_library(FFTW3F_LIBRARY NAMES fftw3f libfftw3f-3 libfftw3f.so.3 REQUIRED)
set(SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/../Source")

add_executable(wifi_tests
    wifi_tests.cpp
    "${SOURCE}/HTSIGDecoder.cpp"
    "${SOURCE}/LLTFSynchronizer.cpp"
    "${SOURCE}/LSIGDecoder.cpp"
    "${SOURCE}/LStfDetector.cpp"
    "${SOURCE}/MacFrameParser.cpp"
    "${SOURCE}/WiFiDemodulator.cpp"
    "${SOURCE}/WifiBcc.cpp"
    "${SOURCE}/WifiDataDecoder.cpp"
    "${SOURCE}/WifiOfdm.cpp")
target_compile_features(wifi_tests PRIVATE cxx_std_20)
target_include_directories(wifi_tests PRIVATE "${SOURCE}" "${FFTW3_INCLUDE_DIR}")
target_link_libraries(wifi_tests PRIVATE "${FFTW3F_LIBRARY}" Threads::Threads)
if(MSVC)
    target_compile_options(wifi_tests PRIVATE /W4 /utf-8)
    target_compile_definitions(wifi_tests PRIVATE _SILENCE_NONFLOATING_COMPLEX_DEPRECATION_WARNING)
else()
    target_compile_options(wifi_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

enable_testing()
foreach(suite IN ITEMS coding mac phy stream)
    add_test(NAME wifi_${suite} COMMAND wifi_tests ${suite} "${CMAKE_CURRENT_SOURCE_DIR}/vectors")
endforeach()
