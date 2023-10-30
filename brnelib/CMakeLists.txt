cmake_minimum_required(VERSION 3.9)
project(brnelib)

find_package(Armadillo REQUIRED)
include_directories(${ARMADILLO_INCLUDE_DIRS})

# create the library
add_library(brnelib src/brne.cpp)
target_link_libraries(brnelib ${ARMADILLO_LIBRARIES})
# enable C++ 17
target_compile_features(brnelib PUBLIC cxx_std_17) 
# warnings are your friend!
target_compile_options(brnelib PUBLIC -Wall -Wextra -Wpedantic)

add_executable(frame_main src/frame_main.cpp)
target_link_libraries(frame_main brnelib)


if(NOT CMAKE_CROSSCOMPILING)
# stuff you don't want to be done when cross compiling
# CMake also has the ability to generate doxygen documentation
find_package(Doxygen)
set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md) # Use the readme in your doxygen docs
doxygen_add_docs(doxygen include/ src/ README.md ALL)
# Use the cmake testing functionality. A test is just an executable.
# We use the catch2 unit testing framework
find_package(Catch2 3 REQUIRED)
include(CTest)
# enable_testing()
add_executable(brne_test tests/brne_test.cpp)
target_link_libraries(brne_test brnelib Catch2::Catch2WithMain)
add_test(NAME Test_of_Brnelib COMMAND brne_test)
endif()

target_include_directories(brnelib
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_options(frame_main PUBLIC -Wall)
# install the include directories
install(DIRECTORY include/brnelib DESTINATION include)
# Install the targets and create a CMake Exported Target
install(TARGETS frame_main brnelib EXPORT brnelib-targets)
# The project_name-targets now also needs to be exported.
install(EXPORT brnelib-targets
        FILE brnelib-config.cmake
        NAMESPACE brnelib::
        DESTINATION lib/cmake/${PROJECT_NAME})