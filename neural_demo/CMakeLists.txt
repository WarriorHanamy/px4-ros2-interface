cmake_minimum_required(VERSION 3.5)
project(neural_demo)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(Eigen3 REQUIRED)


include_directories(include ${Eigen3_INCLUDE_DIRS})

# Main executor node
file(GLOB ${PROJECT_NAME}_SRCS src/*.cpp)
ament_auto_add_executable(neural_demo_node src/main.cpp)

# Fake network node
ament_auto_add_executable(fake_network_node src/fake_network_node.cpp)

target_link_libraries(
    neural_demo_node
    Eigen3::Eigen
)

target_link_libraries(
    fake_network_node
    Eigen3::Eigen
)

install(TARGETS
        neural_demo_node
        fake_network_node
        DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY
        launch
        config
        DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
