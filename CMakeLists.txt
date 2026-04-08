cmake_minimum_required(VERSION 3.5...4.3)

project(ikd_tree)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-pthread -O3")

option(BUILD_EXAMPLES "Build example executables" OFF)

find_package(PCL 1.8 REQUIRED)

include_directories(
    ${PCL_INCLUDE_DIRS}
    ikd_tree
)

if(BUILD_EXAMPLES)
    add_executable(ikd_tree_demo examples/ikd_tree_demo.cpp ikd_tree/ikd_tree.cpp)
    target_link_libraries(ikd_tree_demo ${PCL_LIBRARIES})

    add_executable(ikd_tree_async_demo examples/ikd_tree_async_demo.cpp ikd_tree/ikd_tree.cpp)
    target_link_libraries(ikd_tree_async_demo ${PCL_LIBRARIES})

    add_executable(ikd_tree_search_demo examples/ikd_tree_search_demo.cpp ikd_tree/ikd_tree.cpp)
    target_link_libraries(ikd_tree_search_demo ${PCL_LIBRARIES})

    install(TARGETS ikd_tree_demo ikd_tree_async_demo ikd_tree_search_demo
        RUNTIME DESTINATION bin
    )
endif()

install(FILES ikd_tree/ikd_tree.h DESTINATION include/ikd_tree)
