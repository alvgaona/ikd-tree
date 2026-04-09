cmake_minimum_required(VERSION 3.5...4.3)

project(ikd_tree)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-pthread -O3")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(BUILD_EXAMPLES "Build example executables" OFF)

find_package(PCL 1.8 REQUIRED)

include_directories(
    ${PCL_INCLUDE_DIRS}
    ikd_tree
)

# Workaround: conda-forge's boost_headers-config.cmake hardcodes
# `_BOOST_CMAKEDIR` to the build-time runner path (e.g.
# /Users/runner/miniforge3/conda-bld/.../temp_prefix/lib/cmake), so
# Boost::headers' INTERFACE_INCLUDE_DIRECTORIES points at a path that
# does not exist on the user's machine. Until that upstream packaging
# bug is fixed, inject the bare conda env include dir manually.
if(NOT "$ENV{CONDA_PREFIX}" STREQUAL "")
    add_compile_options("-I$ENV{CONDA_PREFIX}/include")
endif()

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

include(CTest)
if(BUILD_TESTING)
    find_package(GTest REQUIRED)
    add_executable(ikd_tree_tests tests/test_ikd_tree.cpp ikd_tree/ikd_tree.cpp)
    target_include_directories(ikd_tree_tests PRIVATE ${PCL_INCLUDE_DIRS} ikd_tree)
    target_link_libraries(ikd_tree_tests GTest::gtest_main ${PCL_LIBRARIES})
    add_test(NAME ikd_tree_tests COMMAND ikd_tree_tests)
endif()

option(BUILD_BENCHMARKS "Build kNN benchmarks" OFF)
if(BUILD_BENCHMARKS)
    find_package(benchmark REQUIRED)
    add_executable(ikd_tree_bench bench/bench_ikd_tree.cpp ikd_tree/ikd_tree.cpp)
    target_include_directories(ikd_tree_bench PRIVATE ${PCL_INCLUDE_DIRS} ikd_tree)
    target_link_libraries(ikd_tree_bench benchmark::benchmark_main ${PCL_LIBRARIES})
endif()

install(FILES ikd_tree/ikd_tree.h DESTINATION include/ikd_tree)
