cmake_minimum_required(VERSION 3.16)
project(pruneGDPSector)
include_directories(/usr/local/include)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-Wall -Wno-sign-compare -Wunused-variable -Wunused-variable -Wextra -pedantic -march=native -pipe -O3 -mmmx -msse -msse2 -msse3")
set(ENV_SRC monitor.cpp global.cpp lrucache.h labels.h metric.cpp util.cpp global.cpp mcmf.h threadpool.h)
set(SIMD_SRC
        simd/avx2.hpp
        simd/branchless.hpp
        simd/naive.hpp
        simd/shuffle_dictionary.hpp)

add_executable(sample sample.cpp)
add_executable(pruneGDP pruneGDP.cpp ${ENV_SRC} )
add_executable(SARD sard_vldb.cpp ${ENV_SRC} )
add_executable(SARDDynamicGraph sard_dynamic_graph.cpp ${ENV_SRC} )
add_executable(pruneGDPRand pruneGDP_random.cpp ${ENV_SRC} )
add_executable(GAS gas_vldb.cpp ${ENV_SRC})
add_executable(GASDynamic gas_vldb_dynamic.cpp ${ENV_SRC})
add_executable(RTV rtv_pnas.cpp ${ENV_SRC})
add_executable(kineticSwitch kinetic_switch_test.cpp ${ENV_SRC}  kinetic_tree/taxi_schedule.cpp)
add_executable(kineticSwitch2 kinetic_switch_test_2.cpp ${ENV_SRC}  kinetic_tree/taxi_schedule.cpp)

target_link_libraries(SARD pthread)
target_link_libraries(SARDDynamicGraph pthread)
target_link_libraries(pruneGDP pthread)
target_link_libraries(GAS pthread)
target_link_libraries(GASDynamic pthread)
target_link_libraries(RTV pthread)
target_link_libraries(RTV glpk)
target_link_libraries(kineticSwitch pthread)
target_link_libraries(kineticSwitch2 pthread)