cmake_minimum_required(VERSION 3.16)
project(pruneGDPSector)
include_directories(/usr/local/include)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-Wall -Wno-sign-compare -Wunused-variable -Wunused-variable -Wextra -pedantic -march=native -pipe -O3 -mmmx -msse -msse2 -msse3")
set(ENV_SRC monitor.cpp global.cpp lrucache.h labels.h metric.cpp util.cpp global.cpp libs/threadpool.h)
set(SIMD_SRC
        libs/simd/avx2.hpp
        libs/simd/branchless.hpp
        libs/simd/naive.hpp
        libs/simd/shuffle_dictionary.hpp)

add_executable(sample sample.cpp)
add_executable(pruneGDP algos/pruneGDP.cpp ${ENV_SRC} )
add_executable(SARD algos/sard.cpp ${ENV_SRC} )
add_executable(pruneGDPRand algos/RAND.cpp ${ENV_SRC} )
add_executable(GAS algos/gas_vldb.cpp ${ENV_SRC})
add_executable(RTV algos/rtv_pnas.cpp ${ENV_SRC})
add_executable(kineticSwitch algos/kinetic_switch_test.cpp ${ENV_SRC} libs/kinetic_tree/taxi_schedule.cpp)

target_link_libraries(SARD pthread)
target_link_libraries(pruneGDP pthread)
target_link_libraries(GAS pthread)
target_link_libraries(RTV pthread)
target_link_libraries(RTV glpk)
target_link_libraries(kineticSwitch pthread)