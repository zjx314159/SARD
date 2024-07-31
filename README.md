# StructRide: a Framework to Exploit the Structure Information of Shareability Graph in Ridesharing Services
***
The experiment code of StructRide is stored in this repository for reproducing the experimental results.

## Brief Introduction
* `algos`: The entry of the implementation of baseline algorithms and `SARD`.
* `libs`: External useful libraries
    * `threadpool`: Thread Pool
    * `kinetic_tree`: Schedule maintenance structure of [KineticTree](http://www.vldb.org/pvldb/vol7/p2017-huang.pdf)
    * `simd`: Single Instruction Multiple Data library Used for speeding up set intersection operations.
* `scripts`: Convenient experimental scripts.
* `metric`: Shortest Path/Dist Query library. (hub labeling based shortest path algorithms [\[url\]](http://www.vldb.org/pvldb/vol11/p445-li.pdf) ) 
* `lrucache`: LRU cache library.
* `global`: Global variables.
* `util`: Useful util functions in the ridesharing lifecycle.

## Environment

gcc/g++ version: 7.4.0 

OS: Ubuntu

## Compile the algorithms

`cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=TRUE -B cmake-build-debug -G "Unix Makefiles"`

`cmake --build cmake-build-debug --config Debug -j 14 --`

## Run the algorithms

Run the SARD algorithm with different pruning strategies:

`scripts/bench_sard.sh [--dataset] SARD [--opt]`

`opt` represents different pruning modes, where 0 means no pruning strategy and 2 means angle pruning is used.

Run the batch-based algorithms:

`scripts/bench_batch.sh [--dataset] [--algo]`

The batch-based algorithms include `gas`, `rtv` and `sard`.

Run the online-based algorithms:

`scripts/bench_sequential.sh [--dataset] [--algo]`

The online-based algorithms include `gdp` and `rand`.

## Road Networks
You can download the road network from [OpenStreetMap](https://www.openstreetmap.org/).
Besides, since we adopts the hub labeling algorithm for the shortest path query, we need to use algorithm in [link](https://github.com/BUAA-BDA/sspexp_clone) to preprocess the road network.

## Requests and Workers
You can apply for the requests data of `Chengdu` on [Didi Gaia Platform](https://outreach.didichuxing.com/research/opendata/) 
and download the yellow and green taxi data from the official website of [New York City](https://www1.nyc.gov/site/tlc/about/tlc-trip-record-data.page).


## Contact
- Yu Chen: yu.chen@stu.ecnu.edu.cn
- Jiexi Zhan: jxzhan@stu.ecnu.edu.cn
- Peng Cheng: pcheng@sei.ecnu.edu.cn
