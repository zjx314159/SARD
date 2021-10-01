# STRide: a Framework to Exploit the Structure Information of Shareability Graph in Ridesharing Services
***
The experiment code of STRide is stored in this repository for reproducing the experimental results.

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

## Road Networks
You can download the road network from [OpenStreetMap](https://www.openstreetmap.org/)
Besides, since we adopts the hub labeling algorithm for the shortest path query, we need to use algorithm in [link](https://github.com/BUAA-BDA/sspexp_clone) to preprocess the road network.

## Requests and Workers
You can apply for the requests data of `Chengdu` on [Didi Gaia Platform](https://outreach.didichuxing.com/research/opendata/) 
and download the yellow and green taxi data from the official website of [New York City](https://www1.nyc.gov/site/tlc/about/tlc-trip-record-data.page).


## Contact
- Yu Chen: yu.chen@stu.ecnu.edu.cn
- Peng Cheng: pcheng@sei.ecnu.edu.cn