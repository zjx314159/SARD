
#include <bits/stdc++.h>

using namespace std;

#include "util.h"
#include "mcmf.h"

void main_loop() {
    int cur = 0;
    while (pos < n) {
#ifdef WATCH_MEM
        if (pos % 20000 == 0) {
            watchSolutionOnce(getpid(), usedMemory);
        }
#endif
        if (pos / 10000 > cur) {
            cur = pos / 10000;
            cout << pos << endl;
        }

        for (int i = 0; i < m; ++i) {
            updateDriver(i, R[pos].tim);
        }
        tsd = clock();
        vector<int> car;
        if (R[pos].len < INF_WEIGHT) {
            car = single_search(R[pos].s, R[pos].ddl - R[pos].tim - R[pos].len);
        }

        double fit;

        ////pre processing
        vector<pair<double, int> > valued_car;
        for (int &i : car) {
            fit = INF;
            try_insertion_euclid(W[i], pos, fit);
            if (fit < INF)
                valued_car.emplace_back(fit, i);
        }
        random_shuffle(valued_car.begin(), valued_car.end());

        ////assign the request to some taxi
        assignTaxiFirst(valued_car);
        pos++;
    }

    for (int i = 0; i < m; ++i) {
        finishTaxi(i);
    }
    t3 = clock();
    dumpResult("main_loop");
}
struct timeval time_start, time_end;
int main(int argc, char **args) {
    if (argc > 4) vertexFile = string(args[1]);
    if (argc > 4) edgeFile = string(args[2]);
    if (argc > 4) pathLabelFile = string(args[3]);
    if (argc > 4) orderFile = string(args[4]);
    if (argc > 5) dataFile = string(args[5]);
    if (argc > 6) requestFile = string(args[6]);
    if (argc > 7) CAPACITY_CONSTRAINT = atoi(args[7]);
    if (argc > 8) GLOBAL_DDL = atof(args[8]);
    if (argc > 9) TAXI_SIZE = atoi(args[9]);
    if (argc > 10) REQUEST_SIZE = atoi(args[10]);
    if (argc > 11) GLOBAL_PENT = atoi(args[11]);
    if (argc > 12) {
        desFile = string(args[12]);
        freopen(desFile.c_str(), "w", stdout);
    }

    readInput();

    t1 = clock();
    initGrid();
    t2 = clock();

    gettimeofday(&time_start, NULL);
    main_loop();
    gettimeofday(&time_end, NULL);
    cout << "Running Time: "
         << ((time_end.tv_sec - time_start.tv_sec) * 1000 + (time_end.tv_usec - time_start.tv_usec) / 1000) << " ms"
         << endl;
    freeMemory();
    fflush(stdout);

    return 0;
}