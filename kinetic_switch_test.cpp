
#include <bits/stdc++.h>

using namespace std;

#include "util.h"
#include "mcmf.h"

threadpool *pool;
int batch_time = 5;

void pruneGDP(int kineticSize) {
    int cur = 0, src = 0, dst = 0;
    CURRENT_TIME = R[0].tim + batch_time;
    set<int> working_set;
    while (pos < n) {
#ifdef WATCH_MEM
        if (pos % 20000 == 0) {
            watchSolutionOnce(getpid(), usedMemory);
        }
#endif
        if (R[pos++].tim <= CURRENT_TIME) continue;
        src = dst;
        dst = pos;
        if (pos / 2000 > cur) {
            cur = pos / 2000;
            cout << pos << endl;
        }

//        for (int i = 0; i < m; ++i) {
//            if (W[i].path == nullptr) continue;
//            W[i].path->update(batch_time);
//            updateDriver(i, CURRENT_TIME);
//        }
        for (int i = src; i < dst; ++i) {
            if (R[i].len < INF_WEIGHT)
                working_set.insert(i);
        }

        tsd = clock();
        set<int> accepted;
        for (int req:working_set) {
            vector<int> car;
            if (R[req].len < INF_WEIGHT) {
                car = single_search(R[req].s, R[req].ddl - R[req].tim);
            }

            double fit;

            ////pre processing
            vector<pair<double, int> > valued_car;
            vector<future<vector<pair<double, int>>>> futures;
            vector<vector<int>> cands_group;
            uint batch_size = sqrt(car.size());
            for (uint i = 0; i < car.size();) {
                vector<int> cands;
                for (uint j = 0; j < batch_size && i < car.size(); ++j) {
                    cands.push_back(car[i++]);
                }
                cands_group.push_back(cands);
            }
            for (const auto &g:cands_group) {
                futures.push_back(pool->commit([g, kineticSize, req] {
                    vector<pair<double, int>> res;
                    double fit = INF;
                    for (auto i:g) {
                        if (W[i].path == nullptr) {
                            auto tmp = new TaxiSchedule();
                            double f = tmp->value(req);
                            if (f == -1) {
                                delete tmp;
                                continue;
                            }
                            tmp->push();
                            fit = tmp->valueDriver(i);
                            delete tmp;
                        } else {
                            if (W[i].path->requests.size() >= kineticSize) fit = -1;
                            else {
                                fit = W[i].path->value(req);
                                W[i].path->cancel();
                            }
                        }
                        if (fit != -1 && fit < INF)
                            res.emplace_back(fit, i);
                    }
                    return res;
                }));
            }
            for (auto &future : futures) {
                auto tmp_car = future.get();
                for (auto p:tmp_car) valued_car.emplace_back(p);
            }
            sort(valued_car.begin(), valued_car.end());
            if (R[req].len > INF || valued_car.empty() || valued_car[0].first > INF) {
                if (R[req].len < INF_WEIGHT) {
                    Penalty += R[req].len * R[req].pr;
                    cnt++;
                }
            } else {
                auto tmp = new TaxiSchedule();
                tmp->value(req);
                tmp->push();
                tmp->pushDriver(valued_car[0].second);
                delete tmp;
                accepted.insert(req);
            }
            ted = clock();
            dispatchTime += 1. * (ted - tsd) / CLOCKS_PER_SEC;
        }

        for (auto it = working_set.begin(); it != working_set.end();) {
            int r = *it;
            if (R[r].ddl < (CURRENT_TIME + R[r].len)) working_set.erase(it++);
            else it++;
        }
        CURRENT_TIME += batch_time;
    }
    cout << endl;

    for (int i = 0; i < m; ++i) {
        finishTaxi(i);
    }
    cout << "Kinetic Test: " << KINETIC_SWITCH_COUNT << " / " << KINETIC_ACCEPT << endl;
    t3 = clock();
    dumpResult("main_loop");
}

// sorted a and b, return sorted merged vector and diff vector of a
pair<vector<int>, vector<int>> merge_and_diff(const vector<int> &a, const vector<int> &b) {
    vector<int> merged, diff;
    uint i = 0, j = 0;
    while (i < a.size() && j < b.size()) {
        if (a[i] == b[j]) {
            merged.push_back(a[i++]);
            j++;
            continue;
        } else if (a[i] < b[j]) {
            merged.push_back(a[i++]);
        } else {
            diff.push_back(b[j]);
            merged.push_back(b[j++]);
        }
    }
    while (i < a.size()) {
        diff.push_back(a[i]);
        merged.push_back(a[i++]);
    }
    while (j < b.size()) {
        diff.push_back(b[j]);
        merged.push_back(b[j++]);
    }
    return {merged, diff};
}


vector<vector<int>> permutation_group(const vector<int> &g) {
    vector<vector<int>> res(g.size());
    for (uint i = 0; i < g.size(); ++i) {
        res[i].resize(g.size() - 1);
        copy(g.begin(), g.begin() + i, res[i].begin());
        copy(g.begin() + i + 1, g.end(), res[i].begin() + i);
    }
    return res;
}

unordered_map<int, vector<pair<vector<int>, Worker>>> grouping(int w_id, const vector<int> &reqs, uint max_size) {
    unordered_map<int, vector<pair<vector<int>, Worker>>> res;
    set<vector<int>> build;
    // 1. set up first layer requests
    for (const int &req : reqs) {
        Worker cur_worker = W[w_id].clone();
        insertion(cur_worker, req);
        res[1].push_back({{req}, cur_worker});
        build.insert({req});
    }
    // 2. set up further layers
    double fit;
    for (uint cur_layer = 2; cur_layer <= max_size; ++cur_layer) {
        for (uint i = 0; i < res[cur_layer - 1].size(); ++i) {
            for (uint j = i + 1; j < res[cur_layer - 1].size(); ++j) {
                auto merged = merge_and_diff(res[cur_layer - 1][i].first, res[cur_layer - 1][j].first);
                if (merged.first.size() != cur_layer || merged.second.size() != 1 ||
                    build.find(merged.first) != build.end())
                    break;
                // Check pairs exist or not
                auto perms = permutation_group(merged.first);
                if (any_of(perms.begin(), perms.end(), [build](const vector<int> &g) {
                    return build.find(g) == build.end();
                }))
                    continue;
                build.insert(merged.first);
                int r = merged.second[0];
                Worker w = res[cur_layer - 1][i].second.clone();
                try_insertion(w, r, fit);
                if (fit >= INF || fit == -1) continue;
                insertion(w, r);
                res[cur_layer].push_back({merged.first, w});
            }
        }
    }
    return res;
}


int main(int argc, char **args) {
    if (argc > 4) vertexFile = string(args[1]);
    if (argc > 4) edgeFile = string(args[2]);
    if (argc > 4) pathLabelFile = string(args[3]);
    if (argc > 4) orderFile = string(args[4]);
    if (argc > 5) dataFile = string(args[5]);
    if (argc > 6) requestFile = string(args[6]);
    if (argc > 7) CAPACITY_CONSTRAINT = atoi(args[7]);
    if (argc > 8) GLOBAL_DDL = atoi(args[8]);
    if (argc > 9) TAXI_SIZE = atoi(args[9]);
    if (argc > 10) kineticSize = atoi(args[10]);
    if (argc > 11) pool = new threadpool(atoi(args[11]));
    if (argc > 12) {
        desFile = string(args[12]);
//        freopen(desFile.c_str(), "w", stdout);
    }

    readInput();

    t1 = clock();
    initGrid();
    t2 = clock();

    pruneGDP(kineticSize);
    freeMemory();
    delete pool;
    fflush(stdout);

    return 0;
}