#include <bits/stdc++.h>
#include "util.h"

using namespace std;

int batch_time = 5;

pair<vector<int>, Worker> get_best_u(const set<int> &reqs, int w_id);

void main_loop() {
    int idx = 0, cur = 0;
    CURRENT_TIME = R[0].tim + batch_time;
    int rand_car[m];
    int accepted_size[] = {0, 0, 0, 0};
    for (int i = 0; i < m; ++i) rand_car[i] = i;
    set<int> working_set;
    while (pos <= n || !working_set.empty()) {
        while (pos <= n && R[pos].tim <= CURRENT_TIME) pos++;

        if (pos / 1000 > cur) {
            cur = pos / 1000;
            cout << pos << endl;
        }
        for (int i = 0; i < m; ++i) updateDriver(i, CURRENT_TIME);
        for (int i = idx; i < pos; ++i) working_set.insert(i);
        idx = pos;
        tsd = clock();

        // 1. random pickup vehicle
        random_shuffle(rand_car, rand_car + m);
        for (auto w_id : rand_car) {
//            if (!W[w_id].S.empty() || W[w_id].tim > CURRENT_TIME)continue;
            // 2. filtering infeasible riders
            set<int> cands;
            for (int r: working_set) {
                double dist = sssp->shortestDistance(sssp->vertices[W[w_id].pid].id, sssp->vertices[R[r].s].id);
                if (W[w_id].tim + dist + R[r].len <= R[r].ddl && R[r].com <= CAPACITY_CONSTRAINT) cands.insert(r);
            }
            if (cands.empty()) continue;
            auto best_u = get_best_u(cands, w_id);
            if (best_u.first.empty()) continue;
            // update driver schedule with fastest route
            W[w_id] = best_u.second;
            accepted_size[best_u.first.size() - 1]++;
            for (auto r:best_u.first) {
                working_set.erase(r);
//                cout << R[r].len * GLOBAL_DDL << endl;
            }
        }

        // Remove expired riders before next round time
        CURRENT_TIME += batch_time;
        for (auto it = working_set.begin(); it != working_set.end();) {
            int r = *it;
            if (R[r].ddl < (CURRENT_TIME + R[r].len) || CURRENT_TIME - R[r].tim > 300) {
                if (R[r].len < INF_WEIGHT) {
                    Penalty += R[r].len * R[r].pr;
                }
                cnt++;
                working_set.erase(it++);
            } else it++;
        }
    }

    for (int i = 0; i < m; ++i) {
        finishTaxi(i);
    }
    t3 = clock();
    dumpResult("GAS");
    for (int i : accepted_size) cout << i << " ";
    cout << endl;
}

// rider group && candidate routes
pair<vector<int>, vector<Worker>> get_best_group(
        map<vector<int>, pair<double, vector<Worker>>> groups) {
    auto best_it = groups.begin();
    for (auto it = ++groups.begin(); it != groups.end(); it++) {
        auto score = it->second.first;
        if (score > best_it->second.first) best_it = it;
    }
    return {best_it->first, best_it->second.second};
}

// sorted a and b, return sorted merged vector and diff vector of a
pair<vector<int>, vector<int>> merge_and_diff(const vector<int> &a, const vector<int> &b) {
    vector<int> merged, diff;
    uint len_a = a.size(), len_b = b.size();
    merged.reserve(len_b + len_a);
    diff.reserve(max(len_b, len_a));
    uint i = 0, j = 0;
    while (i < len_a && j < len_b) {
        if (a[i] == b[j]) {
            merged.push_back(a[i++]);
            j++;
        } else if (a[i] < b[j]) {
            merged.push_back(a[i++]);
        } else {
            diff.push_back(b[j]);
            merged.push_back(b[j++]);
        }
    }
    while (i < len_a) merged.push_back(a[i++]);
    while (j < len_a) {
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

/**
 * Note that there are at most 2^capacity nodes in additive tree,
 * which will not take too much time to find the best group.
 * So that we return a plain structure instead of a tree.
 *
 * @param w_id grouping worker id
 * @param reqs candidate riders
 * @return map<layer_id, layer_nodes>
 *        layer_nodes construct by two parts: group riders && candidate routes
 */
unordered_map<int, vector<pair<vector<int>, vector<Worker>>>>
construct_additive_tree(int w_id, const vector<int> &reqs) {
    unordered_map<int, vector<pair<vector<int>, vector<Worker>>>> res;
    set<vector<int>> build;
    double fit;
    // 1. set up first layer requests
    for (const int &req : reqs) {
        Worker cur_worker = W[w_id].clone();
        try_insertion(cur_worker, req, fit);
        if (fit < 0 || fit >= INF) continue;
        insertion(cur_worker, req);
        res[1].push_back({{req},
                          {cur_worker}});
        build.insert({req});
    }
    // 2. set up further layers
    for (uint cur_layer = 2; cur_layer <= CAPACITY_CONSTRAINT; ++cur_layer) {
        for (uint i = 0; i < res[cur_layer - 1].size(); ++i) {
            for (uint j = i + 1; j < res[cur_layer - 1].size(); ++j) {
                auto merged = merge_and_diff(res[cur_layer - 1][i].first, res[cur_layer - 1][j].first);
                if (merged.first.size() != cur_layer || merged.second.size() != 1 ||
                    build.find(merged.first) != build.end())
                    continue;
                // Check any sub nodes exist or not
                auto perms = permutation_group(merged.first);
                if (any_of(perms.begin(), perms.end(), [build](const vector<int> &g) {
                    return build.find(g) == build.end();
                }))
                    continue;
                build.insert(merged.first);
                int r = merged.second[0];
                vector<Worker> tmp_paths;
                for (const auto &w:res[cur_layer - 1][i].second) {
                    try_insertion(w, r, fit);
                    if (fit < 0 || fit >= INF) continue;
                    auto updated_paths = insertion_all(w, r);
                    for (const auto &path:updated_paths)tmp_paths.push_back(path);
                }
                if (tmp_paths.empty())continue;
                res[cur_layer].push_back({merged.first, tmp_paths});
            }
        }
        if (res[cur_layer].empty()) break;
    }
    return res;
}

pair<vector<int>, Worker> get_best_u(const set<int> &reqs, int w_id) {
    vector<int> reqs_vec;
    for (int r: reqs) reqs_vec.push_back(r);
    // 2. construct additive tree
    auto groups = construct_additive_tree(w_id, reqs_vec);
    // 3. retrieve best (most profitable sum_length) group node
    map<vector<int>, pair<double, vector<Worker>>> process_seq;
    for (const auto &group:groups) {
        for (const auto &it : group.second) {
            double scores = 0;
            for (auto r: it.first) scores += R[r].len;
            process_seq.insert({it.first, {scores, it.second}});
        }
    }
    if (process_seq.empty())
        return {{},
                {}};
    // 4. retrieve best route
    auto best_g = get_best_group(process_seq);
    Worker real_w;
    int opt = INF_WEIGHT;
    for (auto w:best_g.second) {
        int val = w.reach.back();
        if (val < opt) real_w = w;
        opt = min(opt, val);
    }
    return {best_g.first, real_w};
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
    if (argc > 12) batch_time = atoi(args[12]);
    if (argc > 13) {
        desFile = string(args[13]);
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