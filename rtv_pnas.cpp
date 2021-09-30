
#include <bits/stdc++.h>
#include <glpk.h>
#include "util.h"

using namespace std;

int batch_time = 5;

unordered_map<int, vector<pair<vector<int>, vector<Worker>>>>
grouping(int w_id, const vector<int> &reqs, uint max_size);

int accepted_size[] = {0, 0, 0, 0};

void main_loop() {
    int idx = 0, cur = 0;
    CURRENT_TIME = R[0].tim + batch_time;
    set<int> working_set;
    while (pos <= n||!working_set.empty()) {
        while (pos <= n && R[pos].tim <= CURRENT_TIME) pos++;

        if (pos / 1000 > cur) {
            cur = pos / 1000;
            cout << pos << endl;
        }
        for (int i = 0; i < m; ++i) updateDriver(i, CURRENT_TIME);
        for (int i = idx; i < pos; ++i) working_set.insert(i);
        idx = pos;
        tsd = clock();

        map<int, map<int, Worker>> worker2trips;
        map<vector<int>, int> trip2id;
        map<int, vector<int>> id2trip;
        int cur_trip_id = 0;
        for (int w_id = 0; w_id < m; w_id++) {
            set<int> cands;
            for (int r: working_set) {
                double dist = sssp->shortestDistance(sssp->vertices[W[w_id].pid].id, sssp->vertices[R[r].s].id);
                if (W[w_id].tim + dist + R[r].len <= R[r].ddl && R[r].com <= CAPACITY_CONSTRAINT) cands.insert(r);
            }
            if (cands.empty()) continue;
            vector<int> reqs_vec;
            for (int r: cands) reqs_vec.push_back(r);
            auto groups = grouping(w_id, reqs_vec, CAPACITY_CONSTRAINT);

            for (const auto &group:groups) {
                for (const auto &it : group.second) {
                    if (trip2id.find(it.first) == trip2id.end()) {
                        trip2id.insert({it.first, cur_trip_id});
                        id2trip.insert({cur_trip_id, it.first});
                        cur_trip_id++;
                    }
                    int trip_id = trip2id.find(it.first)->second;
                    Worker best_w;
                    int opt = INF_WEIGHT;
                    for (auto w:it.second) {
                        int val = w.S.back();
                        if (val < opt) best_w = w;
                        opt = min(opt, val);
                    }
                    worker2trips[w_id].insert({trip_id, best_w});
                }
            }
        }


        /* Objective: c11x11 + c12x12 + ... + cijxij + y1 + y2 + ... + yn
       *     For cijxij, ij are from vt_edge
       *     For yn, n is from waiting_customers()
       * Constraints:
       *     1) Each vehicle serves 0 or 1 trips
       *         xi1 + xi2 + ... + xin <= 1, for all i
       *     2) Each customer is either served or unserved
       *         yn + xij == 1, for all n, where j are trips containing n, and
       *                                         i are the vehicles serve them
       * Properties:
       *     - Total terms in the objective == number of columns
       *       == all VehicleId, SharedTripId pairs in vt_edge, plus |customers| */
        glp_prob *mip;
        mip = glp_create_prob();
        glp_set_prob_name(mip, ("mip (t=" + to_string(CURRENT_TIME)).c_str());
        glp_set_obj_dir(mip, GLP_MIN);
        glp_term_out(GLP_OFF);
        //nvted: 好像是所有Trip的乘客数量之和
        int nvted = 0;
        for (const auto &p:worker2trips)nvted += p.second.size();
        int ncol = nvted + working_set.size();
        glp_add_cols(mip, ncol);
        /*     - Total rows == # of vehicles (size of vt_edge), plus |customers| */
//        int vt_edge = 0;
//        for (const auto &p:worker2trips)vt_edge += p.second.size();
        int nrow = worker2trips.size() + working_set.size();
        glp_add_rows(mip, nrow);

        /* Map col_idx to the vehicle/trip edge */
        map<size_t, pair<int, int>> colmap{};
        { /* Populate the coefficients in the objective */
            size_t col_idx = 0;
            for (const auto &kv : worker2trips) {
                int vehl_id = kv.first;
                for (const auto &kv2 : kv.second) {
                    int shtrip_id = kv2.first;
                    col_idx++;
                    colmap[col_idx] = {vehl_id, shtrip_id};
                    glp_set_obj_coef(mip, col_idx, (int) (kv2.second.reach.back() - kv2.second.tim));
                    glp_set_col_kind(mip, col_idx, GLP_BV);
                    glp_set_col_name(mip, col_idx,
                                     ("x_" + to_string(vehl_id) + "_" + to_string(shtrip_id)).c_str());
                }
            }
            for (const auto &cust_id : working_set) {
                col_idx++;
                colmap[col_idx] = {cust_id, -1};
//                int penalty = (unassign_penalty > 0 ? unassign_penalty : R[cust_id].len);
                glp_set_obj_coef(mip, col_idx, GLOBAL_PENT * R[cust_id].len);
                glp_set_col_kind(mip, col_idx, GLP_BV);
                glp_set_col_name(mip, col_idx,
                                 ("y_" + to_string(cust_id)).c_str());
            }
        }  // end populate coefficients

        /* Populate cells of the constraints matrix */
        std::vector<int> ia(ncol * nrow + 1);  // <-- 1-indexed
        std::vector<int> ja(ncol * nrow + 1);
        std::vector<double> ar(ncol * nrow + 1);
        /* For constraint 1) each vehicle serves 0 or 1 trips:
         *     ia[cel_idx] = row_idx
         *     ja[cel_idx] = col_idx
         *     ar[cel_idx] = 1 if the i of col matches the vehl of row_idx,
         *                   0 otherwise */
        size_t row_idx = 0;
        size_t cel_idx = 0;
        for (const auto &kv : worker2trips) {
            row_idx++;
            glp_set_row_bnds(mip, row_idx, GLP_UP, 0.0, 1.0);
            glp_set_row_name(mip, row_idx, ("v" + to_string(kv.first)).c_str());
            for (const auto &kv2 : colmap) {
                cel_idx++;
                ia[cel_idx] = row_idx;
                ja[cel_idx] = kv2.first;
                ar[cel_idx] = (kv.first == kv2.second.first ? 1 : 0);
            }
        }
        /* For constraint 2) each customer is either served or unserved:
         *     ia[cel_idx] = row_idx
         *     ja[cel_idx] = col_idx
         *     ar[cel_idx] = 1 if the j of col contains the cust of row_idx,
         *                   1 if the i of col equals cust.id(),
         *                   0 otherwise */
        for (const auto &cust_id : working_set) {
            row_idx++;
            glp_set_row_bnds(mip, row_idx, GLP_FX, 1.0, 1.0);
            glp_set_row_name(mip, row_idx, ("c" + to_string(cust_id)).c_str());
            for (const auto &kv2 : colmap) {
                cel_idx++;
                ia[cel_idx] = row_idx;
                ja[cel_idx] = kv2.first;
                if (kv2.second.first == cust_id) ar[cel_idx] = 1;
                else if (kv2.second.second == -1) ar[cel_idx] = 0;
                else {
                    ar[cel_idx] = 0;
                    for (const auto &cust2 : id2trip.at(kv2.second.second))
                        if (cust2 == cust_id) {
                            ar[cel_idx] = 1;
                            break;
                        }
                }
            }
        }
        // end populate constraints matrix
        glp_load_matrix(mip, cel_idx, ia.data(), ja.data(), ar.data());
        glp_iocp cparams;
        glp_init_iocp(&cparams);
        cparams.presolve = GLP_ON;

        /* Heuristics */
        cparams.tm_lim = 15 * 1000;  // set time limit to 15 sec
        cparams.mip_gap = 0.001;     // set optimality gap to 0.1%

        // int rc = glp_intopt(mip, &cparams);  // <-- solve!
        glp_intopt(mip, &cparams);  // <-- solve!

        /* Extract assignments from the results and commit to database */
        for (int i = 1; i <= ncol; ++i) {
            if (colmap[i].second == -1)
                continue;
            /* If the binary var for any other column is 1, then commit the
             * assignment. */
            if (glp_mip_col_val(mip, i) == 1) {
                W[colmap[i].first] = worker2trips[colmap[i].first][colmap[i].second];
                accepted_size[id2trip.at(colmap[i].second).size() - 1]++;
                for (const int &cust : id2trip.at(colmap[i].second)) {
                    working_set.erase(cust);
                }
            }
        }

        glp_delete_prob(mip);

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
    dumpResult("rtv");
    for (int i : accepted_size) cout << i << " ";
    cout << endl;
}


pair<vector<int>, pair<double, vector<Worker>>> get_best_group(
        map<vector<int>, pair<double, vector<Worker>>> groups) {
    auto best_it = groups.begin();
    for (auto it = ++groups.begin(); it != groups.end(); it++) {
        auto score = it->second.first;
        if (score < best_it->second.first) best_it = it;
    }
    return *best_it;
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

unordered_map<int, vector<pair<vector<int>, vector<Worker>>>>
grouping(int w_id, const vector<int> &reqs, uint max_size) {
    unordered_map<int, vector<pair<vector<int>, vector<Worker>>>> res;
    set<vector<int>> build;
    double fit;
    // 1. set up first layer requests
    for (const int &req : reqs) {
        Worker cur_worker = W[w_id].clone();
        try_insertion(cur_worker, req, fit);
        if (fit == -1 || fit >= INF) continue;
        insertion(cur_worker, req);
        res[1].push_back({{req},
                          {cur_worker}});
        build.insert({req});
    }
    // 2. set up further layers
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
                vector<Worker> tmp_paths;
                for (const auto &w:res[cur_layer - 1][i].second) {
                    try_insertion(w, r, fit);
                    if (fit == -1 || fit >= INF) continue;
                    auto updated_paths = insertion_all(w, r);
                    for (const auto &path:updated_paths)tmp_paths.push_back(path);
                }
                if (tmp_paths.empty())continue;
                res[cur_layer].push_back({merged.first, tmp_paths});
            }
        }
        if (res[cur_layer].empty()) break;
    }

//    for (uint cur_layer = 2; cur_layer <= max_size; ++cur_layer) {
//        vector<pair<int, int>> tasks;
//        for (uint i = 0; i < res[cur_layer - 1].size(); ++i) {
//            for (uint j = i + 1; j < res[cur_layer - 1].size(); ++j) {
//                auto merged = merge_and_diff(res[cur_layer - 1][i].first, res[cur_layer - 1][j].first);
//                if (merged.first.size() != cur_layer || merged.second.size() != 1 ||
//                    build.find(merged.first) != build.end())
//                    continue;
//                tasks.emplace_back(i, j);
//            }
//        }
//        int batch_size = sqrt(tasks.size());
//        vector<future<map<vector<int>, vector<Worker>>>> futures;
//        for (uint j = 0; j < tasks.size(); ++j) {
//            vector<pair<int, int>> batch;
//            for (int k = 0; k < batch_size && j < tasks.size(); ++k) batch.push_back(tasks[j++]);
//            futures.push_back(pool->commit([batch, &res, cur_layer, &build] {
//                double fit;
//                map<vector<int>, vector<Worker>> tmp_res;
//                for (auto p : batch) {
//                    int i = p.first, j = p.second;
//                    auto merged = merge_and_diff(res[cur_layer - 1][i].first, res[cur_layer - 1][j].first);
//                    // Check pairs exist or not
//                    auto perms = permutation_group(merged.first);
//                    if (any_of(perms.begin(), perms.end(), [build](const vector<int> &g) {
//                        return build.find(g) == build.end();
//                    }))
//                        continue;
//                    int r = merged.second[0];
//                    vector<Worker> tmp_paths;
//                    for (const auto &w:res[cur_layer - 1][i].second) {
//                        try_insertion(w, r, fit);
//                        if (fit == -1 || fit >= INF) continue;
//                        auto updated_paths = insertion_all(w, r);
//                        for (const auto &path:updated_paths)tmp_paths.push_back(path);
//                    }
//                    if (tmp_paths.empty())continue;
//                    tmp_res[merged.first] = tmp_paths;
//                }
//                return tmp_res;
//            }));
//        }
//
//        vector<map<vector<int>, vector<Worker>>> thread_res;
//        for (auto &future : futures) thread_res.push_back(future.get());
//        for (const auto &tmp_res:thread_res) {
//            for (const auto &it: tmp_res) {
//                if (build.find(it.first) != build.end()) continue;
//                res[cur_layer].push_back({it.first, it.second});
//                build.insert(it.first);
//            }
//        }
//    }

    return res;
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