
#include <bits/stdc++.h>

using namespace std;

#include "util.h"
#include "mcmf.h"

int batch_time = 5;
auto pool = new threadpool(20);

struct Car {
    Worker cur_path;
    // (rid,share_dist)
    map<int, double> cands;
    set<int> reqs;
};

set<int> served_req;

Car get_best_k(const set<int> &groups, int w_id, map<int, int> &degrees, map<int, set<int>> &graph);

double get_line_angle(double k1, double k2) {
    double tan_k = (k2 - k1) / (1 + k2 * k1); //直线夹角正切值
    return atan(tan_k) * 180.0 / 3.1415926;
}

double euclid(int a, int b) {
    vector<vertex> &vertices = sssp->vertices;
    return sqrt((vertices[a].x - vertices[b].x) * (vertices[a].x - vertices[b].x) +
                (vertices[a].y - vertices[b].y) * (vertices[a].y - vertices[b].y)) / MAX_speed;
}

map<int, set<int>> grouping_by_angle(int *working_set, int size) {
    map<int, set<int>> res;
    vector<vertex> &vertices = sssp->vertices;
    for (int i = 0; i < size; ++i) {
        double k1 = (vertices[R[working_set[i]].e].y - vertices[R[working_set[i]].s].y)
                    / (vertices[R[working_set[i]].e].x - vertices[R[working_set[i]].s].x);
        for (int j = i + 1; j < size; ++j) {
            double dist = euclid(R[working_set[j]].s, R[working_set[i]].s);
            if ((dist > R[working_set[i]].ddl - CURRENT_TIME - R[working_set[i]].len
                 && dist > R[working_set[j]].ddl - CURRENT_TIME - R[working_set[j]].len))
                continue;
            double angle = get_line_angle(k1, (vertices[R[working_set[j]].e].y - vertices[R[working_set[j]].s].y) /
                                              (vertices[R[working_set[j]].e].x - vertices[R[working_set[j]].s].x));
            if (abs(angle) > 45) {
                continue;
            }
            res[working_set[i]].insert(working_set[j]);
            res[working_set[j]].insert(working_set[i]);
        }
    }
    return res;
}

double delta;

void main_loop() {
    int idx = 0, cur = 0;
    CURRENT_TIME = R[0].tim + batch_time;
    int accepted_size[] = {0, 0, 0, 0};
    int evicted = 0;
    set<int> working_set;
    while (pos <= n) {
        if (idx == pos && pos > n)break;
        if (idx == pos && pos < n && R[pos].tim > CURRENT_TIME) {
            CURRENT_TIME += batch_time;
            continue;
        }
        if (pos < n && R[pos].tim <= CURRENT_TIME) {
            pos++;
            continue;
        }

        if (pos / 10000 > cur) {
            cur = pos / 10000;
            cout << pos << endl;
        }
        for (int i = 0; i < m; ++i) updateDriver(i, CURRENT_TIME);
        for (int i = idx; i < pos; ++i) working_set.insert(i);
        idx = pos;
        if (pos == n)pos++; // For quit
        tsd = clock();

        map<int, int> degrees;
        int *requests = new int[working_set.size()];
        auto it = working_set.begin();
        for (int i = 0; i < working_set.size(); ++i) {
            requests[i] = *it;
            it++;
        }
        auto graph = grouping_by_angle(requests, working_set.size());
        delete[] requests;
        int max_deg = 1;
        for (const auto &node : graph) {
            degrees[node.first] = node.second.size();
            int size = node.second.size();
            max_deg = max(max_deg, size);
        }
        int *deg_count = new int[max_deg + 1];
        int small = 0, large = 0;
        for (int i = 0; i <= max_deg; ++i) deg_count[i] = 0;
        for (const auto &node : graph) {
            deg_count[node.second.size()]++;
            if (node.second.size() > 15)large++;
            else small++;
        }
        deg_count[0] = working_set.size() - graph.size();
        if (1.0 * large / working_set.size() > 0.05)
            cout << setprecision(5) << 1.0 * large / working_set.size() << endl;
//        for (int i = 0; i <= max_deg; ++i) cout << deg_count[i] << " ";
//        cout << endl;
        delete[] deg_count;

        // user prefer table
        double fit = INF;
        map<int, priority_queue<pair<double, int>>> user_prefer;
        for (auto r:working_set) {
            vector<int> car = single_search(R[r].s, R[r].ddl - CURRENT_TIME);
            priority_queue<pair<double, int>> valued_car;
            for (int cand_c : car) {
                try_insertion(W[cand_c], r, fit);
                if (fit >= INF || fit == -1) continue;
                valued_car.push({fit / R[r].len, cand_c});
            }
            if (valued_car.empty()) continue;
            user_prefer[r] = valued_car;
        }

        set<int> users;
        for (const auto &u:user_prefer) users.insert(u.first);

        map<int, Car> car_accepted;
        map<int, Car> pools;
        // user propose requests
        while (!users.empty()) {
            // propose round <v_id,[u_id]>
            set<int> updated_worker;
            for (int uid:users) {
                while (!user_prefer[uid].empty()) {
                    auto p = user_prefer[uid].top();
                    user_prefer[uid].pop();
                    // The car already finished pickup
//                    if (car_accepted.find(p.second) != car_accepted.end()) continue;
                    pools[p.second].reqs.insert(uid);
                    updated_worker.insert(p.second);
                    break;
                }
            }
            // pickup round (only work for cands updated worker)
            for (const auto &w_id:updated_worker) {
                Car best_g = get_best_k(pools[w_id].reqs, w_id, degrees, graph);
                // Not accepted, rematching in next turn
                for (int r_id: pools[w_id].reqs) {
                    if (best_g.reqs.find(r_id) == best_g.reqs.end()) {
                        evicted++;
                        users.insert(r_id);
                    }
                }
                pools[w_id] = best_g;
                for (int u:best_g.reqs) users.erase(u);
//                int sum_cap = 0;
//                for (int r:best_g.reqs) sum_cap += R[r].com;
//                if (sum_cap >= W[w_id].cap) {
//                    car_accepted[w_id] = best_g;
//                }
            }
            for (auto it = users.begin(); it != users.end();) {
                if (!user_prefer[*it].empty()) it++;
                else users.erase(*it++);
            }
        }

        for (const auto &p:pools) {
//            if (car_accepted.find(p.first) != car_accepted.end()) continue;
            car_accepted[p.first] = p.second;
        }

        for (const auto &car:car_accepted) {
            W[car.first] = car.second.cur_path;
            accepted_size[car.second.reqs.size() - 1]++;
            for (auto r:car.second.reqs) {
                working_set.erase(r);
                if (served_req.find(r) != served_req.end()) {
                    cout << "Repeated:" << r << endl;
                } else served_req.insert(r);
            }
        }

        for (auto it = working_set.begin(); it != working_set.end();) {
            int r = *it;
            if (R[r].ddl < (CURRENT_TIME + R[r].len)) {
                Penalty += R[r].len * R[r].pr;
                cnt++;
                working_set.erase(it++);
            } else it++;
        }
    }

    for (int i = 0; i < m; ++i) {
        finishTaxi(i);
    }
    t3 = clock();
    dumpResult("STEADY");
    for (int i : accepted_size) cout << i << " ";
    cout << endl;
    cout << "evicted: " << evicted << endl;
}

pair<double, double>
group_scoring(const vector<int> &reqs, Worker updated_worker, int w_id, const map<int, int> &degrees,
              const map<int, set<int>> &graph) {
    double score = 0;
    double merged = updated_worker.reach.back() - W[w_id].tim;
//    double merged = updated_worker.reach.back() - (W[w_id].reach.empty() ? CURRENT_TIME : W[w_id].reach.back());
    double origin = 0;
    set<int> neighbors;
    for (auto r:reqs) neighbors.insert(r);
    for (auto r: reqs) {
        origin += R[r].len;
//        score += degrees.find(r)->second;
//        for (auto neig:graph.find(r)->second) neighbors.insert(neig);
        for (auto neig:graph.find(r)->second) {
            bool flag = false;
            for (auto hop2:graph.find(neig)->second) {
                if (neighbors.find(hop2) != neighbors.end())continue;
                flag = true;
//                neighbors.insert(hop2);
            }
            if (flag) score++;
        }
    }
    if (reqs.size() == 1 && degrees.find(reqs[0])->second == 0)return {INF, merged / origin};
    if (reqs.size() == 1) return {1 + score, degrees.find(reqs[0])->second};
    return {score / reqs.size(), merged / origin};
    for (auto r:reqs) neighbors.erase(r);
    return {neighbors.size() / reqs.size(), merged / origin};
}

pair<vector<int>, pair<pair<double, double>, Worker>> get_best_group(
        map<vector<int>, pair<pair<double, double>, Worker>> groups) {
    auto best_it = groups.begin();
    for (auto it = ++groups.begin(); it != groups.end(); it++) {
        auto score = it->second.first;
        if (abs(score.first - best_it->second.first.first) < EPS) {
            if (abs(score.second - best_it->second.first.second) < EPS) {
                if (it->first.size() > best_it->first.size()) best_it = it;
            } else if (score.second < best_it->second.first.second) best_it = it;
        } else if (score.first < best_it->second.first.first) best_it = it;
    }
    return *best_it;
}

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

// sorted a and b, return sorted merged vector and diff vector of a
pair<vector<int>, vector<int>>
merge_and_diff_with_deg(const vector<int> &a, const vector<int> &b, map<int, int> &degrees) {
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
    vector<pair<int, int>> reqs_with_degree;
    for (auto r:merged)reqs_with_degree.emplace_back(degrees[r], r);
    sort(reqs_with_degree.begin(), reqs_with_degree.end());
    vector<int> group;
    for (auto r: reqs_with_degree) group.push_back(r.second);
    return {group, diff};
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

unordered_map<int, vector<pair<vector<int>, Worker>>> grouping(int w_id, const vector<int> &reqs) {
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
    for (uint cur_layer = 2;; ++cur_layer) {
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
                fit = INF;
                try_insertion(w, r, fit);
                if (fit >= INF || fit == -1) continue;
                insertion(w, r);
                res[cur_layer].push_back({merged.first, w});
            }
        }
        if (res[cur_layer].empty()) break;
    }
    return res;
}


pair<vector<int>, int> split_max_deg(const vector<int> &group, map<int, int> &degrees) {
    int r = group[0];
    double d = degrees[r];
    for (auto r_id:group) {
        if (degrees[r_id] < d)continue;
        r = r_id;
        d = degrees[r_id];
    }
    vector<int> res;
    for (int req:group) {
        if (req == r)continue;
        res.push_back(req);
    }
    return {res, r};
}

unordered_map<int, map<vector<int>, Worker>>
grouping_opt(int w_id, const vector<int> &reqs, map<int, int> &degrees) {
    unordered_map<int, map<vector<int>, Worker>> res;
    set<vector<int>> build;
    // 1. set up first layer requests
    for (const int &req : reqs) {
        Worker cur_worker = W[w_id].clone();
        insertion(cur_worker, req);
        res[1].insert({{req}, cur_worker});
        build.insert({req});
    }
    // 2. set up further layers
    double fit;
    vector<vector<int>> last_layer;
    for (uint cur_layer = 2;; ++cur_layer) {
        last_layer.clear();
        for (const auto &it:res[cur_layer - 1]) last_layer.push_back(it.first);
        for (uint i = 0; i < last_layer.size(); ++i) {
            for (uint j = i + 1; j < last_layer.size(); ++j) {
                auto merged = merge_and_diff_with_deg(last_layer[i], last_layer[j], degrees);
                if (merged.first.size() != cur_layer || merged.second.size() != 1 ||
                    build.find(merged.first) != build.end())
                    break;
                // Check pairs exist or not
                auto perms = permutation_group(merged.first);
                if (any_of(perms.begin(), perms.end(), [build](const vector<int> &g) {
                    return build.find(g) == build.end();
                }))
                    continue;
                auto plan = split_max_deg(merged.first, degrees);
                int r = plan.second;
                Worker w = res[cur_layer - 1][plan.first].clone();
                try_insertion(w, r, fit);
                if (fit >= INF || fit == -1) continue;
                build.insert(merged.first);
                insertion(w, r);
                res[cur_layer].insert({merged.first, w});
            }
        }
        if (res[cur_layer].empty()) break;
    }
    return res;
}

Car get_best_k(const set<int> &reqs, int w_id, map<int, int> &degrees, map<int, set<int>> &graph) {
    vector<int> reqs_vec;
    for (int r: reqs) reqs_vec.push_back(r);
//    auto groups = grouping(w_id, reqs_vec);
    auto groups = grouping_opt(w_id, reqs_vec, degrees);
    map<vector<int>, pair<pair<double, double>, Worker>> process_seq;
    for (const auto &group:groups) {
        for (const auto &it : group.second) {
            auto scores = group_scoring(it.first, it.second, w_id, degrees, graph);
            process_seq.insert({it.first, {scores, it.second}});
        }
    }
    auto best_g = get_best_group(process_seq);
    auto chose = best_g;
    for (const auto &item:process_seq) {
        auto score = item.second.first;
        if (abs(best_g.second.first.first - score.first) < delta
            && score.second < chose.second.first.second) {
            chose = item;
        }
    }

    Car best;
    for (int r_id:chose.first) best.reqs.insert(r_id);
    best.cur_path = chose.second.second;
    return best;
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
    if (argc > 12) delta = atof(args[12]);
    if (argc > 13) batch_time = atoi(args[13]);
    if (argc > 14) {
        desFile = string(args[14]);
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