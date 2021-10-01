#include <bits/stdc++.h>
#include "../util.h"

using namespace std;

int batch_time = 5;
double theta = 200;
// 0=none; 1=ellipse; 2=ellipse+angle
int optimization_mode = 2;

/**
 * Composite structure to store temporary results.
 */
struct Car {
    Worker cur_path;
    // (rid,share_dist)
    map<int, double> cands{};
    set<int> reqs{};
};

Car get_best_k(const set<int> &reqs, int w_id, map<int, int> &degrees, map<int, set<int>> &graph);

/**
 * Calculate the angle between vectors (sa,sb) and (ea,eb).
 */
double get_line_angle(vertex sa, vertex sb, vertex ea, vertex eb) {
    pair<double, double> v1 = {ea.x - sb.x, ea.y - sb.y};
    if (sb.x == ea.x && sb.y == ea.y) v1 = {ea.x - sa.x, ea.y - sa.y};
    pair<double, double> v2 = {eb.x - sb.x, eb.y - sb.y};
    double module_v1 = sqrt(v1.first * v1.first + v1.second * v1.second);
    double module_v2 = sqrt(v2.first * v2.first + v2.second * v2.second);
    double cos_theta = (v1.first * v2.first + v1.second * v2.second) / (module_v1 * module_v2);
    return acos(fmin(fmax(cos_theta, -1.0), 1.0)) * 180 / 3.1415926;
}

/**
 * Enumerating request combinations with angle pruning.
 */
map<int, set<int>> grouping_by_angle(int *working_set, int size) {
    map<int, set<int>> res;
    double fit = INF;
    vector<vertex> &vertices = sssp->vertices;
    for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j) {
            if (i == j || res[i].find(j) != res[i].end()) continue; // filter replica
            double dist = sssp->shortestDistance(R[working_set[j]].s, R[working_set[i]].s);
            if ((dist > R[working_set[i]].ddl - CURRENT_TIME - R[working_set[i]].len
                 && dist > R[working_set[j]].ddl - CURRENT_TIME - R[working_set[j]].len))
                continue;
            double angle = get_line_angle(vertices[R[working_set[i]].s], vertices[R[working_set[j]].s],
                                          vertices[R[working_set[i]].e], vertices[R[working_set[j]].e]);
            if (abs(angle) >= theta) {
                continue;
            }
            fit = INF;
            Worker w;
            w.tim = CURRENT_TIME;
            w.num = 0;
            w.pid = R[working_set[i]].s;
            w.cap = CAPACITY_CONSTRAINT;
            insertion(w, working_set[i]);
            try_insertion(w, working_set[j], fit);
            if (fit == -1 || fit == INF) continue;
            res[working_set[i]].insert(working_set[j]);
            res[working_set[j]].insert(working_set[i]);
        }
    }
    return res;
}

/**
 * Enumerating request combinations with angle and ellipse pruning.
 */
map<int, set<int>> grouping_by_angle_with_ellipse(int *working_set, int size) {
    map<int, set<int>> res;
    double fit = INF;
    vector<vertex> &vertices = sssp->vertices;
    for (int i = 0; i < size; ++i) {
        int r_i = working_set[i];
        vector<int> cands = ellipseQuery(r_i, CURRENT_TIME);
        for (int r_j: cands) {
            if (r_i == r_j || res[r_i].find(r_j) != res[r_i].end()) continue; // filter replica
            double dist = sssp->shortestDistance(R[r_j].s, R[r_i].s);
            if ((dist > R[r_i].ddl - CURRENT_TIME - R[r_i].len
                 && dist > R[r_j].ddl - CURRENT_TIME - R[r_j].len))
                continue;
            double angle = get_line_angle(vertices[R[r_i].s], vertices[R[r_j].s], vertices[R[r_i].e],
                                          vertices[R[r_j].e]);
            if (abs(angle) >= theta) {
                continue;
            }
            fit = INF;
            Worker w;
            w.tim = CURRENT_TIME;
            w.num = 0;
            w.pid = R[r_i].s;
            w.cap = CAPACITY_CONSTRAINT;
            insertion(w, r_i);
            try_insertion(w, r_j, fit);
            if (fit == -1 || fit == INF) continue;
            res[r_i].insert(r_j);
            res[r_j].insert(r_i);
        }
    }
    return res;
}

/**
 * Retrieve all common neighbors of two requests in the shareability network.
 */
set<int> get_common_neighbors(set<int> s1, set<int> s2) {
    auto it1 = s1.begin(), it2 = s2.begin();
    set<int> common;
    while (it1 != s1.end() && it2 != s2.end()) {
        if (*it1 < *it2)it1++;
        else if (*it1 > *it2)it2++;
        else {
            common.insert(*it1);
            it1++;
            it2++;
        }
    }
    return common;
}

void main_loop() {
    int idx = 0, cur = 0;
    CURRENT_TIME = R[0].tim + batch_time;
    int accepted_size[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    int evicted = 0;
    set<int> working_set;
    while (pos <= n || !working_set.empty()) {
        // partition with time period `batch_time`
        while (pos <= n && R[pos].tim <= CURRENT_TIME) pos++;
        if (pos / 10000 > cur) {
            cur = pos / 10000;
            cout << pos << endl;
        }
        for (int i = 0; i < m; ++i) updateDriver(i, CURRENT_TIME);
        for (int i = idx; i < pos; ++i) {
            working_set.insert(i);
            insertRider(getGridID(R[i].s), i);
        }
        idx = pos;
        tsd = clock();

        map<int, int> degrees;
        int *requests = new int[working_set.size()];
        auto it = working_set.begin();
        for (int i = 0; i < working_set.size(); ++i, it++) requests[i] = *it;
        map<int, set<int>> graph;
        if (optimization_mode == 0) {
            graph = grouping_by_angle(requests, working_set.size());
        } else if (optimization_mode == 1) {
            theta = 60;
            graph = grouping_by_angle(requests, working_set.size());
        } else {
            theta = 60;
            graph = grouping_by_angle_with_ellipse(requests, working_set.size());
        }
        delete[] requests;
        for (const auto &node: graph) degrees[node.first] = node.second.size();

        // user prefer table
        double fit = INF;
        map<int, priority_queue<pair<double, int>>> user_prefer;
        for (auto r: working_set) {
            vector<int> car = single_search(R[r].s, R[r].ddl - CURRENT_TIME);
            priority_queue<pair<double, int>> valued_car;
            for (int cand_c: car) {
                try_insertion(W[cand_c], r, fit);
                if (fit >= INF || fit == -1) continue;
                valued_car.push({fit / R[r].len, cand_c});
            }
            if (valued_car.empty()) continue;
            user_prefer[r] = valued_car;
        }

        set<int> users;
        for (const auto &u: user_prefer) users.insert(u.first);

        map<int, Car> car_accepted;
        map<int, Car> pools;
        // user propose requests
        while (!users.empty()) {
            // propose round <v_id,[u_id]>
            set<int> updated_worker;
            for (int uid: users) {
                while (!user_prefer[uid].empty()) {
                    auto p = user_prefer[uid].top();
                    user_prefer[uid].pop();
                    if (pools[p.second].reqs.size() >= 20) continue;
                    pools[p.second].reqs.insert(uid);
                    updated_worker.insert(p.second);
                    break;
                }
            }
            // pickup round (only work for cands updated worker)
            for (const auto &w_id: updated_worker) {
                Car best_g = get_best_k(pools[w_id].reqs, w_id, degrees, graph);
                // Not accepted, rematching in next turn
                for (int r_id: pools[w_id].reqs) {
                    if (best_g.reqs.find(r_id) != best_g.reqs.end()) continue;
                    evicted++;
                    users.insert(r_id);
                }
                pools[w_id] = best_g;
                for (int u: best_g.reqs) users.erase(u);
            }
            for (auto it = users.begin(); it != users.end();) {
                if (!user_prefer[*it].empty()) it++;
                else users.erase(*it++);
            }
        }

        for (const auto &p: pools) {
            if (car_accepted.find(p.first) != car_accepted.end()) continue;
            car_accepted[p.first] = p.second;
        }
        for (const auto &car: car_accepted) {
            W[car.first] = car.second.cur_path;
            accepted_size[car.second.reqs.size() - 1]++;
            for (auto r: car.second.reqs) {
                working_set.erase(r);
                deleteRider(getGridID(R[r].s), r);
            }
        }

        // remove the expired tasks before the next trigger time.
        CURRENT_TIME += batch_time;
        for (auto it = working_set.begin(); it != working_set.end();) {
            int r = *it;
            if (R[r].ddl < (CURRENT_TIME + R[r].len) || CURRENT_TIME - R[r].tim > 300) {
                Penalty += R[r].len * R[r].pr;
                cnt++;
                deleteRider(getGridID(R[r].s), r);
                working_set.erase(it++);
            } else it++;
        }
    }

    for (int i = 0; i < m; ++i) {
        finishTaxi(i);
    }
    t3 = clock();
    dumpResult("SARD");
    for (int i: accepted_size) cout << i << " ";
    cout << endl;
    cout << "evicted: " << evicted << endl;
    cout << "ssspCount:" << ssspCounter << endl;
}

/**
 * Calculate the shareability loss of the combination.
 */
pair<double, double>
group_scoring(const vector<int> &reqs, Worker updated_worker, int w_id, const map<int, set<int>> &graph) {
    double score;
    double merged = updated_worker.reach.back() - W[w_id].tim;
    double origin = 0;
    set<int> req_set;
    for (auto r: reqs) req_set.insert(r);
    set<int> neighbors;
    score = -1;
    for (auto r: reqs) {
        origin += R[r].len;
        set<int> common_neig;
        bool is_init = false;
        for (auto v: reqs) {
            if (v == r || graph.find(v) == graph.end()) continue;
            if (!is_init) {
                common_neig = graph.find(v)->second;
                is_init = true;
            } else common_neig = get_common_neighbors(common_neig, graph.find(v)->second);
        }
        int n1 = common_neig.size();
        int n2 = graph.find(r) != graph.end() ? graph.find(r)->second.size() - reqs.size() - 1 : 0;
        int n3 = get_common_neighbors(common_neig, graph.find(r)->second).size();
        score = max((int) score, n1 + n2 - n3 - 1);
    }
    // isolate rider
    if (graph.find(reqs[0]) == graph.end() || graph.find(reqs[0])->second.size() == 0) return {INF, merged / origin};
        // single group with loss
    else if (reqs.size() == 1) return {INF / 2, merged / origin};
    return {score, merged / origin};
}

/**
 * Select best group from candidate groups.
 */
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

/**
 * C_n^(n-1).
 * For pruning: subgroup must exist in previous enumeration.
 */
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
 * Retrieve the maximum degree request `max_r` in shareability graph among `group`.
 * @return (group-{max_r}, max_r)
 */
pair<vector<int>, int> split_max_deg(const vector<int> &group, map<int, int> &degrees) {
    int idx = 0, max_deg = degrees[0], len_group = group.size();
    for (uint i = 1; i < len_group; i++) {
        if (degrees[group[i]] < max_deg) continue;
        idx = i;
        max_deg = degrees[group[i]];
    }
    vector<int> res(len_group - 1);
    copy(group.begin(), group.begin() + idx, res.begin());
    copy(group.begin() + idx + 1, group.end(), res.begin() + idx);
    return {res, group[idx]};
}

/**
 * Check whether the requests `reqs` forms clique in the shareability graph
 */
bool check_clique(vector<int> reqs, map<int, set<int>> &graph) {
    for (int i = 0; i < reqs.size(); i++) {
        auto it = graph.find(reqs[i]);
        for (int j = i + 1; j < reqs.size(); j++) {
            if (it == graph.end() || it->second.find(reqs[j]) == it->second.end())
                return false;
        }
    }
    return true;
}

/**
 * Combination enumeration
 */
unordered_map<int, map<vector<int>, Worker>>
grouping(int w_id, const vector<int> &reqs, map<int, int> &degrees, map<int, set<int>> &graph) {
    unordered_map<int, map<vector<int>, Worker>> res;
    set<vector<int>> build;
    // 1. set up first layer requests
    for (const int &req: reqs) {
        Worker cur_worker = W[w_id].clone();
        insertion(cur_worker, req);
        res[1].insert({{req}, cur_worker});
        build.insert({req});
    }
    // 2. set up further layers
    double fit;
    vector<vector<int>> last_layer;
    for (uint cur_layer = 2; cur_layer <= CAPACITY_CONSTRAINT; ++cur_layer) {
        last_layer.clear();
        for (const auto &it: res[cur_layer - 1]) last_layer.push_back(it.first);
        for (uint i = 0; i < last_layer.size(); ++i) {
            for (uint j = i + 1; j < last_layer.size(); ++j) {
                auto merged = merge_and_diff(last_layer[i], last_layer[j]);
                if (merged.first.size() != cur_layer || merged.second.size() != 1 ||
                    build.find(merged.first) != build.end())
                    continue;
                if (!check_clique(merged.first, graph))continue;
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

/**
 * Retrieve the combination with the least shareability loss in the combination tree.
 */
Car get_best_k(const set<int> &reqs, int w_id, map<int, int> &degrees, map<int, set<int>> &graph) {
    vector<int> reqs_vec;
    for (int r: reqs) reqs_vec.push_back(r);
    auto groups = grouping(w_id, reqs_vec, degrees, graph);
    map<vector<int>, pair<pair<double, double>, Worker>> process_seq;
    for (const auto &group: groups) {
        for (const auto &it: group.second) {
            auto scores = group_scoring(it.first, it.second, w_id, graph);
            process_seq.insert({it.first, {scores, it.second}});
        }
    }
    auto chose = get_best_group(process_seq);

    Car best;
    for (int r_id: chose.first) best.reqs.insert(r_id);
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
    if (argc > 12) batch_time = atoi(args[12]);
    if (argc > 13) optimization_mode = atoi(args[13]);
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