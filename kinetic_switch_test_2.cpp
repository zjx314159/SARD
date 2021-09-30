
#include <bits/stdc++.h>

using namespace std;

#include "util.h"
#include "threadpool.h"
#include "mcmf.h"

auto pool = new threadpool(40);
int batch_time = 5;

struct Car {
    Worker cur_path;
    // (rid,share_dist)
    map<int, double> cands;
    set<int> reqs;
};

set<int> served_req;

bool find_best_planner(const Car &car, const Car &new_car, map<int, int> &degrees);

void doEviction(set<int> &cands, map<int, double> &score_map);

unordered_map<int, vector<pair<vector<int>, Worker>>> grouping(int w_id, const vector<int> &reqs, uint max_size);

Car get_best_k(const set<int> &groups, int w_id, const map<int, int> &degrees);

double delta;

int KINETIC_SWITCH_COUNT_3 = 0;
int KINETIC_SWITCH_COUNT_DEGREE_SMALL_3 = 0;
int KINETIC_SWITCH_COUNT_DEGREE_LARGE_3 = 0;
int KINETIC_SWITCH_COUNT_BUF_TIME_SMALL_3 = 0;
int KINETIC_SWITCH_COUNT_BUF_TIME_LARGE_3 = 0;
int KINETIC_SWITCH_COUNT_SHORT_3 = 0;
int KINETIC_SWITCH_COUNT_LONG_3 = 0;
int KINETIC_SWITCH_COUNT_4 = 0;
int KINETIC_SWITCH_COUNT_DEGREE_SMALL_4 = 0;
int KINETIC_SWITCH_COUNT_DEGREE_LARGE_4 = 0;
int KINETIC_SWITCH_COUNT_BUF_TIME_SMALL_4 = 0;
int KINETIC_SWITCH_COUNT_BUF_TIME_LARGE_4 = 0;
int KINETIC_SWITCH_COUNT_SHORT_4 = 0;
int KINETIC_SWITCH_COUNT_LONG_4 = 0;
int KINETIC_ACCEPT_3;
int KINETIC_ACCEPT_4;

void kinetic_with_order(int w_id, const vector<int> &sorted_reqs, int &counter) {
    auto path = new TaxiSchedule();
    path->value(sorted_reqs[0]);
    path->push();
    path->pushDriver(w_id);
    for (int i = 1; i < sorted_reqs.size() - 1; ++i) {
        path->value(sorted_reqs[i]);
        path->push();
    }
    path->value(sorted_reqs[sorted_reqs.size() - 1]);
    path->push_check(counter);
    delete path;
}


double get_line_angle(double k1, double k2) {
    double tan_k = (k2 - k1) / (1 + k2 * k1); //直线夹角正切值
    return atan(tan_k) * 180.0 / 3.1415926;
}
map<int, set<int>> grouping_by_angle(int *working_set, int size) {
    map<int, set<int>> res;
    vector<vertex> &vertices = sssp->vertices;
    for (int i = 0; i < size; ++i) {
        double k1 = (vertices[R[working_set[i]].e].y - vertices[R[working_set[i]].s].y)
                    / (vertices[R[working_set[i]].e].x - vertices[R[working_set[i]].s].x);
        for (int j = 0; j < size; ++j) {
            if(i==j) continue;
//            double dist = sssp->shortestDistance(R[working_set[j]].s, R[working_set[i]].s);
//            if ((dist > R[working_set[i]].ddl - CURRENT_TIME - R[working_set[i]].len
//                 && dist > R[working_set[j]].ddl - CURRENT_TIME - R[working_set[j]].len))
//                continue;
//            double angle = get_line_angle(k1, (vertices[R[working_set[j]].e].y - vertices[R[working_set[j]].s].y) /
//                                              (vertices[R[working_set[j]].e].x - vertices[R[working_set[j]].s].x));
//            if (abs(angle) > 45) {
//                continue;
//            }
            Worker tmpW;
            tmpW.pid = R[working_set[i]].s;
            tmpW.tim = CURRENT_TIME;
            tmpW.cap = CAPACITY_CONSTRAINT;
            tmpW.num = 0;
            double fit = INF;
            insertion(tmpW,working_set[i]);
            try_insertion(tmpW,working_set[j],fit);
            if(fit==INF||fit==-1) continue;
            res[working_set[i]].insert(working_set[j]);
            res[working_set[j]].insert(working_set[i]);
        }
    }
    return res;
}

void main_loop() {
    int src = 0, dst = 0, cur = 0;
    CURRENT_TIME = R[0].tim + batch_time;
    set<int> working_set;
    map<int, set<vector<int>>> visited;
    while (pos < n) {
        if (R[pos++].tim <= CURRENT_TIME) continue;
        src = dst;
        dst = pos;

        if (src / 10000 > cur) {
            cur = src / 10000;
            cout << src << endl;
        }
        cout << pos << endl;
        for (int i = 0; i < m; ++i) updateDriver(i, CURRENT_TIME);
        for (int i = src; i < dst; ++i) {
            if (R[i].len < INF_WEIGHT) working_set.insert(i);
        }

        // user prefer table
        double fit = INF;
        map<int, vector<int>> workers;
        for (auto r:working_set) {
            vector<int> car;
            if (R[r].len < INF_WEIGHT) {
                car = single_search(R[r].s, R[r].ddl - CURRENT_TIME - R[r].len);
            }
            vector<int> valued_car;
            for (int cand_c : car) {
                try_insertion(W[cand_c], r, fit);
                if (fit >= INF || fit == -1) continue;
                workers[cand_c].push_back(r);
            }
        }

        map<int, int> degrees;
        int *requests = new int[working_set.size()];
        auto it = working_set.begin();
        for (int i = 0; i < working_set.size(); ++i) {
            requests[i] = *it;
            it++;
        }
        auto graph = grouping_by_angle(requests, working_set.size());
//        export_weights(requests, working_set.size());
        delete[] requests;
        for (const auto &node : graph) degrees[node.first] = node.second.size();

        for (auto &worker : workers) {
            int w_id = worker.first;
            vector<int> reqs = worker.second;
            auto groups = grouping(w_id, reqs, 4);

            // Build SN
//            map<int, int> degrees;
//            map<int, set<int>> graph;
//            auto edges = groups[2];
//            for (auto &edge : edges) {
//                graph[edge.first[0]].insert(edge.first[1]);
//                graph[edge.first[1]].insert(edge.first[0]);
//            }
//            for (const auto &node : graph) degrees[node.first] = node.second.size();

            vector<vector<int>> tasks;
            for (const auto &it:groups[3]) {
                if (visited[w_id].find(it.first) != visited[w_id].end())continue;
                tasks.push_back(it.first);
                visited[w_id].insert(it.first);
                KINETIC_ACCEPT_3++;
            }
            int batch_size = sqrt(tasks.size());
            vector<future<int>> futures;
            for (uint j = 0; j < tasks.size(); ++j) {
                vector<vector<int>> batch;
                for (int k = 0; k < batch_size && j < tasks.size(); ++k) batch.push_back(tasks[j++]);
                futures.push_back(pool->commit([batch, w_id] {
                    int counter = 0;
                    for (const auto &p : batch) kinetic_with_order(w_id, p, counter);
                    return counter;
                }));
                futures.push_back(pool->commit([batch, w_id, &degrees] {
                    int counter = 0;
                    for (const auto &p : batch) {
                        vector<pair<double, int>> reqs_with_params;
                        vector<int> reqs_sorted;
                        reqs_with_params.emplace_back(degrees[p[0]], p[0]);
                        reqs_with_params.emplace_back(degrees[p[1]], p[1]);
                        reqs_with_params.emplace_back(degrees[p[2]], p[2]);
                        sort(reqs_with_params.begin(), reqs_with_params.end(),
                             [](const pair<int, int> &o1, const pair<int, int> &o2) {
                                 if (o1.first == o2.first) return o1.second < o2.second;
                                 return o1.first < o2.first;
                             });
                        for (auto pair:reqs_with_params) reqs_sorted.push_back(pair.second);
                        kinetic_with_order(w_id, reqs_sorted, counter);
                    }
                    return counter;
                }));
                futures.push_back(pool->commit([batch, w_id, &degrees] {
                    int counter = 0;
                    for (const auto &p : batch) {
                        vector<pair<double, int>> reqs_with_params;
                        vector<int> reqs_sorted;
                        reqs_with_params.emplace_back(degrees[p[0]], p[0]);
                        reqs_with_params.emplace_back(degrees[p[1]], p[1]);
                        reqs_with_params.emplace_back(degrees[p[2]], p[2]);
                        sort(reqs_with_params.begin(), reqs_with_params.end(),
                             [](const pair<int, int> &o1, const pair<int, int> &o2) {
                                 if (o1.first == o2.first) return o1.second < o2.second;
                                 return o1.first > o2.first;
                             });
                        for (auto pair:reqs_with_params) reqs_sorted.push_back(pair.second);
                        kinetic_with_order(w_id, reqs_sorted, counter);
                    }
                    return counter;
                }));
                futures.push_back(pool->commit([batch, w_id] {
                    int counter = 0;
                    for (const auto &p : batch) {
                        vector<pair<double, int>> reqs_with_params;
                        vector<int> reqs_sorted;
                        reqs_with_params.emplace_back(R[p[0]].len, p[0]);
                        reqs_with_params.emplace_back(R[p[1]].len, p[1]);
                        reqs_with_params.emplace_back(R[p[2]].len, p[2]);
                        sort(reqs_with_params.begin(), reqs_with_params.end(),
                             [](const pair<int, int> &o1, const pair<int, int> &o2) {
                                 if (o1.first == o2.first) return o1.second < o2.second;
                                 return o1.first < o2.first;
                             });
                        reqs_sorted.clear();
                        for (auto pair:reqs_with_params) reqs_sorted.push_back(pair.second);
                        kinetic_with_order(w_id, reqs_sorted, counter);
                    }
                    return counter;
                }));
                futures.push_back(pool->commit([batch, w_id] {
                    int counter = 0;
                    for (const auto &p : batch) {
                        vector<pair<double, int>> reqs_with_params;
                        vector<int> reqs_sorted;
                        reqs_with_params.emplace_back(R[p[0]].len, p[0]);
                        reqs_with_params.emplace_back(R[p[1]].len, p[1]);
                        reqs_with_params.emplace_back(R[p[2]].len, p[2]);
                        sort(reqs_with_params.begin(), reqs_with_params.end(),
                             [](const pair<int, int> &o1, const pair<int, int> &o2) {
                                 if (o1.first == o2.first) return o1.second < o2.second;
                                 return o1.first > o2.first;
                             });
                        reqs_sorted.clear();
                        for (auto pair:reqs_with_params) reqs_sorted.push_back(pair.second);
                        kinetic_with_order(w_id, reqs_sorted, counter);
                    }
                    return counter;
                }));
                futures.push_back(pool->commit([batch, w_id] {
                    int counter = 0;
                    for (const auto &p : batch) {
                        vector<pair<double, int>> reqs_with_params;
                        vector<int> reqs_sorted;
                        reqs_with_params.emplace_back(R[p[0]].ddl - R[p[0]].len - CURRENT_TIME, p[0]);
                        reqs_with_params.emplace_back(R[p[1]].ddl - R[p[1]].len - CURRENT_TIME, p[1]);
                        reqs_with_params.emplace_back(R[p[2]].ddl - R[p[2]].len - CURRENT_TIME, p[2]);
                        sort(reqs_with_params.begin(), reqs_with_params.end(),
                             [](const pair<int, int> &o1, const pair<int, int> &o2) {
                                 if (o1.first == o2.first) return o1.second < o2.second;
                                 return o1.first < o2.first;
                             });
                        reqs_sorted.clear();
                        for (auto pair:reqs_with_params) reqs_sorted.push_back(pair.second);
                        kinetic_with_order(w_id, reqs_sorted, counter);
                    }
                    return counter;
                }));
                futures.push_back(pool->commit([batch, w_id] {
                    int counter = 0;
                    for (const auto &p : batch) {
                        vector<pair<double, int>> reqs_with_params;
                        vector<int> reqs_sorted;
                        reqs_with_params.emplace_back(R[p[0]].ddl - R[p[0]].len - CURRENT_TIME, p[0]);
                        reqs_with_params.emplace_back(R[p[1]].ddl - R[p[1]].len - CURRENT_TIME, p[1]);
                        reqs_with_params.emplace_back(R[p[2]].ddl - R[p[2]].len - CURRENT_TIME, p[2]);
                        sort(reqs_with_params.begin(), reqs_with_params.end(),
                             [](const pair<int, int> &o1, const pair<int, int> &o2) {
                                 if (o1.first == o2.first) return o1.second < o2.second;
                                 return o1.first > o2.first;
                             });
                        reqs_sorted.clear();
                        for (auto pair:reqs_with_params) reqs_sorted.push_back(pair.second);
                        kinetic_with_order(w_id, reqs_sorted, counter);
                    }
                    return counter;
                }));
            }

            for (uint i = 0; i < futures.size();) {
                KINETIC_SWITCH_COUNT_3 += futures[i++].get();
                KINETIC_SWITCH_COUNT_DEGREE_SMALL_3 += futures[i++].get();
                KINETIC_SWITCH_COUNT_DEGREE_LARGE_3 += futures[i++].get();
                KINETIC_SWITCH_COUNT_SHORT_3 += futures[i++].get();
                KINETIC_SWITCH_COUNT_LONG_3 += futures[i++].get();
                KINETIC_SWITCH_COUNT_BUF_TIME_SMALL_3 += futures[i++].get();
                KINETIC_SWITCH_COUNT_BUF_TIME_LARGE_3 += futures[i++].get();
            }

            tasks.clear();
            for (const auto &it:groups[4]) {
                if (visited[w_id].find(it.first) != visited[w_id].end())continue;
                tasks.push_back(it.first);
                visited[w_id].insert(it.first);
                KINETIC_ACCEPT_4++;
            }
            batch_size = sqrt(tasks.size());
            futures.clear();
            for (uint j = 0; j < tasks.size(); ++j) {
                vector<vector<int>> batch;
                for (int k = 0; k < batch_size && j < tasks.size(); ++k) batch.push_back(tasks[j++]);
                futures.push_back(pool->commit([batch, w_id] {
                    int counter = 0  ;
                    for (const auto &p : batch) kinetic_with_order(w_id, p, counter);
                    return counter;
                }));
                futures.push_back(pool->commit([batch, w_id, &degrees] {
                    int counter = 0;
                    for (const auto &p : batch) {
                        vector<pair<double, int>> reqs_with_params;
                        vector<int> reqs_sorted;
                        reqs_with_params.emplace_back(degrees[p[0]], p[0]);
                        reqs_with_params.emplace_back(degrees[p[1]], p[1]);
                        reqs_with_params.emplace_back(degrees[p[2]], p[2]);
                        reqs_with_params.emplace_back(degrees[p[3]], p[3]);
                        sort(reqs_with_params.begin(), reqs_with_params.end(),
                             [](const pair<int, int> &o1, const pair<int, int> &o2) {
                                 if (o1.first == o2.first) return o1.second < o2.second;
                                 return o1.first < o2.first;
                             });
                        for (auto pair:reqs_with_params) reqs_sorted.push_back(pair.second);
                        kinetic_with_order(w_id, reqs_sorted, counter);
                    }
                    return counter;
                }));
                futures.push_back(pool->commit([batch, w_id, &degrees] {
                    int counter = 0;
                    for (const auto &p : batch) {
                        vector<pair<double, int>> reqs_with_params;
                        vector<int> reqs_sorted;
                        reqs_with_params.emplace_back(degrees[p[0]], p[0]);
                        reqs_with_params.emplace_back(degrees[p[1]], p[1]);
                        reqs_with_params.emplace_back(degrees[p[2]], p[2]);
                        reqs_with_params.emplace_back(degrees[p[3]], p[3]);
                        sort(reqs_with_params.begin(), reqs_with_params.end(),
                             [](const pair<int, int> &o1, const pair<int, int> &o2) {
                                 if (o1.first == o2.first) return o1.second < o2.second;
                                 return o1.first > o2.first;
                             });
                        for (auto pair:reqs_with_params) reqs_sorted.push_back(pair.second);
                        kinetic_with_order(w_id, reqs_sorted, counter);
                    }
                    return counter;
                }));
                futures.push_back(pool->commit([batch, w_id] {
                    int counter = 0;
                    for (const auto &p : batch) {
                        vector<pair<double, int>> reqs_with_params;
                        vector<int> reqs_sorted;
                        reqs_with_params.emplace_back(R[p[0]].len, p[0]);
                        reqs_with_params.emplace_back(R[p[1]].len, p[1]);
                        reqs_with_params.emplace_back(R[p[2]].len, p[2]);
                        reqs_with_params.emplace_back(R[p[3]].len, p[3]);
                        sort(reqs_with_params.begin(), reqs_with_params.end(),
                             [](const pair<int, int> &o1, const pair<int, int> &o2) {
                                 if (o1.first == o2.first) return o1.second < o2.second;
                                 return o1.first < o2.first;
                             });
                        reqs_sorted.clear();
                        for (auto pair:reqs_with_params) reqs_sorted.push_back(pair.second);
                        kinetic_with_order(w_id, reqs_sorted, counter);
                    }
                    return counter;
                }));
                futures.push_back(pool->commit([batch, w_id] {
                    int counter = 0;
                    for (const auto &p : batch) {
                        vector<pair<double, int>> reqs_with_params;
                        vector<int> reqs_sorted;
                        reqs_with_params.emplace_back(R[p[0]].len, p[0]);
                        reqs_with_params.emplace_back(R[p[1]].len, p[1]);
                        reqs_with_params.emplace_back(R[p[2]].len, p[2]);
                        reqs_with_params.emplace_back(R[p[3]].len, p[3]);
                        sort(reqs_with_params.begin(), reqs_with_params.end(),
                             [](const pair<int, int> &o1, const pair<int, int> &o2) {
                                 if (o1.first == o2.first) return o1.second < o2.second;
                                 return o1.first > o2.first;
                             });
                        reqs_sorted.clear();
                        for (auto pair:reqs_with_params) reqs_sorted.push_back(pair.second);
                        kinetic_with_order(w_id, reqs_sorted, counter);
                    }
                    return counter;
                }));
                futures.push_back(pool->commit([batch, w_id] {
                    int counter = 0;
                    for (const auto &p : batch) {
                        vector<pair<double, int>> reqs_with_params;
                        vector<int> reqs_sorted;
                        reqs_with_params.emplace_back(R[p[0]].ddl - R[p[0]].len - CURRENT_TIME, p[0]);
                        reqs_with_params.emplace_back(R[p[1]].ddl - R[p[1]].len - CURRENT_TIME, p[1]);
                        reqs_with_params.emplace_back(R[p[2]].ddl - R[p[2]].len - CURRENT_TIME, p[2]);
                        reqs_with_params.emplace_back(R[p[3]].ddl - R[p[3]].len - CURRENT_TIME, p[3]);
                        sort(reqs_with_params.begin(), reqs_with_params.end(),
                             [](const pair<int, int> &o1, const pair<int, int> &o2) {
                                 if (o1.first == o2.first) return o1.second < o2.second;
                                 return o1.first < o2.first;
                             });
                        reqs_sorted.clear();
                        for (auto pair:reqs_with_params) reqs_sorted.push_back(pair.second);
                        kinetic_with_order(w_id, reqs_sorted, counter);
                    }
                    return counter;
                }));
                futures.push_back(pool->commit([batch, w_id] {
                    int counter = 0;
                    for (const auto &p : batch) {
                        vector<pair<double, int>> reqs_with_params;
                        vector<int> reqs_sorted;
                        reqs_with_params.emplace_back(R[p[0]].ddl - R[p[0]].len - CURRENT_TIME, p[0]);
                        reqs_with_params.emplace_back(R[p[1]].ddl - R[p[1]].len - CURRENT_TIME, p[1]);
                        reqs_with_params.emplace_back(R[p[2]].ddl - R[p[2]].len - CURRENT_TIME, p[2]);
                        reqs_with_params.emplace_back(R[p[3]].ddl - R[p[3]].len - CURRENT_TIME, p[3]);
                        sort(reqs_with_params.begin(), reqs_with_params.end(),
                             [](const pair<int, int> &o1, const pair<int, int> &o2) {
                                 if (o1.first == o2.first) return o1.second < o2.second;
                                 return o1.first > o2.first;
                             });
                        reqs_sorted.clear();
                        for (auto pair:reqs_with_params) reqs_sorted.push_back(pair.second);
                        kinetic_with_order(w_id, reqs_sorted, counter);
                    }
                    return counter;
                }));
            }

            for (uint i = 0; i < futures.size();) {
                KINETIC_SWITCH_COUNT_4 += futures[i++].get();
                KINETIC_SWITCH_COUNT_DEGREE_SMALL_4 += futures[i++].get();
                KINETIC_SWITCH_COUNT_DEGREE_LARGE_4 += futures[i++].get();
                KINETIC_SWITCH_COUNT_SHORT_4 += futures[i++].get();
                KINETIC_SWITCH_COUNT_LONG_4 += futures[i++].get();
                KINETIC_SWITCH_COUNT_BUF_TIME_SMALL_4 += futures[i++].get();
                KINETIC_SWITCH_COUNT_BUF_TIME_LARGE_4 += futures[i++].get();
            }

            cout << "Kinetic Test 3: " << KINETIC_SWITCH_COUNT_3 << " / " << KINETIC_ACCEPT_3 << " = "
                 << 1.0 * KINETIC_SWITCH_COUNT_3 / KINETIC_ACCEPT_3 << endl;
            cout << "Kinetic Test 4: " << KINETIC_SWITCH_COUNT_4 << " / " << KINETIC_ACCEPT_4 << " = "
                 << 1.0 * KINETIC_SWITCH_COUNT_4 / KINETIC_ACCEPT_4 << endl;
            cout << "Kinetic Test(DEGREE_LOW) 3: " << KINETIC_SWITCH_COUNT_DEGREE_SMALL_3 << " / " << KINETIC_ACCEPT_3
                 << " = "
                 << 1.0 * KINETIC_SWITCH_COUNT_DEGREE_SMALL_3 / KINETIC_ACCEPT_3 << endl;
            cout << "Kinetic Test(DEGREE_LOW) 4: " << KINETIC_SWITCH_COUNT_DEGREE_SMALL_4 << " / " << KINETIC_ACCEPT_4
                 << " = "
                 << 1.0 * KINETIC_SWITCH_COUNT_DEGREE_SMALL_4 / KINETIC_ACCEPT_4 << endl;
            cout << "Kinetic Test(DEGREE_LAR) 3: " << KINETIC_SWITCH_COUNT_DEGREE_LARGE_3 << " / " << KINETIC_ACCEPT_3
                 << " = "
                 << 1.0 * KINETIC_SWITCH_COUNT_DEGREE_LARGE_3 / KINETIC_ACCEPT_3 << endl;
            cout << "Kinetic Test(DEGREE_LAR) 4: " << KINETIC_SWITCH_COUNT_DEGREE_LARGE_4 << " / " << KINETIC_ACCEPT_4
                 << " = "
                 << 1.0 * KINETIC_SWITCH_COUNT_DEGREE_LARGE_4 / KINETIC_ACCEPT_4 << endl;
            cout << "Kinetic Test(SHORT) 3: " << KINETIC_SWITCH_COUNT_SHORT_3 << " / " << KINETIC_ACCEPT_3 << " = "
                 << 1.0 * KINETIC_SWITCH_COUNT_SHORT_3 / KINETIC_ACCEPT_3 << endl;
            cout << "Kinetic Test(SHORT) 4: " << KINETIC_SWITCH_COUNT_SHORT_4 << " / " << KINETIC_ACCEPT_4 << " = "
                 << 1.0 * KINETIC_SWITCH_COUNT_SHORT_4 / KINETIC_ACCEPT_4 << endl;
            cout << "Kinetic Test(LONG) 3: " << KINETIC_SWITCH_COUNT_LONG_3 << " / " << KINETIC_ACCEPT_3 << " = "
                 << 1.0 * KINETIC_SWITCH_COUNT_LONG_3 / KINETIC_ACCEPT_3 << endl;
            cout << "Kinetic Test(LONG) 4: " << KINETIC_SWITCH_COUNT_LONG_4 << " / " << KINETIC_ACCEPT_4 << " = "
                 << 1.0 * KINETIC_SWITCH_COUNT_LONG_4 / KINETIC_ACCEPT_4 << endl;
            cout << "Kinetic Test(BUFTIME_SHORT) 3: " << KINETIC_SWITCH_COUNT_BUF_TIME_SMALL_3 << " / "
                 << KINETIC_ACCEPT_3 << " = "
                 << 1.0 * KINETIC_SWITCH_COUNT_BUF_TIME_SMALL_3 / KINETIC_ACCEPT_3 << endl;
            cout << "Kinetic Test(BUFTIME_SHORT) 4: " << KINETIC_SWITCH_COUNT_BUF_TIME_SMALL_4 << " / "
                 << KINETIC_ACCEPT_4 << " = "
                 << 1.0 * KINETIC_SWITCH_COUNT_BUF_TIME_SMALL_4 / KINETIC_ACCEPT_4 << endl;
            cout << "Kinetic Test(BUFTIME_LONG) 3: " << KINETIC_SWITCH_COUNT_BUF_TIME_LARGE_3 << " / "
                 << KINETIC_ACCEPT_3 << " = "
                 << 1.0 * KINETIC_SWITCH_COUNT_BUF_TIME_LARGE_3 / KINETIC_ACCEPT_3 << endl;
            cout << "Kinetic Test(BUFTIME_LONG) 4: " << KINETIC_SWITCH_COUNT_BUF_TIME_LARGE_4 << " / "
                 << KINETIC_ACCEPT_4 << " = "
                 << 1.0 * KINETIC_SWITCH_COUNT_BUF_TIME_LARGE_4 / KINETIC_ACCEPT_4 << endl;
        }
        for (auto it = working_set.begin(); it != working_set.end();) {
            int r = *it;
            if (R[r].ddl < (CURRENT_TIME + R[r].len)) {
                if (R[r].len < INF_WEIGHT) {
                    Penalty += R[r].len * R[r].pr;
                }
                cnt++;
                working_set.erase(it++);
            } else it++;
        }
        CURRENT_TIME += batch_time;
    }

    for (int i = 0; i < m; ++i) {
        finishTaxi(i);
    }
    t3 = clock();
    dumpResult("main_loop");
}

pair<double, double>
group_scoring(const vector<int> &reqs, Worker updated_worker, int w_id, const map<int, int> &degrees) {
    double score = 0;
    double merged = updated_worker.reach.back() - (W[w_id].reach.empty() ? CURRENT_TIME : W[w_id].reach.back());
    double origin = 0;
    for (auto r: reqs) {
        origin += R[r].len;
        score += degrees.find(r)->second;
    }
//    return {0, 0};
    return {score / reqs.size(), merged / origin};
//    return {score / group.reqs.size(), 0};
//    return {0, origin};
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
        vector<pair<int, int>> tasks;
        for (uint i = 0; i < res[cur_layer - 1].size(); ++i) {
            for (uint j = i + 1; j < res[cur_layer - 1].size(); ++j) {
                auto merged = merge_and_diff(res[cur_layer - 1][i].first, res[cur_layer - 1][j].first);
                if (merged.first.size() != cur_layer || merged.second.size() != 1 ||
                    build.find(merged.first) != build.end())
                    continue;
                tasks.emplace_back(i, j);
            }
        }
        int batch_size = sqrt(tasks.size());
        vector<future<map<vector<int>, Worker>>> futures;
        for (uint j = 0; j < tasks.size(); ++j) {
            vector<pair<int, int>> batch;
            for (int k = 0; k < batch_size && j < tasks.size(); ++k) batch.push_back(tasks[j++]);
            futures.push_back(pool->commit([batch, &res, cur_layer, &build] {
                double fit;
                map<vector<int>, Worker> tmp_res;
                for (auto p : batch) {
                    int i = p.first, j = p.second;
                    auto merged = merge_and_diff(res[cur_layer - 1][i].first, res[cur_layer - 1][j].first);
                    // Check pairs exist or not
                    auto perms = permutation_group(merged.first);
                    if (any_of(perms.begin(), perms.end(), [build](const vector<int> &g) {
                        return build.find(g) == build.end();
                    }))
                        continue;
                    int r = merged.second[0];
                    Worker w = res[cur_layer - 1][i].second.clone();
                    try_insertion(w, r, fit);
                    if (fit >= INF || fit == -1) continue;
                    insertion(w, r);
                    tmp_res.insert({merged.first, w});
                }
                return tmp_res;
            }));
        }

        for (auto &future : futures) {
            map<vector<int>, Worker> tmp_res = future.get();
            for (const auto &it: tmp_res) {
                if (build.find(it.first) != build.end()) continue;
                res[cur_layer].push_back({it.first, it.second});
                build.insert(it.first);
            }
        }
    }
    return res;
}

Car get_best_k(const set<int> &reqs, int w_id, const map<int, int> &degrees) {
    int max_size = W[w_id].cap - W[w_id].num;
    vector<int> reqs_vec;
    for (int r: reqs) reqs_vec.push_back(r);
    auto groups = grouping(w_id, reqs_vec, max_size);
    map<vector<int>, pair<pair<double, double>, Worker>> process_seq;
    for (const auto &group:groups) {
        for (const auto &it : group.second) {
            auto scores = group_scoring(it.first, it.second, w_id, degrees);
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

void doEviction(set<int> &cands, map<int, double> &score_map) {
    if (cands.empty()) return;
    int evicted_req = *cands.begin();
    for (int c:cands) {
        if (score_map[c] >= score_map[evicted_req])continue;
        evicted_req = c;
    }
    cands.erase(evicted_req);
}

bool find_best_planner(const Car &car, const Car &new_car, map<int, int> &degrees) {
    double avg_deg = 0, avg_deg_new = 0;
    for (auto it:car.cands)avg_deg += degrees[it.first];
    for (auto it:new_car.cands)avg_deg_new += degrees[it.first];
    return avg_deg_new / new_car.cands.size() < avg_deg / car.cands.size();
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
    if (argc > 10) delta = atof(args[10]);
    if (argc > 11) {
        desFile = string(args[11]);
//        freopen(desFile.c_str(), "w", stdout);
    }
    readInput();

    t1 = clock();
    initGrid();
    t2 = clock();

    main_loop();
    freeMemory();
    fflush(stdout);

    return 0;
}