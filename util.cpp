
#include "util.h"
#include "monitor.h"

int nV, m, c, n;
double gridL, alpha;
double ans = 0;
int CURRENT_TIME = 0;
double Penalty = 0;
ShortestPath *sssp = NULL;
Worker *W = NULL;
Request *R = NULL;
string vertexFile, edgeFile, pathLabelFile, orderFile;
int kineticSize = 6;
string dataFile, requestFile, desFile;
clock_t t1, t2, t3, tsd, ted;
double dispatchTime = 0;
int true_enum = 0, tot_enum = 0;
double pruneRate = 0;
double gridrate = 0;
int pos = 0;
int cnt = 0;
int mcnt = 0;
double qcnt = 0;
long long min_qcnt = LLONG_MAX / 3;
long long max_qcnt = LLONG_MIN / 3;
Grid *gr = NULL;
int *anchor = NULL;
int graph_len, graph_wid, grid_len, grid_wid;
int grid_num_per_row, grid_num_per_col, grid_sz;
double mnx = INF, mxx = -INF, mny = INF, mxy = -INF;
vector<bool> visitGrid;
int gridm = 0;
uint CAPACITY_CONSTRAINT = 4;
double GLOBAL_DDL = 2.0;
int GLOBAL_PENT = 10;
int TAXI_SIZE = 5000;
int REQUEST_SIZE = 250000;

// MACRO for timing
struct timeval timeVal;
long double tms, tme;
#define TIME_TICK_START gettimeofday( &timeVal, NULL ); tms = (long double) timeVal.tv_sec * 1000.0 + (long double) timeVal.tv_usec / 1000.0;
#define TIME_TICK_END gettimeofday( &timeVal, NULL ); tme = (long double) timeVal.tv_sec * 1000.0 + (long double) timeVal.tv_usec / 1000.0;

inline double dist(int a, int b) {
    return sssp->shortestDistance(a, b) / speed;
}

inline int Pos(int x) {
    return (x & 1) ? R[x >> 1].e : R[x >> 1].s;
}

bool feasible(vector<int> &S, Worker &w) {
    int sum = w.num;
    double tim = w.tim;
    for (int i = 0; i < S.size(); ++i) {
        if (S[i] & 1) sum -= R[S[i] >> 1].com;
        else sum += R[S[i] >> 1].com;
        if (sum > w.cap) return false;
        if (i == 0) {
            tim += dist(w.pid, Pos(S[i]));
        } else {
            tim += dist(Pos(S[i]), Pos(S[i - 1]));
        }
        if (S[i] & 1) {
            if (tim > R[S[i] >> 1].ddl) return false;
        }
    }
    return true;
}

double totalDist(vector<int> &S, Worker &w) {
    double tim = w.tim;
    for (int i = 0; i < S.size(); ++i) {
        if (i == 0) {
            tim += dist(w.pid, Pos(S[i]));
        } else {
            tim += dist(Pos(S[i - 1]), Pos(S[i]));
        }
        if (tim < R[S[i] >> 1].tim) tim = R[S[i] >> 1].tim;
    }
    return tim - w.tim;
}

inline double gridDist(int a, int b) {
    return dist(anchor[a], anchor[b]);
}

int getGridID(int id) {
    vector<vertex> &vertices = sssp->vertices;
    int rid = vertices[id].x / grid_wid;
    int cid = vertices[id].y / grid_len;
    return grid_num_per_row * rid + cid;
}

inline int getGridID(double x, double y) {
    int rid = x / grid_wid;
    int cid = y / grid_len;
    return grid_num_per_row * rid + cid;
}

inline double euclid(int a, int b) {
    vector<vertex> &vertices = sssp->vertices;
    return sqrt((vertices[a].x - vertices[b].x) * (vertices[a].x - vertices[b].x) +
                (vertices[a].y - vertices[b].y) * (vertices[a].y - vertices[b].y)) / MAX_speed;
}

inline double euclid(Position &a, Position &b) {
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y)) / MAX_speed;
}

inline double euclidDist(Position &a, Position &b) {
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

inline double euclidDist(int a, int b) {
    vector<vertex> &vertices = sssp->vertices;
    return sqrt((vertices[a].x - vertices[b].x) * (vertices[a].x - vertices[b].x) +
                (vertices[a].y - vertices[b].y) * (vertices[a].y - vertices[b].y));
}

double centerDist(int gid, int pid) {
    vector<vertex> &vertices = sssp->vertices;
    Position g, pp;
    int rid = gid / grid_num_per_row;
    int cid = gid % grid_num_per_row;
    double lx = rid * grid_wid, rx = min(1. * graph_wid, lx + grid_wid);
    double ly = cid * grid_len, ry = min(1. * graph_len, ly + grid_len);
    g.x = (lx + rx) * 0.5;
    g.y = (ly + ry) * 0.5;
    pp.x = vertices[pid].x;
    pp.y = vertices[pid].y;
    return euclidDist(g, pp);
}

void insertTaxi(int gid, int tid) {
    gr[gid].taxi.push_back(tid);
    int vid = gr[gid].taxi.size() - 1;
    W[tid].gid = gid;
    W[tid].vid = vid;
}

void deleteTaxi(int gid, int tid) {
    if (gid >= 0 && tid >= 0 && W[tid].vid >= 0 && gr[gid].taxi.size() > W[tid].vid) {
        int tid_ = *gr[gid].taxi.rbegin();
        gr[gid].taxi[W[tid].vid] = tid_;
        W[tid_].vid = W[tid].vid;
        gr[gid].taxi.pop_back();
        W[tid].vid = W[tid].gid = -1;
    }
}

void insertRider(int gid, int rid) {
    gr[gid].rider.push_back(rid);
    int vid = gr[gid].rider.size() - 1;
    R[rid].gid = gid;
    R[rid].vid = vid;
}

void deleteRider(int gid, int rid) {
    if (gid >= 0 && rid >= 0 && R[rid].vid >= 0 && gr[gid].rider.size() > R[rid].vid) {
        int tid_ = *gr[gid].rider.rbegin();
        gr[gid].rider[R[rid].vid] = tid_;
        R[tid_].vid = R[rid].vid;
        gr[gid].rider.pop_back();
        R[rid].vid = R[rid].gid = -1;
    }
}

// (x,y) center of ellipse
vector<int> ellipseQuery(int rid, int currentTime) {
    // get a & b
    vector<vertex> &vertices = sssp->vertices;
    double c = R[rid].len / 2.0;
    double a = (R[rid].ddl - currentTime) / 2;
    double b_b = abs(a * a - c * c);
    double theta = atan((vertices[R[rid].s].y - vertices[R[rid].e].y) / (vertices[R[rid].s].x - vertices[R[rid].e].x));
    double x = (vertices[R[rid].s].x + vertices[R[rid].e].x) / 2;
    double y = (vertices[R[rid].s].y + vertices[R[rid].e].y) / 2;

    // standard ellipse bounding after rotation
    double x1 = sqrt(a * a * cos(theta) * cos(theta) + b_b * sin(theta) * sin(theta));
    double x2 = -x1;
    double y1 = sqrt(a * a * sin(theta) * sin(theta) + b_b * cos(theta) * cos(theta));
    double y2 = -y1;

    // move center
    x1 = x1 * MAX_speed + x;
    x2 = x2 * MAX_speed + x;
    y1 = y1 * MAX_speed + y;
    y2 = y2 * MAX_speed + y;

    // search and return
    int gid0 = getGridID(x, y);
    int x0 = gid0 / grid_num_per_row;
    int y0 = gid0 % grid_num_per_row;
    int rx = ceil((x1 - x2) / grid_wid), ry = ceil((y1 - y2) / grid_len);
    vector<int> ret;

    fill(visitGrid.begin(), visitGrid.end(), false);
    for (int dx = -rx; dx <= rx; ++dx) {
        for (int dy = -ry; dy <= ry; ++dy) {
            int x = x0 + dx, y = y0 + dy;
            if (x >= 0 && x < grid_num_per_col && y >= 0 && y < grid_num_per_row) {
                int gid = x * grid_num_per_row + y;
                if (!visitGrid[gid]) {
                    visitGrid[gid] = true;
                    for (int i: gr[gid].rider) {
                        ret.push_back(i);
                    }
                }
            }
        }
    }
    return ret;
}

void initGrid() {
    vector<vertex> &vertices = sssp->vertices;

    for (int i = 0; i < nV; ++i) {
        if (vertices[i].x == 0.0) continue;
        mnx = min(mnx, vertices[i].x);
        mxx = max(mxx, vertices[i].x);
        mny = min(mny, vertices[i].y);
        mxy = max(mxy, vertices[i].y);
    }
    graph_len = ceil(mxy - mny);
    graph_wid = ceil(mxx - mnx);
    grid_len = gridL;
    grid_wid = gridL;

    grid_num_per_row = (graph_len + grid_len) / grid_len;
    grid_num_per_col = (graph_wid + grid_wid) / grid_wid;
    grid_sz = grid_num_per_row * grid_num_per_col;
    visitGrid.resize(grid_sz);
    printf("grid size = %d\n", grid_sz);
    fflush(stdout);

    for (int i = 0; i < nV; ++i) {
        if (vertices[i].x == 0.0) continue;
        vertices[i].x -= mnx;
        vertices[i].y -= mny;
        vertices[i].x = max(vertices[i].x, 0.);
        vertices[i].y = max(vertices[i].y, 0.);
    }

    gr = new Grid[grid_sz];
    anchor = new int[grid_sz];
    memset(anchor, -1, sizeof(int) * grid_sz);
    for (int i = 0; i < nV; ++i) {
        int id = getGridID(i);
        if (anchor[id] == -1 || centerDist(id, i) < centerDist(id, anchor[id])) {
            anchor[id] = i;
        }
    }

    for (int i = 0; i < m; ++i) {
        int id = W[i].pid;
        insertTaxi(getGridID(id), i);
    }
}

void updateGrid(int pid, int wid, int tag) {
    int gid = getGridID(pid);
    if (tag == 1) {
        insertTaxi(gid, wid);
    } else {
        deleteTaxi(gid, wid);
    }
}

vector<int> single_search(int s, double ddl) {
    vector<vertex> &vertices = sssp->vertices;
    int gid0 = getGridID(vertices[s].x, vertices[s].y);
    int x0 = gid0 / grid_num_per_row;
    int y0 = gid0 % grid_num_per_row;
    double radius = ddl * MAX_speed;
    int rx = ceil(radius / grid_wid), ry = ceil(radius / grid_len);
    vector<int> ret;

    fill(visitGrid.begin(), visitGrid.end(), false);
    for (int dx = -rx; dx <= rx; ++dx) {
        for (int dy = -ry; dy <= ry; ++dy) {
            int x = x0 + dx, y = y0 + dy;
            if (x >= 0 && x < grid_num_per_col && y >= 0 && y < grid_num_per_row) {
                int gid = x * grid_num_per_row + y;
                if (!visitGrid[gid]) {
                    visitGrid[gid] = true;
                    for (int i: gr[gid].taxi) {
                        ret.push_back(i);
                    }
                }
            }
        }
    }

    gridm = ret.size();
    gridrate += 1.0 * gridm / m;

    return ret;
}

inline double det(int x, int y, int z) {
    return dist(x, y) + dist(y, z) - dist(x, z);
}

inline double det(int a, int b, int c, int d) {
    return dist(a, b) + dist(b, c) + dist(c, d) - dist(a, d);
}

void updateDriverArr(Worker &w) {
    double tim = w.tim;
    vector<double> &reach = w.reach;

    reach.clear();
    for (int i = 0; i < w.S.size(); ++i) {
        if (i == 0) {
            tim += dist(w.pid, Pos(w.S[i]));
        } else {
            tim += dist(Pos(w.S[i]), Pos(w.S[i - 1]));
        }
        reach.push_back(tim);
    }

    vector<int> &picked = w.picked;
    vector<double> &slack = w.slack;
    picked.clear();
    slack.clear();
    int cc = w.num;

    for (uint i = 0; i < w.S.size(); ++i) {
        if (w.S[i] & 1) {
            cc -= R[w.S[i] >> 1].com;
            slack.push_back(R[w.S[i] >> 1].ddl - reach[i]);
        } else {
            cc += R[w.S[i] >> 1].com;
            slack.push_back(INF);
        }
        picked.push_back(cc);
    }
    for (int i = slack.size() - 2; i >= 0; --i) {
        slack[i] = min(slack[i], slack[i + 1]);
    }
}

void try_insertion_euclid(Worker &w, int rid, double &delta) {
    Request &r = R[rid];
    double opt = INF;

    if (w.S.empty()) {
        double tmp = w.tim + euclid(w.pid, r.s) + r.len;
        if (tmp < r.ddl + EPS && r.com <= w.cap) {
            opt = tmp - w.tim;
            delta = opt;
        }
        return;
    }

    vector<int> &picked = w.picked;
    vector<double> &slack = w.slack;
    vector<double> &reach = w.reach;

    for (int i = 0; i <= w.S.size(); ++i) {
        if (i == 0) {
            double part1 = euclid(w.pid, r.s);
            double detour = part1 + r.len + euclid(r.e, Pos(w.S[0])) - (reach[0] - w.tim);
            if (w.num + r.com <= w.cap &&
                detour < slack[i] + EPS &&
                part1 + r.len < r.ddl - w.tim + EPS) {
                if (opt > detour) {
                    opt = detour;
                    delta = opt;
                }
            }
        } else if (i == w.S.size()) {
            double detour = euclid(Pos(w.S[i - 1]), r.s) + r.len;
            if (picked[i - 1] + r.com <= w.cap &&
                detour < r.ddl - reach[i - 1] + EPS) {
                if (opt > detour) {
                    opt = detour;
                    delta = opt;
                }
            }
        } else {
            double part1 = euclid(Pos(w.S[i - 1]), r.s);
            double detour = part1 + r.len + euclid(r.e, Pos(w.S[i])) - (reach[i] - reach[i - 1]);

            if (picked[i - 1] + r.com <= w.cap &&
                detour < slack[i] + EPS &&
                part1 + r.len < r.ddl - reach[i - 1] + EPS) {
                if (opt > detour) {
                    opt = detour;
                    delta = opt;
                }
            }
        }
    }

    vector<pair<double, int>> Det;
    for (int i = 0; i < w.S.size(); ++i) {
        pair<double, int> tmp = make_pair(INF, -1);
        if (i == 0) {
            if (w.num + r.com <= w.cap) {
                double detour = euclid(w.pid, r.s) + euclid(r.s, Pos(w.S[0])) - (reach[0] - w.tim);
                if (detour < slack[i] + EPS) {
                    tmp = make_pair(detour, i);
                }
            }
        } else {
            if (picked[i - 1] + r.com <= w.cap) {
                tmp = Det.back();
                double detour = euclid(Pos(w.S[i - 1]), r.s) + euclid(r.s, Pos(w.S[i])) - (reach[i] - reach[i - 1]);
                if (detour < slack[i] + EPS) {
                    tmp = min(make_pair(detour, i), tmp);
                }
            }
        }
        Det.push_back(tmp);
    }
    for (int i = 0; i < w.S.size(); ++i) {
        if (i < w.S.size() - 1) {
            if (picked[i] > w.cap - r.com) continue;
            double part1 = euclid(Pos(w.S[i]), r.e);
            double detour = part1 + euclid(r.e, Pos(w.S[i + 1])) - (reach[i + 1] - reach[i]);
            if (Det[i].first + detour < slack[i + 1] + EPS
                && Det[i].first + part1 < r.ddl - reach[i] + EPS) {
                if (opt > Det[i].first + detour) {

                    opt = Det[i].first + detour;
                    delta = opt;
                }

            }

        } else {
            double detour = euclid(Pos(w.S[i]), r.e);
            if (Det[i].first + detour < r.ddl - reach[i] + EPS) {
                if (opt > Det[i].first + detour) {
                    opt = Det[i].first + detour;
                    delta = opt;
                }
            }
        }
    }
}

void try_insertion(const Worker &w, int rid, double &delta) {
    Request &r = R[rid];
    double opt = INF;

    delta = INF;
    if (w.S.empty()) {
        double tmp = w.tim + dist(w.pid, r.s) + r.len;
        if (tmp < r.ddl + EPS && r.com <= w.cap) {
            opt = tmp - w.tim;
            delta = opt;
        }
        return;
    }

    // local cache of distances
    double swd = dist(w.pid, r.s);
    vector<double> sdcache(w.S.size(), INF);
    vector<double> edcache(w.S.size(), INF);
    for (int i = 0; i < w.S.size(); ++i) {
        sdcache[i] = dist(r.s, Pos(w.S[i]));
        edcache[i] = dist(r.e, Pos(w.S[i]));
    }

    const vector<int> &picked = w.picked;
    const vector<double> &slack = w.slack;
    const vector<double> &reach = w.reach;

    for (int i = 0; i <= w.S.size(); ++i) {
        if (i == 0) {
            double part1 = swd;
            double detour = part1 + r.len + edcache[i] - (reach[0] - w.tim);
            if (w.num + r.com <= w.cap &&
                detour < slack[i] + EPS &&
                part1 + r.len < r.ddl - w.tim + EPS) {
                if (opt > detour) {
                    opt = detour;
                    delta = opt;
                }
            }
        } else if (i == w.S.size()) {
            double detour = sdcache[i - 1] + r.len;
            if (picked[i - 1] + r.com <= w.cap &&
                detour < r.ddl - reach[i - 1] + EPS) {
                if (opt > detour) {
                    opt = detour;
                    delta = opt;
                }
            }
        } else {
            double part1 = sdcache[i - 1];
            double detour = part1 + r.len + edcache[i] - (reach[i] - reach[i - 1]);

            if (picked[i - 1] + r.com <= w.cap &&
                detour < slack[i] + EPS &&
                part1 + r.len < r.ddl - reach[i - 1] + EPS) {
                if (opt > detour) {
                    opt = detour;
                    delta = opt;
                }
            }
        }
    }

    vector<pair<double, int>> Det;
    for (int i = 0; i < w.S.size(); ++i) {
        pair<double, int> tmp = make_pair(INF, -1);
        if (i == 0) {
            if (w.num + r.com <= w.cap) {
                double detour = swd + sdcache[0] - (reach[0] - w.tim);
                if (detour < slack[i] + EPS) {
                    tmp = make_pair(detour, i);
                }
            }
        } else {
            if (picked[i - 1] + r.com <= w.cap) {
                tmp = Det.back();
                double detour = sdcache[i - 1] + sdcache[i] - (reach[i] - reach[i - 1]);
                if (detour < slack[i] + EPS) {
                    tmp = min(make_pair(detour, i), tmp);
                }
            }
        }
        Det.push_back(tmp);
    }
    for (int i = 0; i < w.S.size(); ++i) {
        if (i < w.S.size() - 1) {
            if (picked[i] > w.cap - r.com) continue;
            double part1 = edcache[i];
            double detour = part1 + edcache[i + 1] - (reach[i + 1] - reach[i]);
            if (Det[i].first + detour < slack[i + 1] + EPS
                && Det[i].first + part1 < r.ddl - reach[i] + EPS) {
                if (opt > Det[i].first + detour) {
                    opt = Det[i].first + detour;
                    delta = opt;
                }
            }
        } else {
            double detour = edcache[i];
            if (Det[i].first + detour < r.ddl - reach[i] + EPS) {
                if (opt > Det[i].first + detour) {
                    opt = Det[i].first + detour;
                    delta = opt;
                }
            }
        }
    }
}

void insertion(Worker &w, int rid) {
    Request &r = R[rid];
    double opt = INF;
    vector<int> ret = w.S;
    pair<int, int> ins = make_pair(-1, -1);

    if (w.S.empty()) {
        double kk = dist(w.pid, r.s);
        double tmp = w.tim + kk + r.len;
        if (tmp < r.ddl + EPS && r.com <= w.cap) {
            opt = tmp - w.tim;
            w.S.push_back(rid << 1);
            w.S.push_back(rid << 1 | 1);
            updateDriverArr(w);
        }
        return;
    }

    // local cache of distances
    double swd = dist(w.pid, r.s);
    vector<double> sdcache(w.S.size(), INF);
    vector<double> edcache(w.S.size(), INF);
    for (int i = 0; i < w.S.size(); ++i) {
        sdcache[i] = dist(r.s, Pos(w.S[i]));
        edcache[i] = dist(r.e, Pos(w.S[i]));
    }

    vector<int> &picked = w.picked;
    vector<double> &slack = w.slack;
    vector<double> &reach = w.reach;

    for (uint i = 0; i <= w.S.size(); ++i) {
        if (i == 0) {
            double part1 = swd;
            double part2 = edcache[i];
            double detour = part1 + r.len + part2 - (reach[0] - w.tim);
            if (w.num + r.com <= w.cap &&
                detour < slack[i] + EPS &&
                part1 + r.len < r.ddl - w.tim + EPS) {
                if (opt > detour) {
                    opt = detour;
                    ins = make_pair(i, i);
                }
            }
        } else if (i == w.S.size()) {
            double part1 = sdcache[i - 1];
            double detour = part1 + r.len;
            if (picked[i - 1] + r.com <= w.cap &&
                detour < r.ddl - reach[i - 1] + EPS) {
                if (opt > detour) {
                    opt = detour;
                    ins = make_pair(i, i);
                }
            }
        } else {
            double part1 = sdcache[i - 1];
            double part2 = edcache[i];
            double detour = part1 + r.len + part2 - (reach[i] - reach[i - 1]);
            if (picked[i - 1] + r.com <= w.cap &&
                detour < slack[i] + EPS &&
                part1 + r.len < r.ddl - reach[i - 1] + EPS) {
                if (opt > detour) {
                    opt = detour;
                    ins = make_pair(i, i);
                }
            }
        }
    }

    vector<pair<double, int>> Det;
    for (int i = 0; i < w.S.size(); ++i) {
        pair<double, int> tmp = make_pair(INF, -1);
        if (i == 0) {
            if (w.num + r.com <= w.cap) {
                double detour = swd + sdcache[i] - (reach[0] - w.tim);
                if (detour < slack[i] + EPS) {
                    tmp = make_pair(detour, i);
                }
            }
        } else {
            if (picked[i - 1] + r.com <= w.cap) {
                tmp = Det.back();
                double detour = sdcache[i - 1] + sdcache[i] - (reach[i] - reach[i - 1]);
                if (detour < slack[i] + EPS) {
                    tmp = min(make_pair(detour, i), tmp);
                }
            }
        }
        Det.push_back(tmp);
    }
    for (uint i = 0; i < w.S.size(); ++i) {
        if (i < w.S.size() - 1) {
            if (picked[i] > w.cap - r.com) continue;
            double part1 = edcache[i];
            double detour = part1 + edcache[i + 1] - (reach[i + 1] - reach[i]);
            if (Det[i].first + detour < slack[i + 1] + EPS
                && Det[i].first + part1 < r.ddl - reach[i] + EPS) {
                if (opt > Det[i].first + detour) {
                    opt = Det[i].first + detour;
                    ins = make_pair(Det[i].second, i + 1);
                }
            }
        } else {
            double detour = edcache[i];
            if (Det[i].first + detour < r.ddl - reach[i] + EPS) {
                if (opt > Det[i].first + detour) {
                    opt = Det[i].first + detour;
                    ins = make_pair(Det[i].second, i + 1);
                }
            }
        }
    }
    if (ins.first > -1) {
        ret.clear();
        for (int j = 0; j < ins.first; ++j) {
            ret.push_back(w.S[j]);
        }
        ret.push_back(rid << 1);
        for (int j = ins.first; j < ins.second; ++j) {
            ret.push_back(w.S[j]);
        }
        ret.push_back(rid << 1 | 1);
        for (int j = ins.second; j < w.S.size(); ++j) {
            ret.push_back(w.S[j]);
        }

        w.S = ret;
        updateDriverArr(w);
    }
}

Worker getUpdatedWorker(const Worker &w, int rid, pair<int, int> ins) {
    vector<int> ret;
    for (int j = 0; j < ins.first; ++j) {
        ret.push_back(w.S[j]);
    }
    ret.push_back(rid << 1);
    for (int j = ins.first; j < ins.second; ++j) {
        ret.push_back(w.S[j]);
    }
    ret.push_back(rid << 1 | 1);
    for (int j = ins.second; j < w.S.size(); ++j) {
        ret.push_back(w.S[j]);
    }
    auto tmp_w = w.clone();
    tmp_w.S = ret;
    updateDriverArr(tmp_w);
    return tmp_w;
}

// Given one worker return all possible new worker with rid
vector<Worker> insertion_all(const Worker &w, int rid) {
    vector<Worker> res;
    Request &r = R[rid];
    vector<int> ret = w.S;
    pair<int, int> ins;
    if (w.S.empty()) {
        double kk = dist(w.pid, r.s);
        double tmp = w.tim + kk + r.len;
        if (tmp < r.ddl + EPS && r.com <= w.cap) {
            auto tmp_w = w.clone();
            tmp_w.S.push_back(rid << 1);
            tmp_w.S.push_back(rid << 1 | 1);
            updateDriverArr(tmp_w);
            res.push_back(tmp_w);
        }
        return res;
    }

    // local cache of distances
    double swd = dist(w.pid, r.s);
    vector<double> sdcache(w.S.size(), INF);
    vector<double> edcache(w.S.size(), INF);
    for (int i = 0; i < w.S.size(); ++i) {
        sdcache[i] = dist(r.s, Pos(w.S[i]));
        edcache[i] = dist(r.e, Pos(w.S[i]));
    }

    const vector<int> &picked = w.picked;
    const vector<double> &slack = w.slack;
    const vector<double> &reach = w.reach;

    for (uint i = 0; i <= w.S.size(); ++i) {
        if (i == 0) {
            double part1 = swd;
            double part2 = edcache[i];
            double detour = part1 + r.len + part2 - (reach[0] - w.tim);
            if (w.num + r.com <= w.cap &&
                detour < slack[i] + EPS &&
                part1 + r.len < r.ddl - w.tim + EPS) {
                ins = make_pair(i, i);
                res.push_back(getUpdatedWorker(w, rid, ins));
            }
        } else if (i == w.S.size()) {
            double part1 = sdcache[i - 1];
            double detour = part1 + r.len;
            if (picked[i - 1] + r.com <= w.cap &&
                detour < r.ddl - reach[i - 1] + EPS) {
                ins = make_pair(i, i);
                res.push_back(getUpdatedWorker(w, rid, ins));
            }
        } else {
            double part1 = sdcache[i - 1];
            double part2 = edcache[i];
            double detour = part1 + r.len + part2 - (reach[i] - reach[i - 1]);
            if (picked[i - 1] + r.com <= w.cap &&
                detour < slack[i] + EPS &&
                part1 + r.len < r.ddl - reach[i - 1] + EPS) {
                ins = make_pair(i, i);
                res.push_back(getUpdatedWorker(w, rid, ins));
            }
        }
    }

    vector<pair<double, int>> Det;
    for (int i = 0; i < w.S.size(); ++i) {
        pair<double, int> tmp = make_pair(INF, -1);
        if (i == 0) {
            if (w.num + r.com <= w.cap) {
                double detour = swd + sdcache[i] - (reach[0] - w.tim);
                if (detour < slack[i] + EPS) {
                    tmp = make_pair(detour, i);
                }
            }
        } else {
            if (picked[i - 1] + r.com <= w.cap) {
                tmp = Det.back();
                double detour = sdcache[i - 1] + sdcache[i] - (reach[i] - reach[i - 1]);
                if (detour < slack[i] + EPS) {
                    tmp = min(make_pair(detour, i), tmp);
                }
            }
        }
        Det.push_back(tmp);
    }
    for (uint i = 0; i < w.S.size(); ++i) {
        if (i < w.S.size() - 1) {
            if (picked[i] > w.cap - r.com) continue;
            double part1 = edcache[i];
            double detour = part1 + edcache[i + 1] - (reach[i + 1] - reach[i]);
            if (Det[i].first + detour < slack[i + 1] + EPS
                && Det[i].first + part1 < r.ddl - reach[i] + EPS) {
                ins = make_pair(Det[i].second, i + 1);
                res.push_back(getUpdatedWorker(w, rid, ins));
            }
        } else {
            double detour = edcache[i];
            if (Det[i].first + detour < r.ddl - reach[i] + EPS) {
                ins = make_pair(Det[i].second, i + 1);
                res.push_back(getUpdatedWorker(w, rid, ins));
            }
        }
    }
    return res;
}

void readInput() {
    //initialize shortest path interface
    cout << "begin sssp" << endl;
    sssp = new ShortestPath(vertexFile, edgeFile, pathLabelFile, orderFile);
    nV = numOfVertices;
    cout << "end sssp" << endl;

    ifstream other;
    other.open(dataFile.c_str());
    if (!other.is_open()) {
        printf("%s does not exist\n", dataFile.c_str());
        exit(0);
    }
    double ddl, pr;
    other >> m >> c >> gridL >> alpha;
    m = TAXI_SIZE;
    c = CAPACITY_CONSTRAINT;
    W = new Worker[m];
    for (int i = 0; i < m; ++i) {
        other >> W[i].pid >> W[i].cap;
        W[i].cap = CAPACITY_CONSTRAINT;
        W[i].num = 0;
        schedule[i].emplace_back(W[i].pid,0);
    }
    other >> ddl >> pr;
    ddl = GLOBAL_DDL;
    pr = GLOBAL_PENT;
    other.close();

    ifstream ifs;
    ifs.open(requestFile.c_str());
    if (!ifs.is_open()) {
        printf("%s does not exist\n", requestFile.c_str());
        exit(0);
    }
    cout << "begin request" << endl;
    ifs >> n;
    n = REQUEST_SIZE;
    R = new Request[n];
    int cnt = 0;
    while (cnt < REQUEST_SIZE) {
        ifs >> R[cnt].tim >> R[cnt].s >> R[cnt].e >> R[cnt].com;
        R[cnt].len = dist(R[cnt].s, R[cnt].e);
        if (abs(R[cnt].len - 0.0) <= EPS || R[cnt].len >= INF) continue;
//        R[cnt].ddl = R[cnt].tim + R[cnt].len + 300;
        R[cnt].ddl = R[cnt].tim + R[cnt].len * GLOBAL_DDL;
        R[cnt].pr = pr;
        R[cnt].gid = -1;
        R[cnt].vid = -1;
        cnt++;
    }

    for (int i = 0; i < m; ++i) {
        schedule[i].back().second = R[0].tim;
    }
    ifs.close();
    cout << "end request" << endl;

    cout << "Worker: " << m << " Requests: " << n << " Capacity:" << c << " grid:" << gridL << " KineticSize: "
         << kineticSize << " alpha:" << alpha << " ddl:" << ddl << "x" << endl;
}

void freeMemory() {
    delete sssp;
    delete[] R;
//    for (int i = 0; i < m; i++) delete W[i].path;
    delete[] W;
    delete[] gr;
    delete[] anchor;
}

void dumpResult(const char *execName) {
    qcnt *= 2;
    min_qcnt *= 2;
    max_qcnt *= 2;
    printf("%d + %d = %d, %d\n", cnt, mcnt, cnt + mcnt, pos);
    printf("%s:\n", execName);
    printf("Grid Time Cost: %.4f\n", 1.0 * (t2 - t1) / CLOCKS_PER_SEC);
    printf("Simulate Time Cost: %.4f\n", 1.0 * (t3 - t2) / CLOCKS_PER_SEC);
    printf("Dispatch Time Cost(per request): %.4f\n", 1. * dispatchTime / pos);
    printf("Rate: %.4f\nPrice Collection: %.4lf + %.4lf = %.4lf\n", 1. - 1. * cnt / pos, ans, Penalty, ans + Penalty);
    printf("Grid Rate: %.4f\nPrune Rate: %.4f\n", 1.0 - gridrate / pos, 1.0 - pruneRate / pos);
    printf("Saved shortest distance: tot = %.1lf, avg = %.1lf, min = %lld, max = %lld\n", qcnt, qcnt / pos, min_qcnt,
           max_qcnt);
    printf("Queried shortest distance: tot = %.0lf\n", sssp->sdqn);
    printf("Queried shortest path: tot = %.0lf\n", sssp->spqn);
//#ifdef WATCH_MEM
    watchSolutionOnce(getpid(), usedMemory);
    printf("Memory: %.4lf\n", usedMemory / 1024.0);
//#endif
}

void updateDriver(int i, double t) {
    Worker &w = W[i];

    while (w.S.size() > 0 && w.tim < t) {
        updateGrid(w.pid, i, -1);
        double tmp = w.reach[0] - w.tim; // == dist(w.pid, Pos(w.S[0]))
        ans += alpha * tmp;
        w.tim += tmp;
        w.pid = Pos(w.S[0]);
        schedule[i].emplace_back(w.pid,w.reach[0]);
        updateGrid(w.pid, i, 1);
        if (w.S[0] & 1) {
            w.num -= R[w.S[0] >> 1].com;
            mcnt++;
        } else {
            w.num += R[w.S[0] >> 1].com;
        }
        w.pop();
    }
    if (w.tim < t) {
        w.tim = t;
    }
}

void finishTaxi(int i) {
    Worker &w = W[i];
    int w_id = i;
    for (int i = 0; i < w.S.size(); ++i) {
        double tmp = w.reach[i] - w.tim;
        schedule[w_id].emplace_back(Pos(w.S[i]),w.reach[i]);
        ans += alpha * tmp;
        w.tim += tmp;
        if (w.S[i] & 1) {
            mcnt++;
        }
    }
}

void assignTaxi(vector<int> &car) {
    double opt = INF, fit;
    int sz = car.size(), id = -1;

    tot_enum = sz, true_enum = 0;
    if (R[pos].len < INF_WEIGHT) {
        for (int i = 0; i < sz; ++i) {
            true_enum++;
            fit = INF;
            try_insertion(W[car[i]], pos, fit);

            if (fit < INF) {
                if (opt > fit) {
                    opt = fit;
                    id = car[i];
                }
            }
        }
    }

    if (gridm > 0) {
        pruneRate += 1. * true_enum / gridm;
    }
    if (id > -1) {
        insertion(W[id], pos);
    } else {
        if (R[pos].len < INF_WEIGHT) {
            Penalty += R[pos].len * R[pos].pr;
        }
        cnt++;
    }
    pos++;
    ted = clock();
    dispatchTime += 1. * (ted - tsd) / CLOCKS_PER_SEC;
}

void assignTaxi(vector<pair<double, int>> &cars) {
    double opt = INF, fit;
    int sz = cars.size(), id = -1;

    tot_enum = sz, true_enum = 0;
    if (R[pos].len < INF_WEIGHT) {
        for (int i = 0; i < sz; ++i) {
            true_enum++;
            fit = INF;
            try_insertion(W[cars[i].second], pos, fit);

            if (fit < INF) {
                if (opt > fit) {
                    opt = fit;
                    id = cars[i].second;
                }
                if (i < sz - 1 && opt < cars[i + 1].first) {
                    long long tmp = 0;
                    for (int j = i + 1; j < sz; ++j) {
                        qcnt += (W[cars[j].second].S.size() + 1);
                        tmp += (W[cars[j].second].S.size() + 1);
                    }
                    min_qcnt = min(min_qcnt, tmp);
                    max_qcnt = max(max_qcnt, tmp);
                    break;
                }
            }
        }
    }
    if (gridm > 0) {
        pruneRate += 1. * true_enum / gridm;
    }
    if (id > -1) {
        insertion(W[id], pos);
    } else {
        if (R[pos].len < INF_WEIGHT) {
            Penalty += R[pos].len * R[pos].pr;
            cnt++;
        }
    }
//    pos++;
    ted = clock();
    dispatchTime += 1. * (ted - tsd) / CLOCKS_PER_SEC;
}

void assignTaxiFirst(vector<pair<double, int>> &cars) {
    double opt = INF, fit;
    int sz = cars.size(), id = -1;

    tot_enum = sz, true_enum = 0;

    for (int i = 0; i < sz; ++i) {
        true_enum++;
        fit = INF;
        try_insertion(W[cars[i].second], pos, fit);
        if (fit >= INF)continue;
        if (opt > fit) {
            opt = fit;
            id = cars[i].second;
        }
        if (i < sz - 1 && opt < cars[i + 1].first) {
            long long tmp = 0;
            for (int j = i + 1; j < sz; ++j) {
                qcnt += (W[cars[j].second].S.size() + 1);
                tmp += (W[cars[j].second].S.size() + 1);
            }
            min_qcnt = min(min_qcnt, tmp);
            max_qcnt = max(max_qcnt, tmp);
            break;
        }
        break;
    }
    if (gridm > 0) {
        pruneRate += 1. * true_enum / gridm;
    }
    if (id > -1) {
        insertion(W[id], pos);
    } else {
        Penalty += R[pos].len * R[pos].pr;
        cnt++;
    }
//    pos++;
    ted = clock();
    dispatchTime += 1. * (ted - tsd) / CLOCKS_PER_SEC;
}