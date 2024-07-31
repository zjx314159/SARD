
#ifndef UTIL_H
#define UTIL_H

#include <bits/stdc++.h>

using namespace std;

#include "global.h"
#include "metric.h"
#include "labels.h"
#include "libs/threadpool.h"
#include "libs/kinetic_tree/taxi_schedule.h"

#ifdef WATCH_MEM
#include "monitor.h"
#endif

extern uint CAPACITY_CONSTRAINT;

class TaxiSchedule;

struct Worker {
    int pid, num, cap, gid, vid;
    double tim;
    int rwn;
    vector<int> S;
    vector<double> reach;
    vector<int> picked;
    vector<double> slack;

    Worker(double tim) {
        pid = gid = vid = -1;
        num = 0;
        cap = CAPACITY_CONSTRAINT;
        this->tim = tim;
    }

    TaxiSchedule *path = nullptr;

    Worker() {}

    Worker clone() const {
        Worker w;
        w.pid = pid;
        w.num = num;
        w.cap = cap;
        w.gid = gid;
        w.vid = vid;
        w.tim = tim;
        w.rwn = rwn;
        w.S.resize(S.size());
        w.reach.resize(reach.size());
        w.slack.resize(slack.size());
        w.picked.resize(picked.size());
        for (uint i = 0; i < S.size(); ++i) w.S[i] = S[i];
        for (uint i = 0; i < slack.size(); ++i) w.slack[i] = slack[i];
        for (uint i = 0; i < reach.size(); ++i) w.reach[i] = reach[i];
        for (uint i = 0; i < picked.size(); ++i) w.picked[i] = picked[i];
        return w;
    }

    inline void pop() {
        if (!S.empty()) {
            S.erase(S.begin());
            reach.erase(reach.begin());
            picked.erase(picked.begin());
            slack.erase(slack.begin());
        }
    }
};

struct Request {
    int s, e, com, gid, vid;
    double tim, ddl, pr, len;
};

extern int CURRENT_TIME;
extern const int MAX_NODE;
extern int nV, m, c, n;
extern double gridL, alpha;
extern double ans;
extern double Penalty;
extern ShortestPath *sssp;
extern Worker *W;
extern Request *R;
extern string vertexFile, edgeFile, pathLabelFile, orderFile;
extern int kineticSize;
extern string dataFile, requestFile, desFile;
extern clock_t t1, t2, t3, tsd, ted, intersect_tim;
extern double dispatchTime;
extern int true_enum, tot_enum;
extern double pruneRate;
extern double gridrate;
extern int pos, cnt, mcnt;
extern double qcnt;
extern long long min_qcnt, max_qcnt;
extern vector<bool> visitGrid;
extern int gridm;
extern double GLOBAL_DDL;
extern int GLOBAL_PENT;
extern int TAXI_SIZE;
extern int REQUEST_SIZE;

struct Grid {
    vector<int> taxi;
    vector<int> rider;
};

struct Position {
    double x, y;
};

extern Grid *gr;
extern int *anchor;
extern int graph_len, graph_wid, grid_len, grid_wid;
extern int grid_num_per_row, grid_num_per_col, grid_sz;
extern double mnx, mxx, mny, mxy;

// related to common usage
int Pos(int x);

double dist(int a, int b);

double euclid(int a, int b);

double euclid(Position &a, Position &b);

double euclidDist(int a, int b);

double euclidDist(Position &a, Position &b);

bool feasible(vector<int> &S, Worker &w);

double totalDist(vector<int> &S, Worker &w);

void insertTaxi(int gid, int tid);

void deleteTaxi(int gid, int tid);

// maintains for requests in pruning
void insertRider(int gid, int rid);

void deleteRider(int gid, int rid);

vector<int> ellipseQuery(int rid, int currentTime);

void updateDriver(int i, double t);

void finishTaxi(int i);

void readInput();

void freeMemory();

void dumpResult(const char *execName);

void assignTaxi(vector<int> &cars);

void assignTaxi(vector<pair<double, int>> &cars);

void assignTaxiFirst(vector<pair<double, int>> &cars);

void updateDriverArr(Worker &w);

// related to grid index
double gridDist(int a, int b);

int getGridID(int x);

int getGridID(double x, double y);

double centerDist(int gid, int pid);

void updateGrid(int pid, int wid, int tag);

void initGrid();

// related to filter operator
vector<int> single_search(int s, double ddl);

// related to insertion operator
double det(int x, int y, int z);

double det(int a, int b, int c, int d);

void try_insertion_euclid(Worker &w, int rid, double &delta);

void try_insertion(const Worker &w, int rid, double &delta);

void insertion(Worker &w, int rid);

vector<Worker> insertion_all(const Worker &w, int rid);

#ifdef DUMP_ROUTE
void dumpRouteByWorker(int wid);
#endif
#endif