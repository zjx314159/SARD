
#include "global.h"

int numOfVertices = 0;
int numOfEdges = 0;
const double clockLimit = 12 * 3600 * CLOCKS_PER_SEC;
const int INF_WEIGHT = std::numeric_limits<int>::max() / 3;
const double EPS = 1e-6;
const double INF = INF_WEIGHT;
const double speed = 1;
const double MAX_speed = 10 * speed;
const int CACHE_SIZE = 1000000;
long ssspCounter = 0;
int usedMemory = 0;
double usedTime = 0.0;
vector<pair<int,long>> schedule[5000];

int dcmp(double x) {
    if (fabs(x) < EPS)
        return 0;
    return x > 0 ? 1 : -1;
}

double eucDist(const vertex &a, const vertex &b) {
    return sqrt(1.0 * (a.x - b.x) * (a.x - b.x) + 1.0 * (a.y - b.y) * (a.y - b.y));
}

