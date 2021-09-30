//
// Created by heldon on 2020/8/9.
//

#ifndef SCS_MCMF_H
#define SCS_MCMF_H

#include <cmath>
#include <vector>
#include <iostream>

using namespace std;

// START
// from http://web.mit.edu/~ecprice/acm/acm08/MinCostMaxFlow.cc
// Min cost max flow algorithm using an adjacency matrix.  If you
// want just regular max flow, setting all edge costs to 1 gives
// running time O(|E|^2 |V|).
//
// Running time: O(min(|V|^2 * totflow, |V|^3 * totcost))
//
// INPUT: cap -- a matrix such that cap[i][j] is the capacity of
//               a directed edge from node i to node j
//
//        cost -- a matrix such that cost[i][j] is the (positive)
//                cost of sending one unit of flow along a
//                directed edge from node i to node j
//
//        source -- starting node
//        sink -- ending node
//
// OUTPUT: max flow and min cost; the matrix flow will contain
//         the actual flow values (note that unlike in the MaxFlow
//         code, you don't need to ignore negative flow values -- there
//         shouldn't be any)
//
// To use this, create a MinCostMaxFlow object, and call it like this:
//
//   MinCostMaxFlow nf;
//   int maxflow = nf.getMaxFlow(cap,cost,source,sink);

typedef vector<int> VI;
typedef vector<VI> VVI;
typedef vector<double> VB;
typedef vector<VB> VVB;

const double DOUBLE_INF = numeric_limits<double>::max() / 2.0;

struct MinCostMaxFlow {
    int N;
    VI dad, found;
    VVI cap, flow;
    VB dist, pi;
    VVB cost;

    bool search(int source, int sink) {
        fill(found.begin(), found.end(), false);
        fill(dist.begin(), dist.end(), DOUBLE_INF);
        dist[source] = 0;

        while (source != N) {
            int best = N;
            found[source] = true;
            for (int k = 0; k < N; k++) {
                if (found[k]) continue;
                if (flow[k][source]) {
                    double val = dist[source] + pi[source] - pi[k] - cost[k][source];
                    if (dist[k] > val) {
                        dist[k] = val;
                        dad[k] = source;
                    }
                }
                if (flow[source][k] < cap[source][k]) {
                    double val = dist[source] + pi[source] - pi[k] + cost[source][k];
                    if (dist[k] > val) {
                        dist[k] = val;
                        dad[k] = source;
                    }
                }

                if (dist[k] < dist[best]) best = k;
            }
            source = best;
        }
        for (int k = 0; k < N; k++)
            pi[k] = min(pi[k] + dist[k], INF);
        return found[sink];
    }

    pair<int, int> getMaxFlow(const VVI &cap_, const VVB &cost_, int source, int sink) {
        this->cap = cap_;
        this->cost = cost_;

        N = cap_.size();
        found = VI(N);
        flow = VVI(N, VI(N));
        dist = VB(N + 1);
        dad = VI(N);
        pi = VB(N);

        int totflow = 0, totcost = 0;
        while (search(source, sink)) {
            int amt = INF;
            for (int x = sink; x != source; x = dad[x])
                amt = min(amt, flow[x][dad[x]] ? flow[x][dad[x]] :
                               cap_[dad[x]][x] - flow[dad[x]][x]);
            for (int x = sink; x != source; x = dad[x]) {
                if (flow[x][dad[x]]) {
                    flow[x][dad[x]] -= amt;
                    totcost -= amt * cost_[x][dad[x]];
                } else {
                    flow[dad[x]][x] += amt;
                    totcost += amt * cost_[dad[x]][x];
                }
            }
            totflow += amt;
        }
        return make_pair(totflow, totcost);
    }
};
// END
#endif //SCS_MCMF_H
