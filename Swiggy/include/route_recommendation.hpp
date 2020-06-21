#include <bits/stdc++.h>

using namespace std;

// N - number of nodes in graph
// S - source node
// D - destination node
// distance_from_source - filled by function
// currtime - UNIX timestamp in sec
// returns S-D shortest path as vector of nodes
vector<long long int> dijkstra_lengths(long long int N, long long int S, long long int D,
                                       vector<double> &distance_from_source, double currtime);
