#pragma once

#include <bits/stdc++.h>

using namespace std;

// Haversine distance between two nodes
double node_haversine(long long int u, long long int v);

// return edge weight(time to travel) between two nodes at that timeslot
// u-v edge
// timeslot - 0-23 hour of day at which weight is required
double get_edge_weight(long long int u, long long int v, int timeslot);

// Takes in node index, returns the shortest distance between x and y using djikstra at currtime
double query_naive(int x, int y, double currtime);
