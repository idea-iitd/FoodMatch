#include <bits/stdc++.h>
#include "route_recommendation.hpp"
#include "global.hpp"
#include "graph_util.hpp"
#include "constants.hpp"

using namespace std;

vector<long long int> dijkstra_lengths(long long int N, long long int S, long long int D,
                                       vector<double> &distance_from_source, double currtime){

    vector<long long int> prev_node(N);
    vector<int> visited(N, 0);
    for(int i = 0; i < N; i++){
        distance_from_source[i] = MAX_NUM;
        prev_node[i] = -1;
    }
    distance_from_source[S] = 0;
    priority_queue<pair<double, long long int>> dj;
    dj.push(make_pair(0, S));
    pair<double, long long int> x;
    long long int u, v;
    double alt;
    while(!dj.empty()){
        x = dj.top();
        dj.pop();
        u = x.second;
        if (visited[u] == 1)
            continue;
        visited[u] = 1;
        if (u == D)
            break;
        int timeslot = ((((long long int)(currtime + distance_from_source[u]))%86400 + 86400)%86400)/3600;
        for(int i = 0; i < int(edges[u].size()); i++) {
            v = edges[u][i];
            // FIFO assumption
            alt = distance_from_source[u] + get_edge_weight(u, v, timeslot);
            if (alt < distance_from_source[v]){
                distance_from_source[v] = alt;
                dj.push(make_pair(-alt, v));
                prev_node[v] = u;
            }
        }
    }
    for(int i = 0; i < N; i++)
    { 	if( distance_from_source[i] + FP_EPSILON >= MAX_NUM ) {
            distance_from_source[i] = MAX_NUM;
            prev_node[i] = -1;
        }
    }
    vector<long long int> path;
    long long int node = D;
    while(true)
    {
        path.push_back(node);
        if ((node == S) || (prev_node[node] == -1))
            break;
        node = prev_node[node];
    }
    reverse(path.begin(), path.end());
    return path;
}