#include <bits/stdc++.h>
#include "global.hpp"
#include "route_recommendation.hpp"
#include "constants.hpp"

using namespace std;

mt19937 rng(global_conf.random_seed);

double node_haversine(long long int u, long long int v){
    double pi = 3.14159265358979323846;
    double lat1 = nodes_to_latlon[u].first * (pi/180);
    double lon1 = nodes_to_latlon[u].second * (pi/180);
    double lat2 = nodes_to_latlon[v].first * (pi/180);
    double lon2 = nodes_to_latlon[v].second * (pi/180);
    double dlon = lon2 - lon1;
    double dlat = lat2 - lat1;
    double a = pow(sin(dlat/2), 2) + cos(lat1)*cos(lat2)*pow(sin(dlon/2), 2);
    double c = 2 * asin(sqrt(a));
    double r = 6378137;
    return c*r;
}

double get_edge_weight(long long int u, long long int v, int timeslot){
    double speed=MAX_NUM, speed_std=MAX_NUM, dist=MAX_NUM;
    for (int i = 0; i < int(edges[u].size()); i++){
        if (edges[u][i] == v){
            speed = slotted_edge_speed[timeslot][u][i];
            speed_std = slotted_edge_speed_std[timeslot][u][i];
            dist = edge_dist[u][i];
            break;
        }
    }
    if (speed + FP_EPSILON >= MAX_NUM ||
        speed_std + FP_EPSILON >= MAX_NUM ||
        dist + FP_EPSILON >= MAX_NUM){
            throw "SOME ERROR HERE\n";
        }
    return dist/speed;
}

double query_naive(int x, int y, double currtime){
    vector< double > distance_from_source(node_id.size());
    vector<long long int> path = dijkstra_lengths(node_id.size(), x, y,
                                                  distance_from_source, currtime);
    return distance_from_source[y] ;
}
