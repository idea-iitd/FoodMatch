#include <bits/stdc++.h>
#include "global.hpp"
#include "order.hpp"
#include "vehicle.hpp"
#include "config.hpp"

using namespace std;

vector<pair<double, double> > nodes_to_latlon;
vector<long long int> node_id;
unordered_map<long long int, long long int> id_to_node;

vector<vector<long long int> > edges;
vector<vector<double> > edge_weight;
vector<vector<double> > edge_dist;
vector< vector<double> > slotted_edge_speed[24];
vector< vector<double> > slotted_edge_speed_std[24];
vector<double> slotted_max_time;

vector<order> realtime_orders;

unordered_map<string, double> rest_prep_time;
unordered_map<string, double> slotted_rest_prep_time[24];
unordered_map<string, double> slotted_rest_prep_time_std[24];

vector<vehicle> all_vehicles;

config global_conf;

unordered_map<long long int, double> time_opt;