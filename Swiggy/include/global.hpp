#include <bits/stdc++.h>
#include "order.hpp"
#include "vehicle.hpp"
#include "config.hpp"

using namespace std;

/* nodes global structure */
// node - 0 indexed nodes
// ID - node ID
// node to latitude/longitude
extern vector< pair<double, double> > nodes_to_latlon;
// node to ID
extern vector<long long int> node_id;
// ID to node
extern unordered_map<long long int, long long int> id_to_node;

/* Edges global structure */
// Edges Adjacency List
extern vector< vector<long long int> > edges;
// Mean(full day) time to travel the edge in seconds
extern vector< vector<double> > edge_weight;
// Length of edge in meters
extern vector<vector<double> > edge_dist;
// Mean edge speed(per hour) in meter/sec per hour of day
extern vector< vector<double> > slotted_edge_speed[24];
// Std Dev. of edge speed(per hour) in meter/sec per hour of day
extern vector< vector<double> > slotted_edge_speed_std[24];
extern vector<double> slotted_max_time;

/* Order Structures */
// list of all orders for the day
extern vector<order> realtime_orders;

// restaurant id to mean preparation time (full day)
extern unordered_map<string, double> rest_prep_time;
// restaurant id to mean preparation time (per hour)
extern unordered_map<string, double> slotted_rest_prep_time[24];
// restaurant id to std. dev. preparation time (per hour)
extern unordered_map<string, double> slotted_rest_prep_time_std[24];

// List of all vehicles for the day
extern vector<vehicle> all_vehicles;

// Global configuration file
extern config global_conf;

extern unordered_map<long long int, double> time_opt;