#include <bits/stdc++.h>
#include "global.hpp"
#include "order.hpp"
#include "vehicle.hpp"

string data_location;

vector<order> realtime_orders;

vector<vehicle> all_vehicles;

unordered_map<string, pair<double, double>> restaurant_id_to_coord;

vector<pair<double, double>> all_loc;

int max_travel_dist;