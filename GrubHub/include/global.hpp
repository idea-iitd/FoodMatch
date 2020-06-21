#include <bits/stdc++.h>
#include "order.hpp"
#include "vehicle.hpp"

// Folder name of public instance
extern string data_location;

// Realtime orders list
extern vector<order> realtime_orders;

// Vehicles list
extern vector<vehicle> all_vehicles;

// <x,y> positions of restaurants
extern unordered_map<string, pair<double, double>> restaurant_id_to_coord;

// all <x,y> positions given as input
extern vector<pair<double, double>> all_loc;

// maximum travel distance between input <x,y> locations
extern int max_travel_dist;