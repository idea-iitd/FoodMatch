#pragma once

#include <bits/stdc++.h>
#include "event.hpp"
#include "vehicle.hpp"

using namespace std;

// Cost function
double get_route_plan_extra_delivery_time(unordered_map<string, double> &delivered_time, vector<event> &route_plan);

// Given a vehicle and routeplan at time curr_time,
// returns the delivered times of orders in the routeplan if vehicle follows it
unordered_map<string, double> get_delivered_times(vehicle &vh, vector<event> &route_plan, double curr_time);