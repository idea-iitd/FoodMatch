#pragma once

#include <bits/stdc++.h>
#include "order.hpp"
#include "vehicle.hpp"

// Change online offline status of vehicles and return number of online vehicles
// at global_time(UNIX timestamp in sec)
int update_online_status(double global_time);

// get orders received delta time before global_time(UNIX timestamp in sec)
// include or completely reject orders rejected in previous rounds
vector<order> get_delta_orders(double global_time, vector<order> &rejected_orders, int &count_rejections);

// get active vehicles that can deliver current set of orders
vector<int> get_active_vehicle_indices(double global_time, vector<order> &active_orders);