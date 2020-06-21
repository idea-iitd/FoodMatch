#pragma once

#include <bits/stdc++.h>
#include "order.hpp"

using namespace std;

// Implementation of "Default" configuration algoritm as described by Reyes et. al.
void mdrp_assignment(vector<int> &active_vehicles, vector<order> &active_orders,
                          double global_time, vector<order> &rejected_orders);