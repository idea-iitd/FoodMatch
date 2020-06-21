#pragma once

#include <bits/stdc++.h>
#include "order.hpp"

using namespace std;

// Perform Hungarian Assignment
void hungarian_assignment(vector<int> &active_vehicles, vector<order> &active_orders,
                          double global_time, vector<order> &rejected_orders);

// Perform Greedy Assignment
void greedy_accumulate_assignment(vector<int> &active_vehicles, vector<order> &active_orders,
                                  double global_time, vector<order> &rejected_orders);
