#pragma once

#include <bits/stdc++.h>
#include "order.hpp"

using namespace std;

// Implementation of FoodMatch Batching and Reshuffling
void foodmatch_BR(vector<int> &active_vehicles, vector<order> &active_orders,
                  double global_time, vector<order> &rejected_orders);

// Implementation of FoodMatch Batching and Reshuffling + Best First Search
void foodmatch_BFS(vector<int> &active_vehicles, vector<order> &active_orders,
                   double global_time, vector<order> &rejected_orders);

// Implementation of Complete FoodMatch
void foodmatch_FULL(vector<int> &active_vehicles, vector<order> &active_orders,
                    double global_time, vector<order> &rejected_orders);
