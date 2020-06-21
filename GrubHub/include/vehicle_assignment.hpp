#pragma once

#include <bits/stdc++.h>
#include "order.hpp"

using namespace std;

// Implementation of FoodMatch algorithm with all optimizations included
// and adapted for the given dataset (without road network)
void foodmatch(vector<int> &active_vehicles, vector<order> &active_orders,
               double global_time, vector<order> &rejected_orders);