#include <bits/stdc++.h>
#include "constants.hpp"

using namespace std;

const double MAX_NUM = 1e7;
const double FP_EPSILON = 0.000001;
int VERBOSITY = -1;
double delta_time = 300.0;
double vehicle_speed;
double pickup_service;
double dropoff_service;
double target_ctd;
double max_ctd;
double pay_per_order;
double pay_per_hour;
string assignment_algorithm = "MDRP";

//mdrp parameters
double delta_u = 600;
double delta_1 = 600;
double delta_2 = 600;
// Mean value of parameter variations : exact not mentioned
double service_loss_tol = (200.0/10.0);
double freshness_loss_tol = (100.0/10.0);
double freshness_commit = 20.0 * 60.0;
double util_coeff = 5.5;
double freshness_loss_coeff = 0.5;
double service_loss_coeff = 0.5;
// 1 : not mentioned anywhere
int Z_t_default = 1;

// foodmatch parameters
double max_merge_cost_edt = 90;
int vehicle_explore_frac = 90;
double heuristic_multiplier = 0.5;
int max_o_count = 3;