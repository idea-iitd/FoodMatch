#include <bits/stdc++.h>

using namespace std;

// general constants

// Maximum number in simulation
extern const double MAX_NUM;
// Floating point epsilon in simulation
extern const double FP_EPSILON;
// Set to -1 to generate output for evaluation
extern int VERBOSITY;

// Size of accumulation window in seconds
extern double delta_time;

// Dataset parameters
extern double vehicle_speed;
extern double pickup_service;
extern double dropoff_service;
extern double target_ctd;
extern double max_ctd;
extern double pay_per_order;
extern double pay_per_hour;

// Assignment algorithm to choose
// MDRP : Default algorithm by Reyes et. al.
// FM : FoodMatch
extern string assignment_algorithm;

// MDRP parameters defined by Reyes et. al.
extern double delta_u;
extern double delta_1;
extern double delta_2;
extern double service_loss_tol;
extern double freshness_loss_tol;
extern double freshness_commit;
extern double util_coeff;
extern double freshness_loss_coeff;
extern double service_loss_coeff;
extern int Z_t_default;

// FoodMatch parameters
// eta value
extern double max_merge_cost_edt;
// K value
extern int vehicle_explore_frac;
// Gamma value
extern double heuristic_multiplier;
// MAXO value
extern int max_o_count;