#include <bits/stdc++.h>

using namespace std;

/* Constants */
// Maximum number in simulation
extern const double MAX_NUM;
// Floating point epsilon in simulation
extern const double FP_EPSILON;
// Set to -1 to generate output for evaluation
extern int VERBOSITY;
// 45 minute bound
extern double vehicle_rest_radius_cap;
// Time before rejection
extern double rejection_window;
// \Omega (penalty) in seconds
extern double max_cost_val;
// Fraction of drivers to use
extern double fraction_drivers;
// Algorithm Name
extern string assignment_algorithm;
// MAXO
extern int batching_cap;

/* FoodMatch parameters */
// eta
extern double max_merge_cost_edt;
// K
extern int vehicle_explore_frac;
// BFS limit
extern double vehicle_explore_rad;
// Gamma
extern double heuristic_multiplier;

/* Simulation */
// Day of simulation
extern string simulation_day;
// City of simulation
extern string simulation_city;
// Accumulation Window
extern double delta_time;
// Start time of simulation in seconds
extern double start_time;
// End time of simulation in seconds
extern double end_time;