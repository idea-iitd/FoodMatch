#pragma once

#include <bits/stdc++.h>
#include "order.hpp"
#include "event.hpp"

using namespace std;

// Intervals of activity for a DE
class active_interval{
public:

    // start and end of interval in UNIX Timestamp
    double start_time, end_time;

    // position of DE at start time
    long long int start_node;

    active_interval();
    active_interval(double st_time, double en_time, long long int st_node);
};

// Class of DE vehicles
class vehicle{
private:

    // Process Pickup/Delivery events if Restaurant/Customer node is reached
    // Returns true if event is finished, false otherwise (pickup must wait till food is prepared)
    // Updates time_left
    bool process_event(event event_obj, double &next_time, double &time_left);

    // Find path from current position to route_plan[0](pickup/deliver event) at currtime(UNIX timestamp in sec)
    void reroute(double currtime);

public:
    // unique vehicle id
    string vehicle_id;

    // list of orders objects
    vector<order> order_set;

    // list of event objects to follow
    vector<event> route_plan;

    // Intervals when DE is active
    vector<active_interval> de_intervals;

    // courier free time
    double e_d;

    // whether vehicle is currently online
    bool ONLINE;

    // current number of items with vehicle
    long long int curr_load;

    // maximum possible number of items with vehicle
    long long int max_load;

    // Node by node path until the next route_plan
    vector<long long int> path;

    // index into the path array (floor node)
    long long int path_present_idx;

    // fraction the cab has traversed in the last edge
    double fraction_last_edge;

    vehicle();

    vehicle(string v_id, long long int m_load, vector<active_interval> de_int);

    // Simulate vehicle movement from curr_time to next_time, return false if idle
    bool move(double curr_time, double next_time);

    // Assign new order and route plan to vehicle
    void assign_order(order new_order, vector<event> plan, double currtime);

    // Assign new order_pack and route plan to vehicle
    void assign_order_pack(vector<order> new_pack, vector<event> plan, double currtime);

    // courier free time
    void set_e_d(double free_time);

    // Get vehicle current node index
    long long int get_current_location();

    // Route Plan as a string
    string route_plan_str();

    // Return start_node of active interval if DE is active else -1
    long long int is_active(double curr_time);

    // Bring an offline vehicle online at curr_node
    void make_online(long long int c_node);

    // Make an online vehicle offline at curr_node
    void make_offline();
};

