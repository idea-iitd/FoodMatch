#pragma once

#include <bits/stdc++.h>
#include "order.hpp"
#include "event.hpp"

using namespace std;

// Class of vehicles
class vehicle{
private:

    // Process Pickup/Delivery events if restaurant/customer node is reached
    // Returns true if event is finished, false otherwise and updates time_left
    // Handles service times at restaurant/customer locations
    bool process_event(event event_obj, double &next_time, double &time_left);

    // sets next destination of a vehicles
    void reroute();

public:
    // unique vehicle id
    string vehicle_id;

    // Duty times of a vehicle
    double on_time, off_time;

    // start location
    double pos_x, pos_y;

    // previous event location
    string prev_location_id;
    double prev_departure_time, prev_x, prev_y;

    // current destination
    double next_x, next_y;

    // Inside restuarant/customer location
    bool inside_rest;
    double rest_going_in;
    double rest_going_out;

    bool inside_cust;
    double cust_going_in;

    // courier free time
    double e_d;

    // // list of orders objects
    int order_count;

    // list of event objects to follow
    vector<event> route_plan;

    // whether vehicle is currently online
    bool ONLINE;

    vehicle();

    vehicle(string v_id, double v_x, double v_y, double on_t, double off_t);

    // Simulate vehicle movement from curr_time to next_time, return false if idle
    void move(double curr_time, double next_time);

    // Assign new order and route plan to vehicle
    void assign_order(vector<event> plan);

    // Assign new order_pack and route plan to vehicle
    void assign_order_pack(int new_pack_size, vector<event> plan);

    // courier free time
    void set_e_d(double free_time);

    // Get vehicle current node index
    bool at_location(double x, double y);

    // Route Plan as a string
    string route_plan_str();

    // Return start_node of active interval if DE is active else -1
    bool is_active(double curr_time);

    // Bring an offline vehicle online at curr_node
    void make_online();

    // Make an online vehicle offline at curr_node
    void make_offline();
};

