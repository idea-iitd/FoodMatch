#include <bits/stdc++.h>
#include "event.hpp"
#include "vehicle.hpp"
#include "routeplan.hpp"
#include "global.hpp"
#include "constants.hpp"
#include "util.hpp"
#include <stdexcept>

using namespace std;

// Given vehicle object, route plan to follow and current time
// returns delivered times of the orders in route_plan
// Empty if not feasible
unordered_map<string, double> get_delivered_times(vehicle &vh, vector<event> &route_plan, double curr_time){
    unordered_map<string, double> delivered_time;
    double global_time = curr_time;
    double curr_x = vh.pos_x;
    double curr_y = vh.pos_y;
    string prev_id = vh.prev_location_id;
    // Handle service times
    if (route_plan.size() > 0){
        if (prev_id != route_plan[0].loc_id){
            if (vh.inside_rest){
                global_time += vh.rest_going_out;
            }
            else{
                if (vh.inside_cust){
                    throw invalid_argument("SOME ERROR\n");
                }
                else{
                    // continue;
                }
            }
        }
        else{
            if (route_plan[0].type == 0){
                global_time += vh.rest_going_in;
            }
            else{
                global_time += (vh.cust_going_in - dropoff_service/2.0);
            }
        }
    }
    for (int i = 0; i < int(route_plan.size()); i++){
        order event_order = route_plan[i].order_obj;
        double time_taken = get_travel_time(curr_x, curr_y, route_plan[i].x, route_plan[i].y);
        global_time += time_taken;
        // Pick Up
        if (route_plan[i].type == 0){
            if (route_plan[i].loc_id != prev_id){
                global_time += pickup_service/2.0;
            }
            global_time = max(global_time, event_order.ready_time);
            // pickup after off time
            if (!vh.is_active(global_time)){
                return {};
            }
            if (route_plan[i].loc_id != route_plan[i+1].loc_id){
                global_time += pickup_service/2.0;
            }
        }
        else{
            if (route_plan[i].loc_id != prev_id){
                global_time += dropoff_service/2.0;
            }
            delivered_time[event_order.order_id] = global_time;
            global_time += dropoff_service/2.0;
            // max ctd violation
            if ((global_time - dropoff_service/2.0) - event_order.order_time > max_ctd){
                return {};
            }
        }
        curr_x = route_plan[i].x;
        curr_y = route_plan[i].y;
        prev_id = route_plan[i].loc_id;
    }
    return delivered_time;
}

double get_route_plan_extra_delivery_time(unordered_map<string, double> &delivered_time, vector<event> &route_plan){
    double extra_delivery_time = 0;
    for (int i = 0; i < int(route_plan.size()); i++){
        if (route_plan[i].type == 1){
            order event_order = route_plan[i].order_obj;
            double del_time = delivered_time[event_order.order_id] - event_order.order_time;
            double shortest_del_time = event_order.shortest_delivery_time - event_order.order_time;
            extra_delivery_time += del_time - shortest_del_time;
        }
    }
    return extra_delivery_time;
}