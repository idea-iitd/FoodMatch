#include <bits/stdc++.h>
#include "event.hpp"
#include "vehicle.hpp"
#include "routeplan.hpp"
#include "global.hpp"
#include "config.hpp"
#include "graph_util.hpp"
#include "hhl_query.hpp"
#include "constants.hpp"
#include <stdexcept>

using namespace std;

// check if pick up of an order is before delivery for all orders in the route plan
bool valid_route_plan(vector<event> &route_plan){
    unordered_set<string> order_ids;
    for (int i = 0; i < int(route_plan.size()); i++){
        // If event is pickup and orderid is already in order_ids (Delivered First), invalid permutation
        if (route_plan[i].type == 0){
            if (order_ids.find(route_plan[i].order_obj.order_id)!=order_ids.end())
                return false;
        }
        else
            order_ids.insert(route_plan[i].order_obj.order_id);
    }
    return true;
}

void generate_route_plans(vector<vector<event> > &answer, vector<event> &route_plan, int l, int r){
    int n = route_plan.size();
    if (l == r){
        vector<event> toadd;
        for(int i = 0; i < n; i++)
            toadd.push_back(route_plan[i]);
        if (valid_route_plan(toadd))
            answer.push_back(toadd);
        return;
    }
    for (int i = l; i <= r; i++){
        swap(route_plan[l], route_plan[i]);
        generate_route_plans(answer, route_plan, l+1, r);
        swap(route_plan[l], route_plan[i]); //backtrack
    }
}

bool check_capacity_constraint(vehicle &vh, vector<event> &route_plan){
    // Contains all picked up items
    long long int curr_load = vh.curr_load;
    for (int i = 0; i < int(route_plan.size()); i++){
        // If Pickup, increase item count
        if (route_plan[i].type == 0){
            curr_load += route_plan[i].order_obj.items;
            if (curr_load > vh.max_load)
                return false;
        }
        // If Delivery, decrease item count
        else
            curr_load -= route_plan[i].order_obj.items;
    }
    return true;
}

unordered_map<string, double> get_delivered_times(vehicle &vh, vector<event> &route_plan, double curr_time){
    unordered_map<string, double> delivered_time;
    double global_time = curr_time;
    long long int curr_node = vh.get_current_location();
    // If in middle of an edge, complete the edge
    if (vh.fraction_last_edge > 0.1){
        long long int u = vh.path[vh.path_present_idx];
        long long int v = vh.path[vh.path_present_idx + 1];
        int curr_timeslot = ((((long long int)(curr_time))%86400 + 86400)%86400)/3600;
        double edge_length = get_edge_weight(u, v, curr_timeslot);
        global_time = curr_time + (1 - vh.fraction_last_edge)*edge_length;
        curr_node = vh.path[vh.path_present_idx + 1];
    }
    for (int i = 0; i < int(route_plan.size()); i++){
        order event_order = route_plan[i].order_obj;
        double time_taken = hhl_sp_query(curr_node, route_plan[i].node);
        if (time_taken + FP_EPSILON >= MAX_NUM){
            // if any two event nodes in route plan were not reachable, return empty delivered times
            return {};
        }
        global_time += time_taken;
        // Pick Up
        if (route_plan[i].type == 0){
            double food_prep_time = event_order.order_time + event_order.prep_time;
            global_time = max(global_time, food_prep_time);
        }
        else
            delivered_time[event_order.order_id] = global_time;
        curr_node = route_plan[i].node;
    }
    return delivered_time;
}

double get_route_plan_extra_delivery_time(unordered_map<string, double> &delivered_time, vector<event> &route_plan){
    double extra_delivery_time = 0;
    for (int i = 0; i < int(route_plan.size()); i++){
        if (route_plan[i].type == 1){
            order event_order = route_plan[i].order_obj;
            extra_delivery_time += delivered_time[event_order.order_id] - event_order.order_time - event_order.shortest_delivery_time;
        }
    }
    return extra_delivery_time;
}

pair<pair<vector<event>, double>, unordered_map<string, double>> get_best_route_plan(vehicle &vh,
                                                            order &new_order, double curr_time){
    double min_cost = MAX_NUM;
    vector<event> min_plan;
    unordered_map<string, double> min_delivery_times;
    vector<vector<event>> all_route_plans;
    vector<event> org_route_plan = vh.route_plan;
    vector<event> route_plan = vh.route_plan;
    route_plan.push_back(event(new_order, 0));
    route_plan.push_back(event(new_order, 1));
    generate_route_plans(all_route_plans, route_plan, 0, route_plan.size() - 1);
    unordered_map<string, double> org_d_times = get_delivered_times(vh, org_route_plan, curr_time);
    double org_cost = get_route_plan_extra_delivery_time(org_d_times, org_route_plan);
    for(int i = 0; i < int(all_route_plans.size()); i++){
        bool cap_compatible = check_capacity_constraint(vh, all_route_plans[i]);
        unordered_map<string, double> d_times = get_delivered_times(vh, all_route_plans[i], curr_time);
        // empty delivery times implies two event nodes in plan were not reachable
        if (d_times.empty())
            continue;
        if (cap_compatible){
            double plan_cost = get_route_plan_extra_delivery_time(d_times, all_route_plans[i]) - org_cost;
            if (plan_cost < min_cost){
                min_cost = plan_cost;
                min_plan = all_route_plans[i];
                min_delivery_times = d_times;
            }
        }
    }
    return {{min_plan, min_cost}, min_delivery_times};
}
