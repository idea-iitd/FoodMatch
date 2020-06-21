#include <bits/stdc++.h>
#include "global.hpp"
#include "simulation_util.hpp"
#include "order.hpp"
#include "vehicle.hpp"
#include "graph_util.hpp"
#include "constants.hpp"
#include "hhl_query.hpp"

using namespace std;

int update_online_status(double global_time){
    int count_online = 0;
    for(int i = 0; i < int(all_vehicles.size()); i++){
        vehicle* vh = &all_vehicles[i];
        // Empty route plan
        // Case 1 : It was offline and remains offline
        // Case 2 : It was offline and becomes online at this time
        // Case 3 : It was online but idle
        long long int start_node = vh->is_active(global_time);
        // Vehicle is active in this interval
        if (start_node != -1){
            // If vehicle becomes active(some interval contains global_time)
            // Bring it online
            if (!(vh->ONLINE))
                vh->make_online(start_node);
        }
        // If it is not active
        else{
            // If route plan is empty make it offline
            if ((vh->route_plan).size() == 0)
                vh->make_offline();
            // Else let it complete the route plan
            // Don't count in online vehicles
            else
                continue;
        }
        count_online += int(vh->ONLINE);
    }
    return count_online;
}

vector<order> get_delta_orders(double global_time, vector<order> &rejected_orders, int &count_rejections){
    vector<order> active_orders;
    // Push orders that came in delta time
    if (global_time	 > start_time && global_time <= end_time){
        for(int i = 0; i < int(realtime_orders.size()); i++){
            if((realtime_orders[i].order_time >= global_time - delta_time) && (realtime_orders[i].order_time < global_time)){
                active_orders.push_back(realtime_orders[i]);
            }
        }
    }
    // Push previously rejected orders if within rejection window
    for(int i = 0; i < int(rejected_orders.size()); i++){
        if (rejected_orders[i].order_time + rejection_window > global_time)
            active_orders.push_back(rejected_orders[i]);
        else{
            if (VERBOSITY == -1)
                cout << fixed << "REJECT," << rejected_orders[i].order_id << endl;
            count_rejections++;
        }
    }
    rejected_orders.clear();
    cout << "AC ORDERS = " << active_orders.size() << endl;
    return active_orders;
}

vector<int> get_active_vehicle_indices(double global_time, vector<order> &active_orders){
    // Push all active vehicles
    vector<int> active_vehicles;
    for (int i = 0; i < int(all_vehicles.size()); i++){
        vehicle vh = all_vehicles[i];
        if(vh.is_active(global_time) == -1)
            continue;
        double min_vh_rest_dist = MAX_NUM;
        for(int i = 0; i < int(active_orders.size()); i++){
            min_vh_rest_dist =  min(min_vh_rest_dist, hhl_sp_query(vh.get_current_location(),
                                                                active_orders[i].restaurant.rest_node));
        }
        // Batching Capacity
        // Closest restuarant within radius capacity
        if (assignment_algorithm.substr(0,4) == "FM"){
              active_vehicles.push_back(i);
        }
        else{
            // Optimize hungarian and greedy
            if ((min_vh_rest_dist < vehicle_rest_radius_cap) && int(vh.order_set.size()) < batching_cap)
                active_vehicles.push_back(i);
        }
    }
    cout << "AC VH = " << active_vehicles.size() << endl;
    return active_vehicles;
}