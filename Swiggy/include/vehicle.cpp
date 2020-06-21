#include <bits/stdc++.h>
#include "order.hpp"
#include "global.hpp"
#include "route_recommendation.hpp"
#include "vehicle.hpp"
#include "routeplan.hpp"
#include "graph_util.hpp"
#include "constants.hpp"

using namespace std;

active_interval::active_interval(){
}

active_interval::active_interval(double st_time, double en_time, long long int st_node){
    this->start_time = st_time;
    this->end_time = en_time;
    this->start_node = st_node;
}

vehicle::vehicle() {
}

vehicle::vehicle(string v_id, long long int m_load, vector<active_interval> de_int){
    this->vehicle_id = v_id;
    this->curr_load = 0;
    this->max_load = m_load;
    this->de_intervals = de_int;
    this->e_d = (this->de_intervals)[0].start_time;
    this->ONLINE = false;
}

void vehicle::reroute(double currtime) {
    if ((this->route_plan).empty()) return;
    long long int start_node = get_current_location();
    // If in middle of an edge, start from next node
    int insert_at_begin = 0;
    if (this->fraction_last_edge > 0.1){
        start_node = (this->path)[this->path_present_idx + 1];
        insert_at_begin = 1;
    }
    vector<double> distance_from_source(node_id.size());
    vector<long long int> shortest_path;
    // Note (this->path) always exists
    // Do a djikstra
    shortest_path = dijkstra_lengths(node_id.size(), start_node, (this->route_plan)[0].node,
                                     distance_from_source, currtime);
    // if the vehicle was between an edge
    if (insert_at_begin == 1){
        shortest_path.insert(shortest_path.begin(), (this->path)[this->path_present_idx]);
    }
    this->path = shortest_path;
    this->path_present_idx = 0;
}

bool vehicle::process_event(event event_obj, double &next_time, double &time_left){
    order event_order = event_obj.order_obj;
    // Pick Up
    if (event_obj.type == 0){
        if (VERBOSITY == -1){
            cout << fixed << "REACHED," << event_order.order_id << "," << next_time-time_left << endl;
        }
        double food_prepared_time = event_order.order_time + event_order.prep_time;
        // food is already prepared
        if (next_time > food_prepared_time) {
            // If the driver reaches the restaurant after food is prepared - time_left
            // else next_time - food_prepared_time
            time_left = min(time_left, next_time - food_prepared_time);
            if (VERBOSITY == -1){
                cout << fixed << "PICKEDUP," << event_order.order_id << "," << next_time-time_left << endl;
            }
            this->curr_load += event_order.items;
            return true;
        }
        else {
            time_left = 0;
            return false;
        }
    }
    // Deliver
    else {
        int j;
        for (j=0; j < int((this->order_set).size()); j++) {
            if ((this->order_set)[j].order_id.compare(event_order.order_id) == 0)
                break;
        }
        if (VERBOSITY==-1){
            cout << fixed << "DELIVER,"<<event_order.order_id << "," << next_time-time_left << ","<<(this->vehicle_id) <<endl;
        }
        (this->order_set).erase( (this->order_set).begin() + j);
        this->curr_load -= event_order.items;
        return true;
    }
}

bool vehicle::move(double curr_time, double next_time){
    double time_left = next_time - curr_time;
    // The vehicle is idle
    if ((this->route_plan).empty()){
        return false;
    }
    // If current node is Pickup/Delivery node, complete the event
    // while because there can be multiple events at current node
    bool is_pickup_delivery_complete = false;
    while (!(this->route_plan).empty() && time_left > 0) {
        // Not at a Pickup/Delivery Node
        if ((this->route_plan)[0].node != get_current_location())
            break;
        event e = (this->route_plan)[0];
        is_pickup_delivery_complete = process_event(e, next_time, time_left);
        if (is_pickup_delivery_complete) {
            (this->route_plan).erase((this->route_plan).begin());
        }
    }
    // if some events were completed, the driver must be rerouted to next event
    if (is_pickup_delivery_complete){
        reroute((next_time - time_left));
    }
    // Begin Moving
    while (!(this->route_plan).empty() && time_left > 0){
        assert((this->path_present_idx < (int((this->path).size())-1)) || !(cerr << "False: " << this->path_present_idx << " " <<
                                                                        (this->path).size()-1 << " " << is_pickup_delivery_complete << " "
                                                                         << this->route_plan_str() << " " << this->path[(this->path).size()-1] << " "
                                                                         << this->get_current_location() << " " << this->vehicle_id << endl));
        long long int u = (this->path)[this->path_present_idx];
        long long int v = (this->path)[this->path_present_idx + 1];
        int timeslot = ((((long long int)(next_time - time_left))%86400 + 86400)%86400)/3600;
        double time_to_next_node = get_edge_weight(u, v, timeslot);
        int carry_orders = (this->order_set).size()*2 - (this->route_plan).size();
        // Move on the Partial edge
        if (time_to_next_node * (1 - fraction_last_edge) >= time_left){
            if (VERBOSITY == -1){
                // Haversine used because of straight edge
                double dist = node_haversine(u, v) * (time_left/time_to_next_node);
                cout << "MOVE,"<< this->vehicle_id << "," << (this->route_plan)[0].order_obj.order_id << ",";
                cout << next_time - time_left << "," << dist << "," << carry_orders << "," <<  time_left << "," << (this->route_plan)[0].type << endl;
            }
            this->fraction_last_edge += time_left/time_to_next_node;
            time_left = 0;
        }
        // Move on Full Edge
        else {
            if (VERBOSITY == -1){
                double dist = node_haversine(u, v) * (1 - this->fraction_last_edge);
                cout << "MOVE,"<< this->vehicle_id << "," << (this->route_plan)[0].order_obj.order_id << ",";
                cout << next_time - time_left << "," << dist << "," << carry_orders << ",";
                cout <<  time_to_next_node * (1 - this->fraction_last_edge) << "," << (this->route_plan)[0].type  << endl;
            }
            time_left -= time_to_next_node * (1 - this->fraction_last_edge);
            this->fraction_last_edge = 0;
            this->path_present_idx += 1;
            // Can recurse
            // If current node is Pickup/Delivery Node, complete the event
            // while because there can be multiple events at current node
            is_pickup_delivery_complete = false;
            while (!(this->route_plan).empty() && time_left > 0) {
                // Not at a Pickup/Delivery Node
                if ((this->route_plan)[0].node != get_current_location())
                    break;
                event e = (this->route_plan)[0];
                is_pickup_delivery_complete = process_event(e, next_time, time_left);
                if (is_pickup_delivery_complete) {
                    (this->route_plan).erase((this->route_plan).begin());
                }
            }
            // if some events were completed, the driver must be rerouted to next event
            if (is_pickup_delivery_complete){
                reroute(next_time - time_left);
            }
        }
    }
    return true;
}

void vehicle::assign_order(order new_order, vector<event> plan, double currtime){
    // assign route plan and reroute
    this->route_plan = plan;
    reroute(currtime);
    // update order set
    (this->order_set).push_back(new_order);
}

void vehicle::assign_order_pack(vector<order> new_pack, vector<event> plan, double currtime){
    // assign route plan and reroute
    this->route_plan = plan;
    reroute(currtime);
    // update order set
    for (int i = 0; i < int(new_pack.size()); i++)
        (this->order_set).push_back(new_pack[i]);
}

void vehicle::set_e_d(double free_time){
    this->e_d = free_time;
}

long long int vehicle::get_current_location() {
    return (this->path)[this->path_present_idx];
}

string vehicle::route_plan_str(){
    string plan_str = "";
    for (int i = 0; i < int((this->route_plan).size()); i++){
        plan_str += (this->route_plan)[i].str_val();
        plan_str += "|";
    }
    return plan_str;
}

long long int vehicle::is_active(double curr_time){
    for(int i = 0; i < int((this->de_intervals).size()); i++){
        active_interval de_int = (this->de_intervals)[i];
        if (de_int.start_time - FP_EPSILON < curr_time && de_int.end_time + FP_EPSILON > curr_time)
            return de_int.start_node;
    }
    return -1;
}

void vehicle::make_online(long long int c_node){
    // offline vehicle must not have orders and events to complete
    assert((this->route_plan).size() == 0);
    assert((this->path).size() == 0);
    assert((this->order_set).size() == 0);
    (this->path).push_back(c_node);
    this->path_present_idx = 0;
    this->fraction_last_edge = 0;
    this->e_d = (this->de_intervals)[0].start_time;
    this->ONLINE = true;
}

void vehicle::make_offline(){
    // to be made offline vehicle must not have orders and events to complete
    assert(((this->route_plan).size() == 0) || !(cerr << this->vehicle_id << endl));
    assert(((this->order_set).size() == 0) || !(cerr << this->vehicle_id << endl));
    this->path = vector<long long int>();
    this->ONLINE = false;
}
