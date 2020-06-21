#include <bits/stdc++.h>
#include "order.hpp"
#include "global.hpp"
#include "vehicle.hpp"
#include "routeplan.hpp"
#include "constants.hpp"
#include "util.hpp"

using namespace std;

vehicle::vehicle() {
}

vehicle::vehicle(string v_id, double v_x, double v_y, double on_t, double off_t){
    this->vehicle_id = v_id;
    this->pos_x = v_x;
    this->pos_y = v_y;
    this->on_time = on_t;
    this->off_time = off_t;
    this->prev_location_id = "0";
    this->ONLINE = false;
    this->inside_rest = false;
    this->inside_cust = false;
    this->rest_going_in = pickup_service/2.0;
    this->rest_going_out = pickup_service/2.0;
    this->prev_x = this->pos_x;
    this->prev_y = this->pos_y;
    this->e_d = on_t;
    this->order_count = 0.0;
    // managed in output
    this->cust_going_in = dropoff_service;
}

void vehicle::reroute(){
    if ((this->route_plan).empty()) return;
    this->next_x = (this->route_plan)[0].x;
    this->next_y = (this->route_plan)[0].y;
}

bool vehicle::process_event(event event_obj, double &next_time, double &time_left){
    order event_order = event_obj.order_obj;
    if((this->prev_location_id) != event_obj.loc_id){
        if (at_location(this->prev_x, this->prev_y)){
            this->prev_departure_time = next_time - time_left;
        }
        cout << "MDRP MOVE" << "," << this->vehicle_id << "," << this->prev_departure_time << ",";
        cout << this->prev_location_id << "," << event_obj.loc_id << endl;
        this->prev_location_id = event_obj.loc_id;
        this->prev_x = event_obj.x;
        this->prev_y = event_obj.y;
    }
    // Pick Up
    if (event_obj.type == 0){
        if (!(this->inside_rest)){
            if (time_left > (this->rest_going_in)){
                time_left -= (this->rest_going_in);
                this->rest_going_in = 0.0;
                this->inside_rest = true;
                if (VERBOSITY == -1){
                    cout << fixed << "REACHED," << event_order.order_id << "," << next_time-time_left << endl;
                }
            }
            else{
                this->rest_going_in -= time_left;
                time_left = 0;
                return false;
            }
        }
        if (VERBOSITY == -1){
            cout << fixed << "REACHED," << event_order.order_id << "," << next_time-time_left << endl;
        }
        // food is already prepared
        if (next_time > event_order.ready_time){
            // If the driver reaches the restaurant after food is prepared - time_left
            // else next_time - food_prepared_time
            time_left = min(time_left, next_time - event_order.ready_time);
            if (VERBOSITY == -1){
                // manage for a bundle in post processing
                cout << fixed << "PICKEDUP," << event_order.order_id << "," << next_time-time_left << "," << event_obj.loc_id << endl;
            }
            return true;
        }
        else{
            time_left = 0;
            return false;
        }
    }
    // Deliver
    else {
        if (!(this->inside_cust)){
            if (time_left > (this->cust_going_in)){
                time_left -= (this->cust_going_in);
                this->cust_going_in = 0.0;
                this->inside_cust = true;
            }
            else{
                this->cust_going_in -= time_left;
                time_left = 0;
                return false;
            }
        }
        if (VERBOSITY==-1){
            cout << fixed << "DELIVER,"<<event_order.order_id << "," ;
            cout << next_time-time_left - dropoff_service/2.0 << ","<<(this->vehicle_id) <<endl;
            // dropoff managed
        }
        this->order_count--;
        // reset stuff after completing delivery
        this->inside_rest = false;
        this->inside_cust = false;
        this->rest_going_in = pickup_service/2.0;
        this->rest_going_out = pickup_service/2.0;
        this->cust_going_in = dropoff_service;
        return true;
    }
}

void vehicle::move(double curr_time, double next_time){
    double time_left = next_time - curr_time;
    // The vehicle is idle
    if ((this->route_plan).empty()){
        return;
    }
    // If current node is Pickup/Delivery node, complete the event
    // while because there can be multiple events at current node
    bool is_pickup_delivery_complete = false;
    while (!(this->route_plan).empty() && time_left > 0) {
        // Not at a Pickup/Delivery Node
        if (!at_location((this->route_plan)[0].x, (this->route_plan)[0].y))
            break;
        event e = (this->route_plan)[0];
        is_pickup_delivery_complete = process_event(e, next_time, time_left);
        if (is_pickup_delivery_complete) {
            (this->route_plan).erase((this->route_plan).begin());
        }
    }
    // if some events were completed, the driver must be rerouted to next event
    if (is_pickup_delivery_complete){
        reroute();
    }
    if (this->inside_rest){
        if (time_left > (this->rest_going_out)){
            time_left -= (this->rest_going_out);
            this->rest_going_out = 0.0;
            this->inside_rest = false;
            // reset stuff after completing pickup
            this->inside_rest = false;
            this->inside_cust = false;
            this->rest_going_in = pickup_service/2.0;
            this->rest_going_out = pickup_service/2.0;
            this->cust_going_in = dropoff_service;

        }
        else{
            this->rest_going_out -= time_left;
            time_left = 0.0;
        }
    }
    // Begin Moving
    if (!(this->route_plan).empty() && time_left > 0){
        // If beginning to move from location id
        // set the departure time
        if (this->at_location(this->prev_x, this->prev_y)){
            this->prev_departure_time = next_time - time_left;
        }
        double time_to_destination = get_travel_time(pos_x, pos_y, next_x, next_y);
        int carry_orders = (this->order_count)*2 - (this->route_plan).size();
        // Move partially
        if (time_to_destination >= time_left){
            if (VERBOSITY == -1){
                double dist = get_distance(pos_x, pos_y, next_x, next_y) * (time_left/time_to_destination);
                cout << "MOVE,"<< this->vehicle_id << "," << (this->route_plan)[0].order_obj.order_id << ",";
                cout << next_time - time_left << "," << dist << "," << carry_orders << "," <<  time_left << "," << (this->route_plan)[0].type << endl;
            }
            pos_x += (next_x - pos_x)*(time_left/time_to_destination);
            pos_y += (next_y - pos_y)*(time_left/time_to_destination);
            time_left = 0.0;
        }
        // Move fully
        else {
            if (VERBOSITY == -1){
                double dist = get_distance(pos_x, pos_y, next_x, next_y);
                cout << "MOVE,"<< this->vehicle_id << "," << (this->route_plan)[0].order_obj.order_id << ",";
                cout << next_time - time_left << "," << dist << "," << carry_orders << ",";
                cout <<  time_to_destination << "," << (this->route_plan)[0].type  << endl;
            }
            time_left -= time_to_destination;
            pos_x = next_x;
            pos_y = next_y;
            this->move(next_time - time_left, next_time);
        }
    }
    return;
}

void vehicle::assign_order(vector<event> plan){
    // assign route plan and reroute
    this->route_plan = plan;
    reroute();
    this->order_count++;
}

void vehicle::assign_order_pack(int new_pack_size, vector<event> plan){
    // assign route plan and reroute
    this->route_plan = plan;
    reroute();
    this->order_count += new_pack_size;
}

void vehicle::set_e_d(double free_time){
    this->e_d = free_time;
}

bool vehicle::at_location(double x, double y){
    if ((fabs(x-pos_x) < FP_EPSILON) && (fabs(y-pos_y) < FP_EPSILON)){
        return true;
    }
    return false;
}

string vehicle::route_plan_str(){
    string plan_str = "";
    for (int i = 0; i < int((this->route_plan).size()); i++){
        plan_str += (this->route_plan)[i].str_val();
        plan_str += "|";
    }
    return plan_str;
}

bool vehicle::is_active(double curr_time){
    if (((this->on_time) - FP_EPSILON < curr_time) && ((this->off_time) + FP_EPSILON > curr_time))
        return true;
    return false;
}

void vehicle::make_online(){
    // offline vehicle must not have orders and events to complete
    assert((this->route_plan).size() == 0);
    assert((this->order_count) == 0);
    this->ONLINE = true;
    this->prev_location_id = "0";
    this->inside_rest = false;
    this->inside_cust = false;
    this->rest_going_in = pickup_service/2.0;
    this->rest_going_out = pickup_service/2.0;
    this->prev_x = this->pos_x;
    this->prev_y = this->pos_y;
    this->e_d = this->on_time;
    // managed in output
    this->cust_going_in = dropoff_service;
}

void vehicle::make_offline(){
    // to be made offline vehicle must not have orders and events to complete
    assert(((this->route_plan).size() == 0) || !(cerr << this->vehicle_id << endl));
    assert(((this->order_count) == 0) || !(cerr << this->vehicle_id << endl));
    this->ONLINE = false;
}
