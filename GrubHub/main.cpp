#include <bits/stdc++.h>
#include "global.hpp"
#include "order.hpp"
#include "event.hpp"
#include "vehicle.hpp"
#include "food_data_util.hpp"
#include "simulation_util.hpp"
#include "vehicle_assignment.hpp"
#include "mdrp_assignment.hpp"
#include "constants.hpp"
#include <stdexcept>

using namespace std;

int ORDER_REJECTIONS = 0;

void simulation(){
    double global_time;
    vector<order> rejected_orders;
    // Run the simulation from start of day (0 seconds) till end of day(86400 seconds) plus buffer (4 hours)
    for(global_time = 0.0; global_time < 28*60*60 ; global_time += delta_time){
        // Change online offline status of vehicles
        int count_online = update_online_status(global_time);
        // get incoming orders in current round
        vector<order> active_orders = get_delta_orders(global_time, rejected_orders, ORDER_REJECTIONS);
        // // get active vehicles that can deliver current set of orders
        vector<int> active_vehicles = get_active_vehicle_indices(global_time);
        // Perform Assignment
        if (assignment_algorithm == "MDRP"){
            mdrp_assignment(active_vehicles, active_orders, global_time, rejected_orders);
        }
        else if (assignment_algorithm == "FM"){
            foodmatch(active_vehicles, active_orders, global_time, rejected_orders);
        }
        else{
            throw invalid_argument("ASSIGNMENT ALGORITHM NOT RECOGNISED\n");
            return;
        }
        // Move vehicles
        for(int i = 0; i < int(all_vehicles.size()); i++){
            vehicle* vh = &all_vehicles[i];
            vh->move(global_time, global_time + delta_time);
        }
    }
    cout << ORDER_REJECTIONS << "/" << realtime_orders.size() << endl;
}

void usage() {
    cout << "Usage: ./main.o [path_to_instance_location] ALGO" << endl;
    exit(1);
}

int main(int argc, char *argv[]){
    data_location = argv[1];
    assignment_algorithm = argv[2];
    read_instance_parameters();
    cout << "Instance parameters read" << endl;
    read_rest_data();
    cout << "Restaurant location read" << endl;
    read_order_data();
    cout << "Finish taking order input \nOrders = " << realtime_orders.size() << endl;
    vehicle_init();
    cout << "Finish initialising vehicle\nVehicles = " << all_vehicles.size() << endl;
    auto start = std::chrono::high_resolution_clock::now();
    simulation();
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(stop - start);
    cout << "Time taken by Simulation: " << duration.count()/60 << " minutes" << endl;
    return 0;
}