#include <bits/stdc++.h>
#include "global.hpp"
#include "graph_util.hpp"
#include "graph_util.hpp"
#include "hungarian.hpp"
#include "routeplan.hpp"
#include "hhl_query.hpp"
#include "food_data_util.hpp"
#include "vehicle_assignment_baseline.hpp"
#include "constants.hpp"

using namespace std;

// route_plan, plan_cost, delivery_times
typedef pair< pair<vector<event>, double>, unordered_map<string, double>> best_plan_tuple;

void hungarian_assignment(vector<int> &active_vehicles, vector<order> &active_orders,
                          double global_time, vector<order> &rejected_orders){

    auto start = std::chrono::high_resolution_clock::now();
    // Cost Matrix and best route plan for all vehicle-order pairs
    vector<vector<double>> cost_mat(active_vehicles.size(), vector<double>(active_orders.size(), MAX_NUM));
    vector<vector<best_plan_tuple>> best_plans(active_vehicles.size(), vector<best_plan_tuple>(active_orders.size()));
    for(int i = 0; i < int(active_vehicles.size()); i++){
        vehicle vh = all_vehicles[active_vehicles[i]];
        if (int(vh.order_set.size()) >= batching_cap)
            continue;
        for(int j = 0; j < int(active_orders.size()); j++){
            order order_obj = active_orders[j];
            double vh_to_rest_dist = hhl_sp_query(vh.get_current_location(),
                                                  order_obj.restaurant.rest_node);
            if (vh_to_rest_dist >= vehicle_rest_radius_cap)
                continue;
            // route_plan, plan_cost, delivery_times
            best_plan_tuple plan_out = get_best_route_plan(vh, order_obj, global_time);
            cost_mat[i][j] = plan_out.first.second;
            best_plans[i][j] = plan_out;
        }
    }
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    if (VERBOSITY == -1)
        cout << "cost_time," << duration.count() << endl;
    start = std::chrono::high_resolution_clock::now();
    double min_cost = 0;
    for(auto it:cost_mat){
        for(auto it2:it)
            min_cost = min(min_cost, it2);
    }
    // Make cost matrix positive
    // Shortest delivery time is calculated based on graph of one timeslot
    // During simulation, timeslot and therefore graphs may change resulting in negative costs
    for(int i = 0; i < int(active_vehicles.size()); i++){
        for(int j = 0; j < int(active_orders.size()); j++){
            cost_mat[i][j] = (cost_mat[i][j] > max_cost_val)?max_cost_val:cost_mat[i][j];
            cost_mat[i][j] = cost_mat[i][j] + max(0.0, -min_cost);
        }
    }
    // order index assigned to each vehicle (-1 if not assigned an order)
    vector<int> assignment(cost_mat.size(), -1);
    if (int(cost_mat.size()) > 0 && int(cost_mat[0].size()) > 0){
        double cost = HUN_ASSIGN(cost_mat, assignment);
    }
    stop = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    if (VERBOSITY == -1)
        cout << "hungarian_time," << duration.count() << endl;
    for(int i = 0; i < int(assignment.size()); i++){
        if(assignment[i] != -1){
            if (cost_mat[i][assignment[i]] + FP_EPSILON >= (max_cost_val + max(0.0, -min_cost)))
                assignment[i] = -1;
        }
    }
    // Perform respective assignments
    unordered_map<int, int> vehicle_assigned;
    for(int i = 0; i < int(assignment.size()); i++){
        if (assignment[i] != -1){
            all_vehicles[active_vehicles[i]].assign_order(active_orders[assignment[i]],
                                                            best_plans[i][assignment[i]].first.first, global_time);
            if (VERBOSITY == -1)
                cout << fixed << "ASSIGN,"<< active_orders[assignment[i]].order_id << "," << all_vehicles[active_vehicles[i]].vehicle_id << ","<< global_time <<"," << cost_mat[i][assignment[i]] << endl;
            vehicle_assigned[assignment[i]] = i;
        }
    }
    // fill rejected orders in this round
    for (int i = 0; i < int(active_orders.size()); i++){
        if (vehicle_assigned.find(i)==vehicle_assigned.end())
            rejected_orders.push_back(active_orders[i]);
    }
}

void greedy_accumulate_assignment(vector<int> &active_vehicles, vector<order> &active_orders,
                                  double global_time, vector<order> &rejected_orders){
    // (iteration, order_idx, vh_idx, bpt)
    typedef struct o_v_assign
    {
        double cost;
        int iteration;
        int o_idx;
        int v_idx;
        best_plan_tuple bpt;
    }o_v_assign;
    struct compare_struct{
        bool operator()(o_v_assign const& p1, o_v_assign const& p2)
        {
            return p1.cost > p2.cost;
        }
    };
    double cost_time = 0.0;
    auto start = std::chrono::high_resolution_clock::now();
    priority_queue<o_v_assign, vector<o_v_assign>, compare_struct> pq;
    vector<int> o_assigned(active_orders.size(), 0);
    vector<int> latest_mod_iteration(active_vehicles.size(), 0);
    for(int j = 0; j < int(active_orders.size()); j++){
        order order_obj = active_orders[j];
        for(int i = 0; i < int(active_vehicles.size()); i++){
            vehicle vh = all_vehicles[active_vehicles[i]];
            if ((int(vh.order_set.size() + 1) > batching_cap))
                continue;
            double vh_to_rest_dist = hhl_sp_query(vh.get_current_location(),
                                                   order_obj.restaurant.rest_node);
            if ((vh_to_rest_dist >= vehicle_rest_radius_cap))
                continue;
            // route_plan, plan_cost, delivery_times
            best_plan_tuple plan_out = get_best_route_plan(vh, order_obj, global_time);
            if (plan_out.first.second < max_cost_val)
                pq.push({plan_out.first.second, 0, j, i, plan_out});
        }
    }
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    cost_time += duration.count();
    int current_iteration = 0;
    while(!pq.empty()){
        start = std::chrono::high_resolution_clock::now();
        o_v_assign top = pq.top();
        pq.pop();
        int top_iter = top.iteration;
        int o_idx = top.o_idx;
        int v_idx = top.v_idx;
        best_plan_tuple plan_out = top.bpt;
        if (plan_out.first.second > max_cost_val)
            break;
        if (o_assigned[o_idx] == 1 || top_iter != latest_mod_iteration[v_idx])
            continue;
        o_assigned[o_idx] = 1;
        current_iteration++;
        stop = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        cost_time += duration.count();
        all_vehicles[active_vehicles[v_idx]].assign_order(active_orders[o_idx],
                                                            plan_out.first.first, global_time);

        latest_mod_iteration[v_idx] = current_iteration;
        if (VERBOSITY == -1){
            cout << fixed << "ASSIGN,"<< active_orders[o_idx].order_id << "," << all_vehicles[active_vehicles[v_idx]].vehicle_id << ","<< global_time << endl;
        }
        start = std::chrono::high_resolution_clock::now();
        if (int(all_vehicles[active_vehicles[v_idx]].order_set.size() + 1) > batching_cap)
            continue;
        for(int i = 0; i < int(active_orders.size()); i++){
            if(o_assigned[i] == 0){
                vehicle vh = all_vehicles[active_vehicles[v_idx]];
                order order_obj = active_orders[i];
                double vh_to_rest_dist = hhl_sp_query(vh.get_current_location(),
                                                    order_obj.restaurant.rest_node);
                if ((vh_to_rest_dist >= vehicle_rest_radius_cap))
                    continue;
                // route_plan, plan_cost, delivery_times
                best_plan_tuple plan_out = get_best_route_plan(vh, order_obj, global_time);
                if (plan_out.first.second < max_cost_val)
                    pq.push({plan_out.first.second, current_iteration, i, v_idx, plan_out});
            }
        }
        stop = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        cost_time += duration.count();
    }
    if (VERBOSITY == -1)
        cout << "cost_time," << cost_time << endl;
    // fill rejected orders in this round
    for (int i = 0; i < int(active_orders.size()); i++){
        if (o_assigned[i] == 0)
            rejected_orders.push_back(active_orders[i]);
    }
}
