#include <bits/stdc++.h>
#include "global.hpp"
#include "hungarian.hpp"
#include "routeplan.hpp"
#include "hhl_query.hpp"
#include "mdrp_baseline.hpp"
#include "constants.hpp"

using namespace std;

double target_ctd = 40*60;
// Mean value of parameter variations : exact not mentioned
double service_loss_tol = (200.0/10.0);
double freshness_loss_tol = (100.0/10.0);
double freshness_commit = 20.0 * 60.0;
double util_coeff = 5.5;
double freshness_loss_coeff = 0.5;
double service_loss_coeff = 0.5;
// 1 : not mentioned anywhere
int Z_t_default = 1;

// Check route plan capacity constraint
bool rp_cap_constraint(vector<event> &route_plan){
    long long int curr_load = 0;
    for (int i = 0; i < int(route_plan.size()); i++){
        // If Pickup, increase item count
        if (route_plan[i].type == 0){
            curr_load += route_plan[i].order_obj.items;
            if (curr_load > global_conf.vehicle_max_items)
                return false;
        }
        // If Delivery, decrease item count
        else
            curr_load -= route_plan[i].order_obj.items;
    }
    return true;
}

// <time_per_order, route_cost> for Bundling procedure
pair<double, double> get_route_cost(vector<event> &route_plan, double curr_time){
    if (route_plan.empty() || !rp_cap_constraint(route_plan))
        return {MAX_NUM, 0.0};
    double travel_time = 0.0, service_delay = 0.0;
    double max_pick_time = -1;
    double order_count = 0.0;
    long long int curr_node = route_plan[0].node;
    double global_time = curr_time;
    double time_per_order = 0.0;
    for (int i = 0; i < int(route_plan.size()); i++){
        order event_order = route_plan[i].order_obj;
        double time_taken = hhl_sp_query(curr_node, route_plan[i].node);
        travel_time += time_taken;
        global_time += time_taken;
        // Pick Up
        if (route_plan[i].type == 0){
            double food_prep_time = event_order.order_time + event_order.prep_time;
            global_time = max(global_time, food_prep_time);
            max_pick_time = max(max_pick_time, global_time);
        }
        else{
            order_count += 1.0;
            time_per_order += global_time - event_order.order_time;
        }
        curr_node = route_plan[i].node;
    }
    for(auto it:route_plan){
        if (it.type==0)
            service_delay += max_pick_time - (it.order_obj.order_time + it.order_obj.prep_time);
    }
    return {time_per_order/order_count,
            travel_time + service_loss_coeff*service_delay};
}

// Given a route plan, generate routeplans with the new order inserted into it
void generate_inserted_rp(vector<event> &org_rp, vector<vector<event>> &answer, order &new_order){
    event o_pickup(new_order, 0);
    event o_deliver(new_order, 1);
    if(org_rp.empty()){
        answer.push_back({o_pickup, o_deliver});
        return;
    }
    org_rp.insert(org_rp.begin(), o_pickup);
    for (int eid = 0; eid < int(org_rp.size()); eid++){
        if (org_rp[eid].type == 0){
            continue;
        }
        vector<event> cand_rp(org_rp);
        cand_rp.insert(cand_rp.begin()+eid, o_deliver);
        answer.push_back(cand_rp);
    }
    vector<event> cand_rp(org_rp);
    cand_rp.push_back(o_deliver);
    answer.push_back(cand_rp);
}

// order insertion in Bundling Procedure
double best_order_insertion(vector<event> &org_plan, order &o, int zt, vector<event> &best_rp,
                            int vh_idx, double global_time){
    pair<double, double> org_cost = get_route_cost(org_plan, global_time);
    int order_count = 0;
    for(auto it:org_plan){
        if (it.type == 1)
            order_count++;
    }
    double org_tpo = org_cost.first;
    vector<event> rp_copy(org_plan);
    vector<vector<event>> all_rp;
    generate_inserted_rp(rp_copy, all_rp, o);
    double best_cost = MAX_NUM;
    double best_eff = MAX_NUM;
    for (auto rp:all_rp){
        pair<double, double> rp_cost = get_route_cost(rp, global_time);
        if (rp_cost.first + FP_EPSILON >= MAX_NUM)
            continue;
        double r_cost = rp_cost.second - org_cost.second;
        if (r_cost < best_cost){
            best_cost = r_cost;
            best_rp = rp;
            best_eff = rp_cost.first;
        }
    }
    double new_tpo = best_eff;
    if ((order_count+1 <= zt) || (new_tpo < org_tpo)){
        return best_cost;
    }
    return MAX_NUM + FP_EPSILON;
}

// Linear formulation cost values for "Default" configuration
pair<pair<double, double>, double> get_profit_vals(vehicle &vh, vector<event> &route_plan, double curr_time){
    double max_picked = -1;
    double max_drop = -1;
    double max_ready_time = -1;
    double e_d_time = -1;
    double global_time;
    long long int curr_node;
    if (vh.route_plan.size() == 0){
        curr_node = vh.get_current_location();
        global_time = curr_time;
        e_d_time = global_time;
    }
    else{
        curr_node = vh.route_plan.back().node;
        global_time = vh.e_d;
        e_d_time = vh.e_d;
    }
    for (int i = 0; i < int(route_plan.size()); i++){
        order event_order = route_plan[i].order_obj;
        double time_taken = hhl_sp_query(curr_node, route_plan[i].node);
        global_time += time_taken;
        // Pick Up
        if (route_plan[i].type == 0){
            double food_prep_time = event_order.order_time + event_order.prep_time;
            global_time = max(global_time, food_prep_time);
            max_picked = max(max_picked, global_time);
            max_ready_time = max(max_ready_time, food_prep_time);
        }
        else{
            max_drop = max(max_drop, global_time);
        }
        curr_node = route_plan[i].node;
    }
    // assuming minutes, since all input is in minutes
    return {{(max_drop - e_d_time)/60.0, (max_picked - max_ready_time)/60.0}, global_time};
}

// Three priority groups
pair<bool, bool> get_groups(vector<event> &route_plan, double curr_time){
    double drop_ctd = true;
    bool pick_by_ready = true;
    long long int curr_node = route_plan[0].node;
    double global_time = curr_time;
    for (int i = 0; i < int(route_plan.size()); i++){
        order event_order = route_plan[i].order_obj;
        double time_taken = hhl_sp_query(curr_node, route_plan[i].node);
        global_time += time_taken;
        // Pick Up
        if (route_plan[i].type == 0){
            double food_prep_time = event_order.order_time + event_order.prep_time;
            global_time = max(global_time, food_prep_time);
            if (global_time > food_prep_time)
                pick_by_ready = false;
        }
        else{
            if (global_time - event_order.order_time > target_ctd)
                drop_ctd = false;
        }
        curr_node = route_plan[i].node;
    }
    return {drop_ctd, pick_by_ready};
}

// Linear assignment
void perform_assignment(vector<int> &available_vehicles, vector<vector<event>> &all_bundles,
                        vector<int> &index_to_assign, double global_time, vector<order> &rejected_orders){

    // Cost Matrix and best route plan for all vehicle-order pairs
    vector<vector<double>> cost_mat(available_vehicles.size() + index_to_assign.size(), vector<double>(index_to_assign.size(), MAX_NUM));
    vector<vector<double>> new_ed_mat(available_vehicles.size() + index_to_assign.size(), vector<double>(index_to_assign.size(), MAX_NUM));
    double min_cost_val = MAX_NUM;
    for(int i = 0; i < int(available_vehicles.size()); i++){
        vehicle vh = all_vehicles[available_vehicles[i]];
        for(int j = 0; j < int(index_to_assign.size()); j++){
            vector<event> bundle = all_bundles[index_to_assign[j]];
            double N_s = 0.0;
            for(auto it:bundle){
                if (it.type == 1)
                    N_s++;
            }
            pair<pair<double, double>, double> profit_vals = get_profit_vals(vh, bundle, global_time);
            double denom = profit_vals.first.first;
            double num = profit_vals.first.second;
            double new_ed = profit_vals.second;
            double profit = N_s/denom - freshness_loss_coeff*num;
            cost_mat[i][j] = -profit;
            new_ed_mat[i][j] = new_ed;
            min_cost_val = min(min_cost_val, cost_mat[i][j]);
        }
    }
    cout << "HUN " << min_cost_val << endl;
    for(int i = 0; i < int(available_vehicles.size()); i++){
        for(int j = 0; j < int(index_to_assign.size()); j++){
            cost_mat[i][j] += (min_cost_val > 0)?0:-min_cost_val;
            cost_mat[i][j] = min(cost_mat[i][j], MAX_NUM);
        }
    }
    // order index assigned to each vehicle (-1 if not assigned an order)
    vector<int> assignment(cost_mat.size(), -1);
    if (int(cost_mat.size()) > 0 && int(cost_mat[0].size()) > 0){
        double cost = HUN_ASSIGN(cost_mat, assignment);
    }
    for(int i = 0; i < int(available_vehicles.size()); i++){
        if(assignment[i] != -1){
            if (cost_mat[i][assignment[i]] + FP_EPSILON >= MAX_NUM){
                assignment[i] = -1;
            }
        }
        if(assignment[i] != -1){
            double max_ready = -1;
            for(auto it:all_bundles[index_to_assign[assignment[i]]]){
                max_ready = max(max_ready, it.order_obj.order_time + it.order_obj.prep_time);
            }
            // 2 stage commitment or not possible
            if ((all_vehicles[available_vehicles[i]].e_d > global_time + delta_time) && (global_time - max_ready <= freshness_commit)){
                assignment[i] = -1;
            }
        }
    }
    vector<int> remaining_vehicles;
    // Perform respective assignments
    unordered_map<int, int> vehicle_assigned;
    for(int i = 0; i < int(available_vehicles.size()); i++){
        if (assignment[i] != -1){
            vector<event> rp_assign(all_vehicles[available_vehicles[i]].route_plan);
            vector<order> new_pack;
            for(auto it:all_bundles[index_to_assign[assignment[i]]]){
                rp_assign.push_back(it);
                if (it.type == 1){
                    new_pack.push_back(it.order_obj);
                }
            }
            all_vehicles[available_vehicles[i]].assign_order_pack(new_pack,
                                                                  rp_assign, global_time);

            if (VERBOSITY == -1){
                for (auto it:all_bundles[index_to_assign[assignment[i]]]){
                    if (it.type == 0){
                        cout << fixed << "ASSIGN,"<< it.order_obj.order_id << "," << all_vehicles[available_vehicles[i]].vehicle_id;
                        cout << "," << global_time <<"," << cost_mat[i][assignment[i]] << endl;
                    }
                }
            }
            all_vehicles[available_vehicles[i]].set_e_d(new_ed_mat[i][assignment[i]]);
            vehicle_assigned[assignment[i]] = i;
        }
        else{
            remaining_vehicles.push_back(available_vehicles[i]);
        }
    }
    available_vehicles = remaining_vehicles;
    // fill rejected orders in this round
    for (int i = 0; i < int(index_to_assign.size()); i++){
        if (vehicle_assigned.find(i)==vehicle_assigned.end()){
            for (auto it:all_bundles[index_to_assign[i]]){
                if (it.type == 0)
                    rejected_orders.push_back(it.order_obj);
            }
        }
    }

}


void mdrp_assignment(vector<int> &active_vehicles, vector<order> &active_orders,
                          double global_time, vector<order> &rejected_orders){
    //mdrp parameters - 2f as mentioned by Reyes et. al.
    double delta_u = 2*delta_time;
    double delta_1 = 2*delta_time;
    double delta_2 = 2*delta_time;
    // double delta_u = 360;
    // double delta_1 = 360;
    // double delta_2 = 360;
    auto start = std::chrono::high_resolution_clock::now();
    // System intensity and a target bundle size
    vector<order> U_t;
    double Z_t_num = 0.0, Z_t_denom = 0.0;
    double max_prep_time = 0.0;
    for(auto it:active_orders){
        if ((it.order_time + it.prep_time) <= (global_time + delta_u)){
            U_t.push_back(it);
        }
        else{
            rejected_orders.push_back(it);
        }
        if ((it.order_time + it.prep_time) <= (global_time + delta_1)){
            Z_t_num++;
        }
        max_prep_time = max(max_prep_time, it.prep_time);
    }
    rejection_window = max(rejection_window, max_prep_time + 15*60);
    unordered_map<string, vector<int>> rest_to_vehicle_index;
    unordered_map<string, vector<pair<double, int>>> rest_to_order_index;
    for (int i = 0; i < int(U_t.size()); i++){
        rest_to_order_index[U_t[i].restaurant.rest_id].push_back({(U_t[i].order_time + U_t[i].prep_time), i});
        rest_to_vehicle_index[U_t[i].restaurant.rest_id] = {};
    }
    for (int i = 0; i < int(active_vehicles.size()); i++){
        vehicle vh = all_vehicles[active_vehicles[i]];
        if (vh.e_d <= (global_time + delta_2)){
            Z_t_denom++;
        }
        // two stage commitment - partially committed vehicles
        if ((vh.route_plan.size() > 0) && (vh.route_plan[0].type == 0)){
            rest_to_vehicle_index[vh.route_plan[0].order_obj.restaurant.rest_id].push_back(active_vehicles[i]);
        }
    }
    int Z_t = Z_t_default;
    if (Z_t_denom > 0)
        Z_t = int(ceil(Z_t_num/Z_t_denom) + FP_EPSILON);
    // Creation of bundles and delivery routes
    vector<vector<event>> bundles_to_assign;
    for (auto ro_it:rest_to_order_index){
        string r_id = ro_it.first;
        vector<pair<double, int>> U_t_r = ro_it.second;
        // Initial construction
        sort(U_t_r.begin(), U_t_r.end());
        vector<int> k_t_r = rest_to_vehicle_index[r_id];
        int m_r = int(max(double(k_t_r.size()),
                          ceil( double(U_t_r.size())/double(Z_t) ) )
                      + FP_EPSILON);
        vector<int> bundle_index((U_t_r.size()), -1);
        // empty bundles
        vector<vector<event>> rest_bundles(m_r);
        // initial are partially committed vehicles
        for(int kj = 0; kj < int(k_t_r.size()); kj++){
            rest_bundles[kj] = all_vehicles[k_t_r[kj]].route_plan;
        }
        for(int u_i = 0; u_i < int(U_t_r.size()); u_i++){
            order o = U_t[U_t_r[u_i].second];
            double best_cost = MAX_NUM;
            vector<event> best_rp = {};
            int best_bundle = -1;
            for(int m = 0; m < m_r; m++){
                vector<event> cand_rp;
                int vh_idx = -1;
                if(m < int(k_t_r.size()))
                    vh_idx = k_t_r[m];
                double cand_cost = best_order_insertion(rest_bundles[m], o, Z_t, cand_rp, vh_idx, global_time);
                if (cand_cost < best_cost){
                    best_cost = cand_cost;
                    best_rp = cand_rp;
                    best_bundle = m;
                }
            }
            if (best_bundle != -1){
                rest_bundles[best_bundle] = best_rp;
                bundle_index[u_i] = best_bundle;
            }
        }
        // Improvement by ‘‘remove-reinsert’’ local-search
        for(int u_i = 0; u_i < int(U_t_r.size()); u_i++){
            if (bundle_index[u_i] < 0){
                event o_pickup(U_t[U_t_r[u_i].second], 0);
                event o_deliver(U_t[U_t_r[u_i].second], 1);
                bundles_to_assign.push_back({o_pickup, o_deliver});
                continue;
            }
            int b_id = bundle_index[u_i];
            order o = U_t[U_t_r[u_i].second];
            vector<event> new_rp;
            vector<event> old_rp(rest_bundles[b_id]);
            for(auto it:rest_bundles[b_id]){
                if (it.order_obj.order_id != o.order_id){
                    new_rp.push_back(it);
                }
            }
            rest_bundles[b_id] = new_rp;
            double best_cost = MAX_NUM;
            vector<event> best_rp = {};
            int best_bundle = -1;
            for(int m = 0; m < m_r; m++){
                vector<event> cand_rp;
                int vh_idx = -1;
                if(m < int(k_t_r.size()))
                    vh_idx = k_t_r[m];
                double cand_cost = best_order_insertion(rest_bundles[m], o, Z_t, cand_rp, vh_idx, global_time);
                if (cand_cost < best_cost){
                    best_cost = cand_cost;
                    best_rp = cand_rp;
                    best_bundle = m;
                }
            }
            if (best_bundle != -1){
                rest_bundles[best_bundle] = best_rp;
                bundle_index[u_i] = best_bundle;
            }
            else{
                rest_bundles[b_id] = old_rp;
                bundle_index[u_i] = b_id;
            }
        }
        vector<vector<order>> new_packs(k_t_r.size());
        for(int u_i = 0; u_i < int(U_t_r.size()); u_i++){
            if ((bundle_index[u_i] >= 0) && (bundle_index[u_i] < int(k_t_r.size()))){
                order o = U_t[U_t_r[u_i].second];
                cout << fixed << "ASSIGN,"<< o.order_id << "," << all_vehicles[k_t_r[bundle_index[u_i]]].vehicle_id;
                cout << "," << global_time <<"," << "-1" << endl;
                new_packs[bundle_index[u_i]].push_back(U_t[U_t_r[u_i].second]);
            }
        }
        for(int i = 0; i < int(k_t_r.size()); i++){
            all_vehicles[k_t_r[i]].assign_order_pack(new_packs[i], rest_bundles[i], global_time);
            unordered_map<string, double> d_times = get_delivered_times(all_vehicles[k_t_r[i]], rest_bundles[i], global_time);
            double max_dtime = -1;
            for (auto temp_it:d_times){
                max_dtime = max(max_dtime, temp_it.second);
            }
            all_vehicles[k_t_r[i]].set_e_d(max_dtime);
        }
        for(int i = int(k_t_r.size()); i < m_r; i++){
            if (rest_bundles[i].size() > 0)
                bundles_to_assign.push_back(rest_bundles[i]);
        }
    }
    vector<int> group_1, group_2, group_3;
    for(int i = 0; i < bundles_to_assign.size(); i++){
        pair<bool, bool> condition = get_groups(bundles_to_assign[i], global_time);
        if (!condition.first){
            group_1.push_back(i);
        }
        else if(!condition.second){
            group_2.push_back(i);
        }
        else{
            group_3.push_back(i);
        }
    }
    vector<int> remaining_vehicles(active_vehicles);
    perform_assignment(remaining_vehicles, bundles_to_assign, group_1,
                       global_time, rejected_orders);
    perform_assignment(remaining_vehicles, bundles_to_assign, group_2,
                       global_time, rejected_orders);
    perform_assignment(remaining_vehicles, bundles_to_assign, group_3,
                       global_time, rejected_orders);
}
