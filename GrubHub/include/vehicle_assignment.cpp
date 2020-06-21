#include <bits/stdc++.h>
#include "global.hpp"
#include "hungarian.hpp"
#include "routeplan.hpp"
#include "dsu.hpp"
#include "util.hpp"
#include "vehicle_assignment.hpp"
#include "constants.hpp"

using namespace std;

// route_plan, plan_cost, delivery_times
typedef pair< pair<vector<event>, double>, unordered_map<string, double>> best_plan_tuple;

// Get the bearing angle between two points
double get_direction(double x, double y, double x2, double y2){
    double d_y = y2-y;
    double d_x = x2-x;
    double brng = atan2(d_y,d_x);
    return brng;
}

// Return the heuristic function value
double heuristic_destination(vehicle &vh, double query_x, double query_y){
    double dest_x = vh.route_plan[1].x;
    double dest_y = vh.route_plan[1].y;
    double vh_x = vh.route_plan[0].x;
    double vh_y = vh.route_plan[0].y;
    // If current pickup/deliver is same as query node, it is fine
    if ((fabs(vh_x - query_x) <= FP_EPSILON) && (fabs(vh_y - query_y) <= FP_EPSILON)){
        return -1.0;
    }
    // Direction vehicle would be moving in after current pickup/deliver event
    double dir1 = get_direction(vh_x, vh_y, dest_x, dest_y);
    // Direction of query node after current pickup/deliver event
    double dir2 = get_direction(vh_x, vh_y, query_x, query_y);
    return -cos(dir1 - dir2);
}

// Normalized heuristic function value
// low is good
double heuristic_function(vehicle &vh, double query_x, double query_y){
    if(vh.route_plan.size() <= 1)
        return 0.0;
    return ((1.0+heuristic_destination(vh, query_x, query_y))/2.0);
}

// Given a route plan, assuming vehicle is at the first event node at global time
// find the delivered times of orders in the route plan
unordered_map<string, double> get_routeplan_delivered_times(vector<event> &route_plan, double global_time){
    unordered_map<string, double> delivered_time;
    double curr_x = route_plan[0].x;
    double curr_y = route_plan[0].y;
    string prev_id = "0";
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

            if (route_plan[i].loc_id != route_plan[i+1].loc_id){
                global_time += pickup_service/2.0;
            }
        }
        // Deliver
        else{
            if (route_plan[i].loc_id != prev_id){
                global_time += dropoff_service/2.0;
            }
            delivered_time[event_order.order_id] = global_time;

            global_time += dropoff_service/2.0;
        }
        curr_x = route_plan[i].x;
        curr_y = route_plan[i].y;
    }
    return delivered_time;
}

// Return the extra delivery time of the routeplan
best_plan_tuple get_routeplan_cost(vector<event> &rp, double global_time){
    unordered_map<string, double> d_times = get_routeplan_delivered_times(rp, global_time);
    return {{rp, get_route_plan_extra_delivery_time(d_times, rp)}, d_times};
}

// Generate the order graph
vector<vector<best_plan_tuple>> generate_start_order_undirected_graph(vector<order> order_set, double global_time){
    int n = order_set.size();
    vector<vector<best_plan_tuple>> order_graph(n, vector<best_plan_tuple>(n, {{{}, MAX_NUM}, {}}));
    for(int i = 0; i < n; i++){
        order order_i = order_set[i];
        event event_i_p(order_i, 0);
        event event_i_d(order_i, 1);
        vector<event> temp_e({event_i_p, event_i_d});
        best_plan_tuple temp = get_routeplan_cost(temp_e, global_time);
        order_graph[i][i] = temp;
    }
    for (int i = 0; i < n; i++){
        order order_i = order_set[i];
        event event_i_p(order_i, 0);
        event event_i_d(order_i, 1);
        for(int j = i+1; j < n; j++){
            order order_j = order_set[j];
            event event_j_p(order_j, 0);
            event event_j_d(order_j, 1);
            vector<event> rp_1{event_i_p, event_i_d, event_j_p, event_j_d};
            vector<event> rp_2{event_i_p, event_j_p, event_i_d, event_j_d};
            vector<event> rp_3{event_i_p, event_j_p, event_j_d, event_i_d};
            vector<event> rp_4{event_j_p, event_j_d, event_i_p, event_i_d};
            vector<event> rp_5{event_j_p, event_i_p, event_j_d, event_i_d};
            vector<event> rp_6{event_j_p, event_i_p, event_i_d, event_j_d};
            vector<vector<event>> all_rp{rp_1, rp_2, rp_3, rp_4, rp_5, rp_6};
            best_plan_tuple bp{{{}, MAX_NUM}, {}};
            for(auto rp:all_rp){
                best_plan_tuple cp = get_routeplan_cost(rp, global_time);
                if (cp.first.second < bp.first.second){
                    bp = cp;
                }
            }
            if (bp.first.second + FP_EPSILON >= MAX_NUM)
                continue;
            order_graph[i][j] = bp;
            order_graph[i][j].first.second = order_graph[i][j].first.second - order_graph[i][i].first.second - order_graph[j][j].first.second;
        }
    }
    return order_graph;
}

// Given two routeplans, returns a interleaving routeplan with minimal cost
// while keeping the sequence of input routeplans fixed
best_plan_tuple get_inserted_bestplan(vector<event> &rp0, vector<event> &rp1, double global_time){
    int n0 = rp0.size();
    int n1 = rp1.size();
    vector<int> perm(n0 + n1, 0);
    for(int i = n0; i < n0+n1; i++){
        perm[i] = 1;
    }
    best_plan_tuple bp{{{}, MAX_NUM}, {}};
    vector<event> new_rp(n0 + n1);
    do {
        int cnt0=0;
        int cnt1=0;
        for(auto it:perm){
            if (it == 0){
                new_rp[cnt1+cnt0] = rp0[cnt0];
                cnt0++;
            }
            else{
                new_rp[cnt1+cnt0] = rp1[cnt1];
                cnt1++;
            }
        }
        best_plan_tuple cp = get_routeplan_cost(new_rp, global_time);
        if (cp.first.second < bp.first.second){
            bp = cp;
        }
    } while (next_permutation(perm.begin(),perm.end()));
    return bp;
}

// Perform clustering on order graph
vector<best_plan_tuple> hac_cluster_orders(vector<order> &order_set, double global_time){
    // merge_parameter
    double max_merge_cost = max_merge_cost_edt;
    int n = order_set.size();
    if (n == 0)
        return {};
    DSU dsu;
    dsu.init(n);
    vector<vector<best_plan_tuple>> order_graph = generate_start_order_undirected_graph(order_set, global_time);
    // Reference - https://nlp.stanford.edu/IR-book/pdf/17hier.pdf
    double double_to_int = 100000000.0;
    double total_sum = 0.0;
    double total_batches = n;
    map<pair<long long int, int>, pair<int, int>> PQ[n];
    for(int i = 0; i < n; i++){
        for(int j = i+1; j < n; j++){
            double cost = order_graph[i][j].first.second;
            if (cost + FP_EPSILON >= MAX_NUM)
                continue;
            PQ[i][{(long long int)(cost*double_to_int), j}] = {i,j};
        }
        if (order_graph[i][i].first.second + FP_EPSILON <= MAX_NUM)
            total_sum += order_graph[i][i].first.second;
    }
    vector<int> I(n, 1);
    int iter_c = 0;
    while(true){
        if (total_sum/total_batches > max_merge_cost)
            break;
        long long int curr_min = (long long int)((max_merge_cost+1)*double_to_int);
        int i_min = -1, j_min = -1;
        for(int i = 0; i < n; i++){
            if ((I[i] == 0) || PQ[i].empty())
                continue;
            pair<long long int, int> min_elem = PQ[i].begin()->first;
            if (min_elem.first < curr_min){
                curr_min = min_elem.first;
                i_min = i;
                j_min = min_elem.second;
            }
        }
        if (i_min == -1 || j_min == -1)
            break;
        dsu.merge(i_min, j_min);
        int set_i = dsu.root(i_min);
        double new_cost = order_graph[i_min][j_min].first.second + order_graph[i_min][i_min].first.second + order_graph[j_min][j_min].first.second;
        order_graph[set_i][set_i] = order_graph[i_min][j_min];
        order_graph[set_i][set_i].first.second = new_cost;
        total_sum += order_graph[i_min][j_min].first.second;
        total_batches--;
        I[j_min] = 0;
        PQ[i_min].clear();
        for(int i = 0; i < j_min; i++){
            if (I[i] == 1){
                double cost = order_graph[i][j_min].first.second;
                PQ[i].erase({(long long int)(cost*double_to_int), j_min});
            }
        }
        for(int i = 0; i < i_min; i++){
            if (I[i] == 1){
                double cost = order_graph[i][i_min].first.second;
                PQ[i].erase({(long long int)(cost*double_to_int), i_min});
            }
        }
        int size_set_i = dsu.set_to_elems[set_i].size();
        for(int i = 0; i < i_min; i++){
            if (I[i] == 1){
                int size_i = dsu.get_elems(i).size();
                if (size_set_i + size_i > max_o_count ||
                    (order_graph[i][i_min].first.second + FP_EPSILON >= MAX_NUM) ||
                    (order_graph[i][j_min].first.second + FP_EPSILON >= MAX_NUM)){
                        continue;
                }
                order_graph[i][set_i] = get_inserted_bestplan(order_graph[set_i][set_i].first.first,
                                                                   order_graph[i][i].first.first, global_time);
                order_graph[i][set_i].first.second -= (order_graph[set_i][set_i].first.second + order_graph[i][i].first.second);
                double cost = order_graph[i][set_i].first.second;
                if (cost + FP_EPSILON >= MAX_NUM)
                    continue;
                PQ[i][{(long long int)(cost*double_to_int), set_i}] = {i,set_i};
            }
        }
        for(int i = set_i+1; i < n; i++){
            if (I[i] == 1){
                int size_i = dsu.get_elems(i).size();
                if (size_set_i + size_i > max_o_count ||
                    (order_graph[i_min][i].first.second + FP_EPSILON >= MAX_NUM) ||
                    (order_graph[i][j_min].first.second + FP_EPSILON >= MAX_NUM && order_graph[j_min][i].first.second + FP_EPSILON >= MAX_NUM)){
                        continue;
                }
                order_graph[set_i][i] = get_inserted_bestplan(order_graph[set_i][set_i].first.first,
                                                                   order_graph[i][i].first.first, global_time);
                order_graph[set_i][i].first.second -= (order_graph[set_i][set_i].first.second + order_graph[i][i].first.second);
                double cost = order_graph[set_i][i].first.second;
                if (cost + FP_EPSILON > MAX_NUM)
                    continue;
                PQ[set_i][{(long long int)(cost*double_to_int), i}] = {set_i,i};
            }
        }
        iter_c++;
    }
    vector<best_plan_tuple> cluster_pack;
    for(auto it:dsu.set_to_elems){
        cluster_pack.push_back(order_graph[it.first][it.first]);
    }
    return cluster_pack;
}

// Given two routeplans, returns a list of interleaving routeplans
// while keeping the sequence of input routeplans fixed
vector<vector<event>> gen_inserted_sequence_rp(vector<event> &rp0, vector<event> &rp1){
    vector<vector<event>> ans;
    int n0 = rp0.size();
    int n1 = rp1.size();
    vector<int> perm(n0 + n1, 0);
    for(int i = n0; i < n0+n1; i++){
        perm[i] = 1;
    }
    do {
        vector<event> new_rp(n0 + n1);
        int cnt0=0;
        int cnt1=0;
        for(auto it:perm){
            if (it == 0){
                new_rp[cnt1+cnt0] = rp0[cnt0];
                cnt0++;
            }
            else{
                new_rp[cnt1+cnt0] = rp1[cnt1];
                cnt1++;
            }
        }
        ans.push_back(new_rp);
    } while (next_permutation(perm.begin(),perm.end()));
    return ans;
}

void foodmatch(vector<int> &active_vehicles, vector<order> &active_orders,
               double global_time, vector<order> &rejected_orders){
    // Reshuffling
    if ((global_time < 86400)){
        for(auto idx:active_vehicles){
            vehicle* vh = &all_vehicles[idx];
            if ((vh->route_plan).empty())
                continue;
            vector<event> new_route_plan;
            int new_order_count = 0;
            unordered_set<string> picked_idxs;
            // First sequence of pickups won't be reshuffled
            // Constraint of MDRP
            int start_idx = 0;
            while((vh->route_plan[start_idx]).type == 0){
                new_route_plan.push_back(vh->route_plan[start_idx]);
                start_idx++;
            }
            for(int i = start_idx; i < vh->route_plan.size(); i++){
                event evt = vh->route_plan[i];
                if (evt.type == 0){
                    active_orders.push_back(evt.order_obj);
                    picked_idxs.insert(evt.order_obj.order_id);
                }
                else{
                    if (picked_idxs.find(evt.order_obj.order_id) == picked_idxs.end()){
                        new_route_plan.push_back(evt);
                        new_order_count++;
                    }
                }
            }
            (vh->order_count) = new_order_count;
            (vh->route_plan) = new_route_plan;
        }
    }
    vector<best_plan_tuple> bp_packs = hac_cluster_orders(active_orders, global_time);
    vector<int> order_packs(bp_packs.size(), 0);
    for(int i = 0; i < int(bp_packs.size()); i++){
        for (auto ev:bp_packs[i].first.first){
            if (ev.type == 0)
                order_packs[i]++;
        }
    }
    vector<vector<double>> cost_mat(active_vehicles.size(), vector<double>(order_packs.size(), MAX_NUM));
    vector<vector<best_plan_tuple>> best_plans(active_vehicles.size(), vector<best_plan_tuple>(order_packs.size()));
    double min_cost_val = MAX_NUM;
    for(int i = 0; i < int(active_vehicles.size()); i++){
        vehicle vh = all_vehicles[active_vehicles[i]];
        vector<event> org_route_plan = vh.route_plan;
        unordered_map<string, double> org_d_times = get_delivered_times(vh, org_route_plan, global_time);
        double org_cost = get_route_plan_extra_delivery_time(org_d_times, org_route_plan);
        vector<pair<double, int>> order_heur(order_packs.size());
        for(int j = 0; j < int(order_packs.size()); j++){
            vector<event> bp = bp_packs[j].first.first;
            double h_val = 0.0;
            if (vh.route_plan.size() > 1){
                h_val = heuristic_multiplier*heuristic_function(vh, bp[0].x, bp[0].y) +
                        (1-heuristic_multiplier)*get_distance(vh.route_plan[0].x, vh.route_plan[0].y, bp[0].x, bp[0].y)/max_travel_dist;
            }
            order_heur[j] = {h_val, j};
        }
        // There is no road network to explore
        // Find a modified distance function and choose top-k of those edges
        sort(order_heur.begin(), order_heur.end());
        int explore_edges = int(ceil(double(order_packs.size())/double(active_vehicles.size()) * vehicle_explore_frac) + FP_EPSILON);
        explore_edges = min(explore_edges, int(order_packs.size()));
        for(int k = 0; k < explore_edges; k++){
            int j = order_heur[k].second;
            double min_cost = MAX_NUM;
            vector<event> min_plan;
            unordered_map<string, double> min_delivery_times;
            vector<event> bp = bp_packs[j].first.first;
            int op_size = order_packs[j];
            if (int(vh.order_count + op_size) > max_o_count)
                continue;
            vector<vector<event>> all_route_plans = gen_inserted_sequence_rp(org_route_plan, bp);
            for(int ri = 0; ri < int(all_route_plans.size()); ri++){
                // First point can't be rerouted
                // Constraint of MDRP
                if ((org_route_plan.size() > 0) && (all_route_plans[ri][0].loc_id != org_route_plan[0].loc_id)){
                    continue;
                }
                unordered_map<string, double> d_times = get_delivered_times(vh, all_route_plans[ri], global_time);
                if (d_times.empty()){
                    continue;
                }
                double plan_cost = get_route_plan_extra_delivery_time(d_times, all_route_plans[ri]) - org_cost;
                if (plan_cost < min_cost){
                    min_cost = plan_cost;
                    min_plan = all_route_plans[ri];
                    min_delivery_times = d_times;
                }
            }
            cost_mat[i][j] = min_cost;
            min_cost_val = min(min_cost_val, min_cost);
            best_plans[i][j] = {{min_plan, min_cost}, min_delivery_times};
        }
    }
    for(int i = 0; i < int(active_vehicles.size()); i++){
        for(int j = 0; j < int(bp_packs.size()); j++){
            cost_mat[i][j] += (min_cost_val > 0)?0:-min_cost_val;
            cost_mat[i][j] = min(cost_mat[i][j], MAX_NUM);
        }
    }
    for(int i = 0; i < int(active_vehicles.size()); i++){
        for(int j = 0; j < int(bp_packs.size()); j++){
            cost_mat[i][j] = (cost_mat[i][j] > max_ctd)?max_ctd:cost_mat[i][j];
        }
    }
    // order index assigned to each vehicle (-1 if not assigned an order)
    vector<int> assignment(cost_mat.size(), -1);
    if (int(cost_mat.size()) > 0 && int(cost_mat[0].size()) > 0){
        double cost = HUN_ASSIGN(cost_mat, assignment);
        cout << "HUN_COST = " << cost << endl;
    }
    for(int i = 0; i < int(assignment.size()); i++){
        if(assignment[i] != -1){
            if (cost_mat[i][assignment[i]] + FP_EPSILON >= max_ctd)
                assignment[i] = -1;
        }
    }
    // Perform respective assignments
    unordered_map<int, int> vehicle_assigned;
    for(int i = 0; i < int(assignment.size()); i++){
        if (assignment[i] != -1){
            all_vehicles[active_vehicles[i]].assign_order_pack(order_packs[assignment[i]],
                                                               best_plans[i][assignment[i]].first.first);
            if (VERBOSITY == -1){
                for (auto it:bp_packs[assignment[i]].first.first){
                    if (it.type == 0){
                        cout << fixed << "ASSIGN,"<< it.order_obj.order_id << "," << all_vehicles[active_vehicles[i]].vehicle_id;
                        cout << "," << global_time <<"," << cost_mat[i][assignment[i]] << endl;
                    }
                }
            }
            vehicle_assigned[assignment[i]] = i;
        }
    }
    // fill rejected orders in this round
    for (int i = 0; i < int(order_packs.size()); i++){
        if (vehicle_assigned.find(i)==vehicle_assigned.end()){
            for (auto it:bp_packs[i].first.first){
                if (it.type == 0)
                    rejected_orders.push_back(it.order_obj);
            }
        }
    }
}
