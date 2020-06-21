#include <bits/stdc++.h>
#include "global.hpp"
#include "config.hpp"
#include "order.hpp"
#include "food_data_util.hpp"
#include "vehicle.hpp"
#include <glob.h>
#include "graph_util.hpp"
#include "hhl_query.hpp"
#include "constants.hpp"

using namespace std;

mt19937 rng2(global_conf.random_seed);

// Returns preparation time of a restaurant sampled from a truncated normal distribution
// id - restaurant_id
// timeslot - 0-23 hour of day at which prep time is required
double get_prep_time(string id, int timeslot){
    double mean_p_time, std_p_time;
    if (slotted_rest_prep_time[timeslot].find(id) == slotted_rest_prep_time[timeslot].end())
        mean_p_time = global_conf.avg_rest_p_time;
    else
        mean_p_time = slotted_rest_prep_time[timeslot][id];

    if (slotted_rest_prep_time_std[timeslot].find(id) == slotted_rest_prep_time_std[timeslot].end())
        std_p_time = global_conf.std_rest_p_time;
    else
        std_p_time = slotted_rest_prep_time_std[timeslot][id];
    double min_time = global_conf.min_rest_p_time;
    double max_time = global_conf.max_rest_p_time;
    normal_distribution<double> nd(mean_p_time, std_p_time);
    double sample_p_time;
    int cnt = 0;
    do{
        if (cnt >= 5){
            sample_p_time = mean_p_time;
            break;
        }
        sample_p_time = nd(rng2);
        cnt++;
    }while((sample_p_time < min_time) || (sample_p_time > max_time));
    return sample_p_time;
}

// Lists a directory (os.listdir in python)
// Reference - https://stackoverflow.com/questions/8401777/simple-glob-in-c-on-unix-system
vector<string> glob_listdir(const string& pattern) {
    // glob struct resides on the stack
    glob_t glob_result;
    memset(&glob_result, 0, sizeof(glob_result));
    // do the glob operation
    int return_value = glob(pattern.c_str(), GLOB_TILDE, NULL, &glob_result);
    if(return_value != 0) {
        globfree(&glob_result);
        stringstream ss;
        ss << "glob() failed with return_value " << return_value << "\n";
        throw std::runtime_error(ss.str());
    }
    // collect all the filenames into a std::list<std::string>
    vector<string> filenames;
    for(size_t i = 0; i < glob_result.gl_pathc; ++i) {
        filenames.push_back(string(glob_result.gl_pathv[i]));
    }
    // cleanup
    globfree(&glob_result);
    // done
    return filenames;
}

void read_food_data() {
    ifstream read_orders;
    string s;
    stringstream ss;
    // Input restaurant prep times
    // id, time, time_std
    read_orders.open(global_conf.data_base_location + "food_data/" + simulation_day +"/rest_prep_time.csv");
    getline(read_orders, s);
    while (getline(read_orders, s)){
        ss.str(string());
        ss.clear();
        ss<<s;
        string r_id;
        double p_time, p_std;
        ss>> r_id >> p_time >> p_std;
        rest_prep_time[r_id] = p_time;
    }
    read_orders.close();
    cout << "Finish taking prep time input \nRestaurants = " << rest_prep_time.size() << "\n";
    // Input slotted restaurant prep times
    // id, time, time_std, timeslot
    read_orders.open(global_conf.data_base_location + "food_data/" + simulation_day +"/rest_prep_time_slotted.csv");
    getline(read_orders, s);
    while (getline(read_orders, s)){
        ss.str(string());
        ss.clear();
        ss<<s;
        string r_id;
        double p_time, p_std;
        int timeslot;
        ss>> r_id >> p_time >> p_std >> timeslot;
        slotted_rest_prep_time[timeslot][r_id] = p_time;
        slotted_rest_prep_time_std[timeslot][r_id] = p_std;
    }
    read_orders.close();
    cout << "Finish taking prep time input \nRestaurants = " << rest_prep_time.size() << "\n";
    // Input orders
    // order_id,ordered_time,restaurant_id,rest_node,
    // rest_lat_long,node_id,customer_lat_lng,item_count,sla
    read_orders.open(global_conf.data_base_location + "food_data/" + simulation_day +"/orders.csv");
    getline(read_orders, s);
    while (getline(read_orders, s)){
        ss.str(string());
        ss.clear();
        ss<<s;
        string order_id, r_id, r_latlon, c_latlon;
        double order_time, sla;
        long long int r_node, c_node, item_count;
        ss>> order_id >> order_time >> r_id >> r_node >> r_latlon >> c_node >> c_latlon >> item_count >> sla;
        if (order_time < start_time || order_time >= end_time){
            continue;
        }
        r_node = id_to_node[r_node];
        c_node = id_to_node[c_node];
        int curr_timeslot = ((((long long int)(order_time))%86400 + 86400)%86400)/3600;
        double rest_p_time = get_prep_time(r_id, curr_timeslot);
        double sdt = rest_p_time + query_naive(r_node, c_node, order_time + rest_p_time);
        if (sdt + FP_EPSILON >= MAX_NUM)
            continue;
        if (VERBOSITY == -1){
            cout << fixed << "SDT,"<< order_id << "," << sdt << "," << order_time << "," << rest_p_time <<"," << sla << "\n";
        }
        order new_order(order_id, order_time, r_id, r_node, r_latlon, c_node,
                        c_latlon, rest_p_time, sdt, item_count, sla);
        realtime_orders.push_back(new_order);
    }
    read_orders.close();
}

void vehicle_init(){
    ifstream read_vh;
    string s;
    stringstream ss;
    // Used to sample a fraction of drivers
    mt19937 rng(global_conf.random_seed);
    bernoulli_distribution dist(fraction_drivers);
    // List DE files
    vector<string> files = glob_listdir(global_conf.data_base_location + "food_data/" + simulation_day +"/de_intervals/" + "*");
    for (int i = 0; i < int(files.size()); i++){
        if (dist(rng) == 0)
            continue;
        int name_st = (global_conf.data_base_location + "food_data/" + simulation_day +"/de_intervals/").size();
        int name_len = files[i].size() - name_st - 4;
        string vh_id = files[i].substr(name_st, name_len);
        vector<active_interval> de_int;
        read_vh.open(files[i]);
        getline(read_vh, s);
        while(getline(read_vh, s)){
            ss.str(string());
            ss.clear();
            ss<<s;
            double st_time;
            long long int st_node;
            ss >> st_time >> st_node;
            getline(read_vh, s);
            ss.str(string());
            ss.clear();
            ss<<s;
            double en_time;
            long long int en_node;
            ss >> en_time >> en_node;
            // Increase end time of drivers working after 11:55 PM of date x by 5 minutes, assing last order set at 12:00 AM
            // Else from real data, get record of next day DE and append
            int end_date = (en_time + 300)/86400;
            if (end_date >= 1)
                en_time += 300;
            st_node = id_to_node[st_node];
            active_interval intv(st_time, en_time, st_node);
            de_int.push_back(intv);
        }
        vehicle new_vehicle(vh_id, global_conf.vehicle_max_items, de_int);
        all_vehicles.push_back(new_vehicle);
        read_vh.close();
    }
}
