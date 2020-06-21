#include <bits/stdc++.h>
#include "global.hpp"
#include "order.hpp"
#include "food_data_util.hpp"
#include "vehicle.hpp"
#include "constants.hpp"
#include "util.hpp"

using namespace std;

void read_instance_parameters(){
    ifstream read_para;
    string s;
    stringstream ss;
    // input
    // meters_per_minute       pickup service minutes  dropoff_service_minutes
    // target click-to-door    maximum click-to-door   pay_per_order   guaranteed_pay_per_hour
    read_para.open(data_location + "/instance_parameters.txt");
    getline(read_para, s);
    getline(read_para, s);
    ss.str(string());
    ss.clear();
    ss << s;
    double v_sp, p_min, d_min, t_ctd, m_ctd, ppo, gph;
    ss >> v_sp >> p_min >> d_min  >> t_ctd >> m_ctd >> ppo >> gph;
    cout << v_sp << " " << p_min << " " << d_min  << " " << t_ctd << " " << m_ctd << " " << ppo  << " " <<  gph << endl;
    read_para.close();
    vehicle_speed = v_sp/60.0;
    pickup_service = p_min*60.0;
    dropoff_service = d_min*60.0;
    target_ctd = t_ctd*60.0;
    max_ctd = m_ctd*60.0;
    pay_per_order = ppo;
    pay_per_hour = gph;
}

void read_rest_data(){
    ifstream read_rest;
    string s;
    stringstream ss;
    // input
    // restaurant      x       y
    read_rest.open(data_location + "/restaurants.txt");
    getline(read_rest, s);
    while(getline(read_rest, s)){
        ss.str(string());
        ss.clear();
        ss << s;
        string r_id;
        double x,y;
        ss >> r_id >> x >> y ;
        all_loc.push_back({x,y});
        restaurant_id_to_coord[r_id] = {x,y};
    }
    read_rest.close();
}

void read_order_data(){
    ifstream read_orders;
    string s;
    stringstream ss;
    // input
    // order	x	y	placement_time	restaurant	ready_time
    read_orders.open(data_location + "/orders.txt");
    getline(read_orders, s);
    while(getline(read_orders, s)){
        ss.str(string());
        ss.clear();
        ss << s;
        string o_id, r_id;
        double o_time, c_x, c_y, r_time;
        ss >> o_id >> c_x >> c_y >> o_time >> r_id >> r_time;
        r_time *= 60.0;
        o_time *= 60.0;
        double r_x = restaurant_id_to_coord[r_id].first;
        double r_y = restaurant_id_to_coord[r_id].second;
        all_loc.push_back({c_x,c_y});
        double sdt = r_time + pickup_service/2.0 + get_travel_time(c_x, c_y, r_x, r_y) + dropoff_service/2.0;
        if (VERBOSITY == -1){
            cout << fixed << "SDT,"<< o_id << "," << sdt << "," << o_time << "," << r_time << endl;
        }
        order new_order(o_id, o_time, r_id, c_x, c_y, sdt, r_time);
        realtime_orders.push_back(new_order);
    }
    read_orders.close();
    sort(realtime_orders.begin(), realtime_orders.end());
}

void vehicle_init(){
    ifstream read_vh;
    string s;
    stringstream ss;
    // input
    // courier x       y       on_time off_time
    read_vh.open(data_location + "/couriers.txt");
    getline(read_vh, s);
    while(getline(read_vh, s)){
        ss.str(string());
        ss.clear();
        ss << s;
        string v_id;
        double v_x, v_y, on_time, off_time;
        ss >> v_id >> v_x >> v_y >> on_time >> off_time;
        on_time *= 60.0;
        off_time *= 60.0;
        all_loc.push_back({v_x,v_y});
        vehicle new_vehicle(v_id, v_x, v_y, on_time, off_time);
        all_vehicles.push_back(new_vehicle);
    }
    read_vh.close();
    double max_dist_val = -1;
    for (int i = 0; i < all_loc.size(); i++){
        for (int j = i+1; j < all_loc.size(); j++){
            double dist = get_distance(all_loc[i].first, all_loc[i].second, all_loc[j].first, all_loc[j].second);
            max_dist_val = max(max_dist_val, dist);
        }
    }
    max_travel_dist = max_dist_val;
}
