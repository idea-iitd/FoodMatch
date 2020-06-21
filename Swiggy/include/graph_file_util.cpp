#include <bits/stdc++.h>
#include "global.hpp"
#include "config.hpp"
#include "order.hpp"
#include "graph_file_util.hpp"
#include "constants.hpp"

using namespace std;

void read_graph_input(){
    ifstream Location;
    string s;
    stringstream ss;
    char ch;
    int num_nodes = 0, num_edges = 0;
    // Input index_to_node_id
    // index,node_id
    cout << global_conf.data_base_location + "map/" + simulation_day +"/index_to_node_id" << endl;
    Location.open(global_conf.data_base_location + "map/index_to_node_id");
    while(getline(Location, s)){
        ss.str(string());
        ss.clear();
        ss<<s;
        long long int id, index;
        ss>>index>>ch>>id;
        id_to_node[id] = index;
    }
    Location.close();
    num_nodes = id_to_node.size();
    node_id.resize(num_nodes);
    for(auto it: id_to_node)
        node_id[it.second] = it.first;
    cout<<"# of nodes= "<<node_id.size()<<"\n";
    // Input node_id,lat_lon
    // node_id,lat,lon
    nodes_to_latlon.resize(num_nodes);
    Location.open(global_conf.data_base_location + "map/location");
    while(getline(Location, s)){
        ss.str(string());
        ss.clear();
        ss<<s;
        long long int id;
        double lon, lat;
        ss>>id>>ch>>lat>>ch>>lon;
        nodes_to_latlon[id_to_node[id]] = {lat, lon};
    }
    Location.close();
    // Input edges
    // u,v,dist(meter),time(second),speed(m/s),speed_std(m/s)
    Location.open(global_conf.data_base_location + "map/" + simulation_day +"/edges");
    edges.resize(num_nodes);
    edge_weight.resize(num_nodes);
    edge_dist.resize(num_nodes);
    getline(Location, s);
    while(getline(Location, s)){
        ss.str(string());
        ss.clear();
        ss<<s;
        long long int node1, node2;
        double dist, time_needed, speed_std, speed;
        ss>>node1>>ch>>node2>>ch>>dist>>ch>>time_needed>>ch>>speed>>ch>>speed_std;
        node1 = id_to_node[node1];
        node2 = id_to_node[node2];
        edges[node1].push_back(node2);
        edge_weight[node1].push_back(time_needed);
        edge_dist[node1].push_back(dist);
        num_edges++;
    }
    Location.close();
    // Slotted input edges
    // u,v,dist(meter),time_weight(second), speed(m/s), speed_std(m/s), timeslot
    Location.open(global_conf.data_base_location + "map/" + simulation_day +"/edges_slotted");
    for (int i = 0; i < 24; i++){
        slotted_edge_speed[i].resize(num_nodes);
        slotted_edge_speed_std[i].resize(num_nodes);
    }
    for(int i = 0; i < num_nodes; i++){
        int sz = edges[i].size();
        for (int j = 0; j < 24; j++){
            slotted_edge_speed[j][i].resize(sz);
            slotted_edge_speed_std[j][i].resize(sz);
        }
    }
    getline(Location, s);
    while(getline(Location, s)){
        ss.str(string());
        ss.clear();
        ss<<s;
        long long int node1, node2;
        double dist, time_needed, speed, speed_std;
        int timeslot;
        ss>>node1>>ch>>node2>>ch>>dist>>ch>>time_needed>>ch>>speed>>ch>>speed_std>>ch>>timeslot;
        node1 = id_to_node[node1];
        node2 = id_to_node[node2];
        int node2_pos = -1;
        for(int i = 0; i < int(edges[node1].size()); i++){
            if (edges[node1][i] == node2){
                node2_pos = i;
                break;
            }
        }
        slotted_edge_speed[timeslot][node1][node2_pos] = speed;
        slotted_edge_speed_std[timeslot][node1][node2_pos] = speed_std;
    }
    Location.close();
    slotted_max_time.resize(24);
    for (int curr_time_slot=0; curr_time_slot < 24; curr_time_slot++){
        double max_time_dist = -1;
        for(int u = 0; u < nodes_to_latlon.size(); u++){
            for (int i = 0; i < int(edges[u].size()); i++){
                double speed = slotted_edge_speed[curr_time_slot][u][i];
                double dist = edge_dist[u][i];
                max_time_dist = max(max_time_dist, dist/speed);
            }
        }
        if ((max_time_dist >= MAX_NUM) || (max_time_dist <= 0)){
            throw "MAX TIME DIST ERROR\n";
        }
        cout << "max_edge_weight " << curr_time_slot << " => " << max_time_dist << endl;
        slotted_max_time[curr_time_slot] = max_time_dist;
    }
}
