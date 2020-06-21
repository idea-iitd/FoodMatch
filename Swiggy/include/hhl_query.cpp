#include <bits/stdc++.h>

#include "labeling.hpp"
#include "global.hpp"
#include "constants.hpp"

using namespace std;

hl::Labeling hhl_labels;

void load_labels(int timeslot){
    hhl_labels.clear();
    string prefix = global_conf.data_base_location + "map/" + simulation_day +"/per_hour_edges/dimacs";
    string label_file = (prefix + "_" + to_string(timeslot) + ".label");
    char *cstr = new char[label_file.length() + 1];
    strcpy(cstr, label_file.c_str());
    hhl_labels.read(cstr);
    time_opt.clear();
}

double hhl_sp_query(int u, int v){
    long long int hash = ((long long int) u)*((long long int) node_id.size()) + ((long long int) v);
    if (time_opt.find(hash) != time_opt.end())
        return time_opt[hash];
    double dist = double(hhl_labels.query(u, v));
    if (dist > MAX_NUM){
        dist = MAX_NUM;
    }
    time_opt.emplace(hash, dist);
    return dist;
}
