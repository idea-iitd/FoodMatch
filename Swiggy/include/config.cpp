#include <bits/stdc++.h>
#include "config.hpp"

using namespace std;

void config::read_from_file(const string path_to_config) {
    ifstream config_file;
    config_file.open(path_to_config);
    string line;
    while (getline(config_file, line)) {
        string key, value;
        stringstream ss_line(line);
        if (getline(ss_line, key, '=')) {
            getline(ss_line, value);
            stringstream ss(value);
            if (key.compare("data_base_location") == 0) {
                ss >> this->data_base_location;
            }
            else if (key.compare("vehicle_max_items") == 0) {
                ss >> this->vehicle_max_items;
            }
            else if (key.compare("random_seed") == 0){
                ss >> this->random_seed;
            }
            else if (key.compare("min_vh_speed") == 0){
                ss >> this->min_vh_speed;
            }
            else if (key.compare("max_vh_speed") == 0){
                ss >> this->max_vh_speed;
            }
            else if (key.compare("max_rest_p_time") == 0){
                ss >> this->max_rest_p_time;
            }
            else if (key.compare("min_rest_p_time") == 0){
                ss >> this->min_rest_p_time;
            }
            else if (key.compare("avg_rest_p_time") == 0){
                ss >> this->avg_rest_p_time;
            }
            else if (key.compare("std_rest_p_time") == 0){
                ss >> this->std_rest_p_time;
            }
        }
    }
    config_file.close();
}
