#pragma once

#include <bits/stdc++.h>

using namespace std;

// Configuration Structure
struct config
{
    /* Parameters */
    string data_base_location;

    //MAXI
    long long int vehicle_max_items;

    // seed for reproducibility
    int random_seed;

    // City specific parameters
    double max_vh_speed;
    double min_vh_speed;
    double max_rest_p_time;
    double min_rest_p_time;
    double avg_rest_p_time;
    double std_rest_p_time;

    void read_from_file(const string path_to_config);
};
