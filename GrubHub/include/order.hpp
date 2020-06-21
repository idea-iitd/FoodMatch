#pragma once

#include <bits/stdc++.h>

using namespace std;

// Food Order
class order{
public:
    // Order ID
    string order_id;

    // order timestamp in seconds
    double order_time;

    // Restaurant ID
    string rest_id;

    // Customer Position
    double x, y;

    // Food_Prep + Restaurant_to_Customer
    double shortest_delivery_time;

    // Order ready time
    double ready_time;

    order();

    // Order Constructor
    order(string o_id, double o_time, string r_id,
          double c_x, double c_y, double sdt,
          double r_time);

    bool operator<(const order &other) const;
};