#pragma once

#include <bits/stdc++.h>

using namespace std;

// Food Order
class order{
public:
    // Order ID
    string order_id;

    // order UNIX timestamp in seconds
    double order_time;

    struct {
        // Restaurant node id
        long long int rest_node;
        // Restaurant ID
        string rest_id;
        // Restaurant Position: "lat,lon"
        string rest_latlon;
    } restaurant;

    struct {
        // Restaurant node id
        long long int cust_node;
        // Customer Position: "lat,lon"
        string cust_latlon;
    } customer;

    // Food_Prep + Restaurant_to_Customer
    double shortest_delivery_time;
    // Promised Delivery time in seconds - Anonymized
    double SLA;
    // Number of Items (constraint on count)
    long long int items;
    // Restaurant order prep time
    double prep_time;

    order();

    // Order Constructor
    order(string o_id, double o_time, string r_id,
          long long int r_node, string r_latlon, long long int c_node,
          string c_latlon, double p_time, double sdt,
          long long int item_count, double sla);

    bool operator<(const order &other) const;
};