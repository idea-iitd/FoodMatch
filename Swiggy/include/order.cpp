#include <bits/stdc++.h>
#include "order.hpp"

using namespace std;

order::order() {
}

order::order(string o_id, double o_time, string r_id, 
            long long int r_node, string r_latlon, long long int c_node,
            string c_latlon, double p_time, double sdt,
            long long int item_count, double sla){
    this->order_id = o_id;
	this->order_time = o_time;
    this->restaurant.rest_id = r_id;
    this->restaurant.rest_node = r_node;
    this->restaurant.rest_latlon = r_latlon;
    this->customer.cust_node = c_node;
    this->customer.cust_latlon = c_latlon;
	this->items = item_count;
    this->prep_time = p_time;
    this->shortest_delivery_time = sdt;
    this->SLA = sla;
}

bool order::operator<(const order &other) const {
    if (this->order_time == other.order_time)
        return (this->order_id).compare(other.order_id) < 0;
    return this->order_time < other.order_time;
}