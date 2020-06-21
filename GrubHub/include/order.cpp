#include <bits/stdc++.h>
#include "order.hpp"

using namespace std;

order::order() {
}

order::order(string o_id, double o_time, string r_id,
          double c_x, double c_y, double sdt,
          double r_time){
    this->order_id = o_id;
    this->order_time = o_time;
    this->rest_id = r_id;
    this->x = c_x;
    this->y = c_y;
    this->ready_time = r_time;
    this->shortest_delivery_time = sdt;
}

bool order::operator<(const order &other) const {
    if (this->order_time == other.order_time)
        return (this->order_id).compare(other.order_id) < 0;
    return this->order_time < other.order_time;
}