#include "event.hpp"

using namespace std;

event::event() {
}

event::event(order ord, int et) {
    this->order_obj = ord;
    if (et == 0){
        // If pickup event then node is restaurant node
        this->node = ord.restaurant.rest_node;
    }
    else{
        // If deliver event then node is customer node
        this->node = ord.customer.cust_node;
    }
    this->type = et;
}

string event::str_val(){
    if (this->type == 0)
        return this->order_obj.order_id + " Pickup";
    else
        return this->order_obj.order_id + " Delivery";
}
