#include "event.hpp"
#include "global.hpp"

using namespace std;

event::event() {
}

event::event(order ord, int et) {
    this->order_obj = ord;
    if (et == 0){
        // If pickup event then node is restaurant node
        this->x = restaurant_id_to_coord[ord.rest_id].first;
        this->y = restaurant_id_to_coord[ord.rest_id].second;
        this->loc_id = ord.rest_id;
    }
    else{
        // If deliver event then node is customer node
        this->x = ord.x;
        this->y = ord.y;
        this->loc_id = ord.order_id;
    }
    this->type = et;
}

bool event::operator==(const event& other){
    return ((this->type == other.type)&&((this->order_obj.order_id) == other.order_obj.order_id));
}

string event::str_val(){
    if (this->type == 0)
        return this->order_obj.order_id + " Pickup";
    else
        return this->order_obj.order_id + " Delivery";
}
