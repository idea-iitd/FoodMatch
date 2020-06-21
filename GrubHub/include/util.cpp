#include <bits/stdc++.h>
#include "constants.hpp"
#include "util.hpp"

double get_distance(double x1, double y1, double x2, double y2){
    double euclid = pow((x1-x2), 2) + pow((y1-y2), 2);
    return sqrt(euclid);
}

double get_travel_time(double x1, double y1, double x2, double y2){
    double dist = get_distance(x1, y1, x2, y2);
    return ceil(dist/vehicle_speed);
}