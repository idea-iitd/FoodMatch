#pragma once

#include <bits/stdc++.h>

using namespace std;

// Load HHL Index labels for the "timeslot"
void load_labels(int timeslot);

// HHL shortest path query between nodes x and y
double hhl_sp_query(int x, int y);