#pragma once

#include <bits/stdc++.h>

using namespace std;

// Disjoint Set Union
class DSU{
public:
    // element to set
    vector<int> id;

    // set to all elements
    unordered_map<int, vector<int>> set_to_elems;

    DSU();
    // n - number of elements
    DSU(int n);

    // initialize DSU of size n
    void init(int n);

    // find operation
    int root(int x);

    // union operation
    // set(y) is equal to set(x)
    void merge(int x, int y);

    // get all elements in same set as x
    vector<int> get_elems(int x);
};