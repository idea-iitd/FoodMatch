#include <bits/stdc++.h>
#include "dsu.hpp"

using namespace std;

DSU::DSU(){
}

DSU::DSU(int n){
    init(n);
}

void DSU::init(int n){
    id.resize(n);
    for(int i = 0; i < n; i++){
        id[i] = i;
        set_to_elems[i] = {i};
    }
}

int DSU::root(int x){
    while(id[x] != x){
        id[x] = id[id[x]];
        x = id[x];
    }
    return x;
}

void DSU::merge(int x, int y){
    int p = root(x);
    int q = root(y);
    id[q] = p;
    vector<int> q_elems = set_to_elems[q];
    vector<int> p_elems = set_to_elems[p];
    set_to_elems.erase(q);
    p_elems.insert(p_elems.end(), q_elems.begin(), q_elems.end());
    set_to_elems[p] = p_elems;
}

vector<int> DSU::get_elems(int x){
    int p = root(x);
    return set_to_elems[p];
}