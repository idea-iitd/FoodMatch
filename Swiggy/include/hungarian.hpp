#pragma once

#include <bits/stdc++.h>

using namespace std;

// Take an input rectangular cost matrix and return an assigment minimizing cost
// Taken from - https://bougleux.users.greyc.fr/lsape/ 
double HUN_ASSIGN(vector <vector<double> >& DistMatrix, vector<int>& Assignment);