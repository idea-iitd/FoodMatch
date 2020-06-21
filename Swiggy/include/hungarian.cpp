#include "hungarian.hpp"
#include "hungarian-lsap.cpp"

#include <chrono>

double HUN_ASSIGN(vector <vector<double> >& DistMatrix, vector<int>& Assignment){
	int nbR = DistMatrix.size(), nbC = DistMatrix[0].size();
	double *C = new double[nbR*nbC];
	int *rho = new int[nbR];
	int *varrho = nullptr;
	double *u = new double[nbR], *v = new double[nbC];  // dual variables
	for (int c = 0; c < nbC; c++) {
		for (int r = 0; r < nbR; r++) {
			C[nbR*c+r] = DistMatrix[r][c];
		}
	}
	hungarianLSAP(C,nbR,nbC,rho,u,v);
	double optSol = 0.0;
	for (int c = 0; c < nbC; c++) optSol += v[c];
	for (int r = 0; r < nbR; r++) optSol += u[r];
	for (int r = 0; r < nbR; r++) Assignment[r] = rho[r];
	delete[] C;
	delete[] rho;
	delete[] u;
	delete[] v;
	return optSol;
}
