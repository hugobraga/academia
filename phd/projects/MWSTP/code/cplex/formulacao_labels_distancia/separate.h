#ifndef SEPARATE_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define SEPARATE_H

#include <vector>

//------------lemon---------
#include <ilcplex/ilocplex.h>
//--------------------------


std::vector<int> separate(IloNumArray sol, IloNum tol, int nNodes, int nEdges);

#endif
