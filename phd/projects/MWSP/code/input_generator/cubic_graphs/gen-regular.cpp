/*
	This code implements the random generator of K-regular graphs descrived in the 
	paper "Generating random regular graphs quickly", A. STEGER and N. C. WORMALD, 
	Journal Combinatorics, Probability and Computing, 1999.

	For random generation of numbers we use the library provided by PCG: 
	http://www.pcg-random.org

	This code must be compiled using C++11

	Author: Renzo Gonzalo Gomez Diaz
	Last update: 18 / 10 / 2016
*/
// #include <stdio.h>      /* printf, scanf, puts, NULL */
// #include <stdlib.h>     /* srand, rand */
// #include <time.h>       /* time */
#include <random>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>

#include<cstdio>
#include<iostream>
#include<cstdlib>
#include<cassert>
#include<climits>
#include<list>
#include<algorithm>
#include<random>
#include "pcg_random.hpp"
#include "randutils.hpp"

using namespace std;
// Max num. vertices
const int MAXN = 256;
// Max degree
const int MAXK = 9;

bool G[MAXN][MAXN];
list<int> U;

const int NO_EDGE_CONST = 999999;
const int MAX_STR_LENGTH = 1024;

int main(int argc, char **argv){

  //argv[0]: number of vertices
  //argv[1]: degree
  //argv[2]: lowest cost
  //argv[3]: highest cost
  // /* initialize random seed: */
  // srand (time(NULL));

  // /* generate secret number between 1 and 10: */
  // r = rand() % 10 + 1;


	argv++, argc--;
	if ( argc != 2 ){
		cout << "Error:\nUsage: gen-regular N k\nN: Num. of vertices.\nk: Degree of the vertices.\n";
		return 1;
	}
	int N = atoi(argv[0]);
	int K = atoi(argv[1]);
	// int lowestCost = atoi(argv[2]);
	// int highestCost = atoi(argv[3]);
	// Checking the bounds on N and K
	if ( N < 2 || N > MAXN ){
		cout << "Error: 2 <= N <= " << MAXN << endl;
		return 1;
	}
	if ( K < 3 || K > MAXN ){
		cout << "Error: 3 <= K <= " << MAXK << endl;
		return 1;
	}
	// N must be even
	if ( N & 1 ){
		cout << "Error: N must be even" << endl;
		return 1;
	}
	// Initialization of the structures
	for ( int i=0; i < N*K; ++i ){
		U.push_back(i);
	}
	for ( int i=0; i < N; ++i ) fill( G[i] , G[i] + N, false );
	/* Create k-regular graph */
	int left = N * K;
	int a, b;

	randutils::random_generator<pcg32> rng;

	while ( left > 2 ){
		list<int>::iterator it1 = rng.choose(U);
		list<int>::iterator it2 = rng.choose(U);
		a = *it1 / K;
		b = *it2 / K;
		if ( a == b ) continue;
		if ( !G[a][b] ){
			G[a][b] = G[b][a] = true;
			U.erase( it1 );
			U.erase( it2 );
			left -= 2;
		}
	}

	a = U.front() / K, b = U.back() / K;
	if ( G[a][b] ){ //Bad simulation
			return 1;
			//cout << "Bad simulation!" << endl;
	}
	// Instance created
	G[a][b] = G[b][a] = true;
	int M = (N * K) >> 1;
	//cout << N << " " << M << endl;


	double costs[]= {1,2,4,8,16};
	std::random_device rdev;
	std::default_random_engine re(rdev());
	//std::mt19937 rgen(rdev());
	std::uniform_int_distribution<int> idist(0,4); //(inclusive, inclusive)
	int cost[N][N]; 


	for ( int i=0; i < N; ++i ) {
	  cost[i][i] = NO_EDGE_CONST;
	  for ( int j=i+1; j < N; ++j ) {
	    if ( G[i][j] ) {
	      //cout << i << " " << j << endl;
	      //double val = costs[idist(re)];
	      double val = 1;
	      cost[i][j] = val;
	      cost[j][i] = val;
	    } else {
	      cost[i][j] = NO_EDGE_CONST;
	      cost[j][i] = NO_EDGE_CONST;
	    }
	  }
	}

	ofstream costOutFile;
	char fileName[40];
	snprintf(fileName, sizeof(fileName), "%d-unit-input-cubic.dat", N);	
	//costOutFile.open (fileName, ios::out | ios::app);
	//cout << "vai criar arquivo de saida: " << fileName << endl;
	costOutFile.open (fileName, ios::out);
	costOutFile << "[";
	for (int i = 0; i < N; i++) {
	  char row[MAX_STR_LENGTH] = "[";
	  for (int j = 0; j < N; j++) {
	    char buffer[10];  // make sure this is big enough!!!
	    if (G[i][j]) {
	      snprintf(buffer, sizeof(buffer), "%d", (int)cost[i][j]);	      
	    } else {
	      snprintf(buffer, sizeof(buffer), "%d", NO_EDGE_CONST);
	    }
	    strcat(row, buffer);
	    if (j < (N-1)) {
	      strcat(row, ", ");
	    }
	  }
	  strcat(row, "]");
	  if (i < (N-1)) {
	    strcat(row, ",");
	  }
	  costOutFile << row << endl;
	}
	costOutFile << "]";
	costOutFile.close();
 return 0;
}
