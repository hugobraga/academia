#include <iostream>     // std::cout
#include <algorithm>    // std::shuffle
//#include <array>        // std::array
#include <vector>
#include <random>       // std::default_random_engine
#include <chrono>       // std::chrono::system_clock

#include <iostream>
#include <fstream>

#include <lemon/connectivity.h>
#include <lemon/list_graph.h>
#include <lemon/adaptors.h>

using namespace std;
using namespace lemon;

int main(int argc, char* argv[]) {
  int n = atoi(argv[1]);
  int qtInst = atoi(argv[2]);
  int inst = 0;

  while (inst < qtInst) {
  // for (int inst = 0; inst < qtInst; inst++) {

    //std::array<int, 2* n> indices;
    std::vector<int> indices;

    int edgeFlag[n][n];

    for (int i=0; i<n; ++i) {
      indices.push_back(i); 
      indices.push_back(i); 

      for (int j = (i+1); j < n; j++) {
	edgeFlag[i][j] = 0;
	edgeFlag[j][i] = 0;
      }
    }
    //indices.reserve(2*n);
  
    //taking values of elements from user
    // for(int i = 0; i < n ; i++) {
    //   indices[2*i] = i;
    //   indices[2*i + 1] = i;
    // }
    
    // obtain a time-based seed:
    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()    
    // unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    // shuffle (indices.begin(), indices.end(), std::default_random_engine(seed));
    //shuffle
    //for (int i=(last-first)-1; i>0; --i) {
    for (int i=(indices.size()-1); i>0;) {
      int prop = n/2;
      int menor = max(0, (i-1)-prop);
      int maior = i-1;
      if (menor == 0) {
	menor = i-1;
	maior = (i-1) + prop;
      }
      // int elem = max(0, (i-1)-prop);
      // std::uniform_int_distribution<> d(elem,i-1);
      std::uniform_int_distribution<> d(menor,maior);
      //int index = d(std::default_random_engine(seed));
      int index = d(gen);
      if (index % 2!= 0)
	index = index+1;
      // cout << "vou trocar " << indices[i-1] << " com " << indices[index] << endl;
      swap(indices[i-1], indices[index]);
      i = i - 2;
    }
    // for (int i = 0; i < 2*n; i++) {
    //   cout << "indices[" << i << "]: " << indices[i] << endl;
    // }

  
    // for (int i = 0; i < 2*n; i++) {
    //   cout << "[" << i << "]: " << indices[i] << endl;
    // }
  
    for (int i = 0; i < n ; i++) {//each node
      for (int j = 0; j < 2*n ; j++) {//looking for node i
	if (indices[j] == i) {
	  for(int k = (j+1); k < 2*n ; k++) {//looking for the other node i
	    if (indices[k] == i) {
	      break;
	    } else {//in the between of nodes i -> there is an intersection of interval i and interval k
	      edgeFlag[i][indices[k]] = 1;
	      edgeFlag[indices[k]][i] = 1;
	    }
	  }
	  break; //já resolvi para o nó i
	}
      }
    }

    //testing connectivity
    ListGraph g;
    std::vector<ListGraph::Node> nodes;
    int nEdges = 0;
    nodes.reserve(n);

    for (int i = 0; i < n; i++)
      nodes[i] = g.addNode();

    for (int i = 0; i < n; i++) {
      for (int j = (i+1); j < n; j++) {
	if (edgeFlag[i][j]) {
	  g.addEdge(nodes[i], nodes[j]);
	  nEdges++;
	  // cout << "aresta entre " << i << " e " << j << endl;
	}
      }
    }
    if (connected(g)) {
      inst++;
      cout << "grafo eh conexo" << endl;
    
      ofstream outputFile;
      ofstream outputFileCliqueFormat;
      char temp[20];
      char tempCliqueFormat[20];
      sprintf(temp, "%d-%d-interval.dat", inst, n);
      sprintf(tempCliqueFormat, "%d-%d-intervalClique.dat", inst, n);
      //strcat (varName, temp);
      
      outputFile.open(temp);  
      outputFile << n << " " << nEdges << endl;
      outputFileCliqueFormat.open(tempCliqueFormat);
      outputFileCliqueFormat << n << " " << 2*nEdges << endl;

      for (int i = 0; i < n; i++) {
	for (int j = (i+1); j < n; j++) {
	  if (edgeFlag[i][j]) {
	    outputFile << i << " " << j << " 1" << endl;
	    outputFileCliqueFormat << i << "," << j << endl;
	    outputFileCliqueFormat << j << "," << i << endl;
	  }
	}
      }
    
      outputFile.close();
      outputFileCliqueFormat.close();
    }
    
  }
  
  return 0;
  
}
