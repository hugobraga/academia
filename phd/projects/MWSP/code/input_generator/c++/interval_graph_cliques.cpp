#include <iostream>     // std::cout
#include <algorithm>    // std::shuffle
//#include <array>        // std::array
#include <vector>
#include <list>

#include <iostream>
#include <fstream>
#include <sstream>

using namespace std;

int n;

void generateEdges(std::list<std::pair<int,int> > &interv, int** edgeFlag) {
  
  std::vector<int> vertices;
  int nElems = 0;
  // int nEdges = 0;

  // ofstream outputFile;
  // char temp[20];
  // sprintf(temp, "%d-%d-interval.dat", inst, n);
  // outputFile.open(temp);  
  // outputFile << n << " " << nEdges << endl;  

  for (list<std::pair<int,int> >::iterator it=interv.begin(); it != interv.end(); it++) {
    std::pair<int, int> ext = *it;

    int elem = ext.first;
    int last = ext.second;

    if (elem == last) {
      nElems++;
      vertices.push_back(elem);
    } else {
      while (elem <= last) {
	nElems++;
	vertices.push_back(elem);
	elem++;
      }
    }    
  }

  for (int i = 0; i < nElems; i++) {
    std::vector<int>::iterator iter = std::next(vertices.begin(), i);
    int u = *iter;
    for (int j = (i+1); j < nElems; j++) {
      std::vector<int>::iterator iter2 = std::next(vertices.begin(), j);
      int v = *iter2;
      edgeFlag[u][v] = 1;
      edgeFlag[v][u] = 1;
      //nEdges++;
      //outputFile << u << " " << v << " 1" << endl;      
    }
  }
}

int main(int argc, char* argv[]) {
  std::list<std::pair<int,int>  > intervalos;

  n = atoi(argv[1]);
  int nEdges = 0;

  int** edgeFlag;
  edgeFlag = new int *[n];

  for (int i=0; i<n; ++i) {
    edgeFlag[i] = new int[n];
  }
  
  for (int i=0; i<n; ++i) {
    // edgeFlag[i] = new int[n];
    for (int j = (i+1); j < n; j++) {
      edgeFlag[i][j] = 0;
      edgeFlag[j][i] = 0;
    }
  }


  //-------------reading cliques file---------------
  int nCliques;
  std::ifstream infile(argv[3]);
  std::string line;
  getline(infile, line);
  istringstream issHead(line);

  issHead >> nCliques;

  for (int i = 0; i < nCliques; i++) {
    int first, last;
    first = 0;
    intervalos.clear();
    while (first != -1) {
      getline(infile, line);
      istringstream iss(line);
      if (!(iss >> first >> last )) { break; }
      if (first != -1)
	intervalos.push_back(std::make_pair(first,last));
    }
    generateEdges(intervalos, edgeFlag);    
  }    
  //-----------------------------------------------

  //couting number of edges
  for (int i = 0; i < n; i++) {
    for (int j = (i+1); j < n; j++) {
      if (edgeFlag[i][j]) {
	nEdges++;
      }
    }
  }
  
  ofstream outputFile;
  char temp[20];
  int instancia = atoi(argv[2]);
  sprintf(temp, "%d-%d-interval.dat", instancia, n);
  //outputFile.open(temp, std::ofstream::out | std::ofstream::app);
  outputFile.open(temp);
  outputFile << n << " " << nEdges << endl;  

  for (int i = 0; i < n; i++) {
    for (int j = (i+1); j < n; j++) {
      if (edgeFlag[i][j]) {
	outputFile << i << " " << j << " 1" << endl;
      }
    }
  }
  
  outputFile.close();
}
