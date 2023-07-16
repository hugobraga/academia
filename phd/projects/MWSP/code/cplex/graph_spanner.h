#ifndef GRAPH_SPANNER_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define GRAPH_SPANNER_H

#include <list>
#include <math.h>
#include <assert.h>
#include <algorithm>
#include <ilcplex/ilocplex.h>
#include <map>
#include <lemon/maps.h> //ArcMap
#include<set>

//------------lemon---------
#include <lemon/dijkstra.h>
#include <lemon/list_graph.h> //ListDigraph
//--------------------------

//#include "trie_for_path.h"
/* #define TREE_CONSTRAINT 1 */

ILOSTLBEGIN

using namespace std;
using std::list;
using namespace lemon;

const int TIME_LIMIT = 1800;

/* void readInput(); */
/* void resetTime(); */
/* float getSpentTime(); */

//----------------------------------
typedef IloArray<IloNumVarArray> IloNumVarArray2;

//-------forwarding declarations---------
struct OurEdge;
//---------------------------------------

//--------tuples
/* typedef std::tuple<OurEdge*,OurEdge*> edge2tuple; */

//------------types used by lemon---------
typedef ListDigraph Digraph;
typedef Digraph::ArcMap<double> LengthMap;
//typedef map<Digraph::Arc,float> CostMap;

typedef std::map<Digraph::Arc,OurEdge*> EdgeMap;
typedef std::map<Digraph::Node,bool> Node2BoolMap;
/* typedef std::map<OurEdge*,int> Edge2IntMap; */
typedef std::map<OurEdge*,IloNum> Edge2FracMap;

//----------------------------------------

typedef struct OurEdge {
    int u;
    int v;
    int label;
    IloNum cost;
    IloNumVar varX;
    IloConversion* conv;
    Digraph::Arc uv;
    Digraph::Arc vu;

    int disabled;

    bool bridge;
    bool unallowed;

    int flag;

  int height; //temporario
  //TrieNode* ptr;

  bool operator==(struct OurEdge other) {
    return ((u == other.u) && (v == other.v));
  }
} OurEdge;

struct OurEdgeComparator
{
  bool operator()(OurEdge* item)
  {
    return toFind == *item;
  }
  OurEdge toFind;
};

typedef struct InputData {
    int nK;
    int nEdges;
    int nNodes;
    double strFactor;
    vector<OurEdge*> edges;
    vector<pair<int,int> > pairs;
    double** dist;
    int unit;    
} InputData;

typedef struct CplexEnv {
    IloEnv env;
    IloModel* model;
    IloNumVarArray varXArray;
    IloNumVarArray2 varYArray;
  #ifdef TREE_CONSTRAINT
  IloRange* tree_constraint;
  #endif
    IloObjective* obj;
    IloCplex* solver;
  
    IloNum tol;
} CplexEnv;



#define NUMBER_OF_BRANCHED_VARIABLES 1
//--------------------------

#define FOR_EACH_EDGE_e for(int indE = 0; indE < input.nEdges; indE++) { if (input.edges[indE]->disabled) continue; OurEdge* e = input.edges[indE];

#define FOR_EACH_EDGE_f for(int indF = 0; indF < input.nEdges; indF++) { if (input.edges[indF]->disabled) continue; OurEdge* f = input.edges[indF];


#endif
