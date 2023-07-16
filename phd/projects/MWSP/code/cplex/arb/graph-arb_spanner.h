#include <list>
#include <math.h>
#include <vector>
#include <assert.h>
#include <algorithm>
#include <ilcplex/ilocplex.h>
#include <map>
#include <lemon/maps.h> //ArcMap

//------------lemon---------
#include <lemon/list_graph.h> //ListDigraph
#include <lemon/dijkstra.h>
//--------------------------

ILOSTLBEGIN

using namespace std;
using namespace lemon;
using std::list;

#define RC_EPS 1.0e-6
const double tolerance = RC_EPS;
const int NO_EDGE_CONST = 999999;
const int NUMBER_ARB = 3;
const int TIME_LIMIT = 3600;

/* void readInput(); */
/* void resetTime(); */
/* float getSpentTime(); */


//-------forwarding declarations---------
struct Edge;
struct EdgeArcs;
struct Node;
//------------types used by lemon---------
typedef ListDigraph Digraph;
typedef Digraph::ArcMap<IloNum> LengthMap;
//typedef map<Digraph::Arc,float> LengthMap;
//typedef Digraph::ArcMap<Edge*> EdgeMap;
typedef map<Digraph::Arc,Edge*> EdgeMap;
typedef map<Edge*,EdgeArcs*> ReverseEdgeMap;
//typedef Digraph::NodeMap<CSPPNode*> NodeMap;
//typedef map<Digraph::Node,CSPPNode*> NodeMap;
typedef map<Digraph::Node,int> ReverseNodeMap;
typedef map<int,Node*> NodeMap;
typedef IloArray<IloNumVarArray> IloNumVarArray2;
typedef IloArray<IloNumVarArray2> IloNumVarArray3;
//typedef IloArray<IloNumArray> IloNumArray2;
typedef IloArray<IloNumArray2> IloNumArray3;


/* typedef struct LemonObjects { */
/*   Digraph *digraph;   */
/*   vector<Digraph::Node> digraphNodes; */
/*   LengthMap weight; */
/*   LengthMap oppWeight; */
/*   EdgeMap edgeMap; */
/*   ReverseEdgeMap revEdgeMap; */
/*   ReverseNodeMap nodeLabelMap; */
/*   NodeMap labelNodeMap; */
/* }LemonObjects; */

typedef struct EdgeArcs {
  Digraph::Arc in;
  Digraph::Arc out;
  int label;
} EdgeArcs;
//----------------------------------------

//----------------------------------------

typedef struct Node {
  int label;
  Digraph::Node* nodeRef;
}Node;

typedef struct Edge {
    int u;
    int v;
  int label;
    float cost;
    IloNumVar* varX;
    IloConversion* conv;
} Edge;

typedef struct InputData {
  int nEdges;
  float strFactor;
  int nNodes;
  vector<Edge*> edges;
  vector<EdgeArcs*> arcs;
  vector<Node*> nodes;
  float** dist;  
  IloNumArray2 edgeCost;
} InputData;

typedef struct CplexEnv {
    IloEnv env;
    IloModel* model;
    IloCplex* solver;
  IloNum lb;
  IloNum ub;
  IloNumVarArray xSol;
  IloNumVarArray2 x;
  IloNumVarArray3 y;
  IloNumVarArray3 z;

  //IloNumArray xSolVal; 
    
} CplexEnv;

#define FOR_EACH_EDGE_e for(int i = 0; i < input.nEdges; i++) { Edge* e = input.edges[i];
#define FOR_EACH_EDGE_f for(int y = 0; y < input.nEdges; y++) { Edge* f = input.edges[y];
#define FOR_EACH_NODE_u for(int i = 0; i < input.nNodes; i++) { Node* u = input.nodes[i]; //Digraph::Node* u = node->nodeRef; int l1 = node->label;
#define FOR_EACH_NODE_v for(int z = 0; z < input.nNodes; z++) { Node* v = input.nodes[z]; //Digraph::Node* v = node->nodeRef; int l2 = node->label;
#define FOR_EACH_ARC_pair for(int z = 0; z < input.nEdges; z++) { EdgeArcs* pair = input.arcs[z];
