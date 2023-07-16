#ifndef SPANNER_H
#define SPANNER_H

#include <list>
#include <math.h>
#include <assert.h>
#include <algorithm>
#include <ilcplex/ilocplex.h>
#include <map>
#include <lemon/maps.h> //ArcMap
#include<set>
#include <tuple>

//------------lemon---------
#include <lemon/dijkstra.h>
#include <lemon/list_graph.h> //ListDigraph
//--------------------------

//#include "trie_for_path.h"

//uma das duas opções deve ser escolhida
/* #define TREE_CONSTRAINT 1 */
/* #define TREE_OBJEC_FUNC 1 */

ILOSTLBEGIN

using namespace std;
using std::list;
using namespace lemon;

#define RC_EPS 1.0e-6
const double tolerance = RC_EPS;
const int TIME_LIMIT = 3600;
const int FAKE_TIME_LIMIT =  TIME_LIMIT + 100;

void readInput();
void resetTime();
float getSpentTime();

//-------forwarding declarations---------
struct CSPPNode;
struct OurEdge;
struct EdgeArcs;
struct SpanPath;
//---------------------------------------

//--------tuples
typedef std::tuple<OurEdge*,OurEdge*> edge2tuple;

//------------types used by lemon---------
typedef ListDigraph Digraph;
typedef Digraph::ArcMap<double> LengthMap;
//typedef map<Digraph::Arc,float> CostMap;

typedef std::map<Digraph::Arc,OurEdge*> EdgeMap;
typedef std::map<Digraph::Node,CSPPNode*> NodeMap;
typedef std::map<Digraph::Node,bool> Node2BoolMap;
/* typedef std::map<OurEdge*,int> Edge2IntMap; */
typedef std::map<OurEdge*,IloNum> Edge2FracMap;

typedef struct EdgeArcs {
  Digraph::Arc* in;
  Digraph::Arc* out;
} EdgeArcs;
//----------------------------------------

//------------types used by labelling algorithm---------
typedef struct CSPPLabel {
  //int teste;
  list<OurEdge*> p;
  bool disabled;
  //int p;
  CSPPNode* n;
  double w;
  double c;

  
} CSPPLabel;

//Representa um vértice no grafo de entrada
typedef struct CSPPNode {
  int lemonNodeRef;
  list<CSPPLabel*> labels;

} CSPPNode;

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

    SpanPath* single_edge_path;

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

typedef struct SpanPath {
    int uvLabel;
    OurEdge** edges;
    int length;
  //IloNumVarArray varYArray;
    IloNumVar varY;
} SpanPath;

typedef struct InputData {
    int nK;
    int nEdges;
    int nNodes;
    double strFactor;
    vector<OurEdge*> edges;
    vector<pair<int,int> > pairs;
    double** dist;
    int unit;
#if defined(TREE_CONSTRAINT)// || defined(TREE_OBJEC_FUNC)
  int solRoot;
  #endif  
} InputData;

typedef struct MasterLP {
    IloEnv env;
    IloModel* model;
    IloNumVarArray varXArray;
  #ifdef TREE_CONSTRAINT
  IloRange* tree_constraint;
  /* IloRange* tree_constraint_le; */
  #endif
    IloRangeArray* edge_constraints;
    IloRangeArray* path_constraints;
    IloRangeArray* bound_constraints;
    IloObjective* obj;
    IloCplex* solver;
    vector<list<SpanPath*> > colsAdded;
    /* vector<list<SpanPath*> > colsAddedPerEdge; */
    int nAddedCols;
    /* list<SpanPath*> colsAdded; */

    // dual variables
    double* sigma;
    double* pi;
  #ifdef TREE_CONSTRAINT  
    double alpha;
    double beta;
  #endif

  double gap;
  double relGap;
  // double minLB;
  // double incumbent;
  /* double bestLowerBound; */
  /* #ifdef TREE_OBJEC_FUNC */
  /* double bestRealLowerBound; */
  /* #endif */
  
    IloNum tol;
} MasterLP;

//------branch tree structure
struct BranchTreeNode {
  int label;//temporário
  /*
restricao que fixa o valor de algumas variaveis em 1 (que sofreram o branch 
na arvore de decisao). Ao inves de especificar uma restrição para cada variável 
do conjunto, eu utilizo apenas uma restrição da seguinte forma:
3 * 1 <= x_e + x_f + x_g <= 3 * 1 (o 3 corresponde a quantidade de variaveis)
   */  
  //IloRangeArray* branch_one_constraint;
  /* IloRangeArray* branch_one_constraint; */
  IloRange* branch_one_constraint;
  /*
restricao que fixa o valor de algumas variaveis em 0 (que sofreram o branch 
na arvore de decisao). Ao inves de especificar uma restrição para cada variável 
do conjunto, eu utilizo apenas uma restrição da seguinte forma:
0 <= x_e + x_f + x_g <= 0
   */
  /* IloRangeArray* branch_zero_constraint; */
  IloRange* branch_zero_constraint;
  /*
restricao que fixa o valor de algumas variaveis em 1 (que representam ponte naquele determinado no da árvore de decisao). Ao inves de especificar uma restrição para cada variável 
do conjunto, eu utilizo apenas uma restrição da seguinte forma:
3 * 1 <= x_e + x_f + x_g <= 3 * 1 (o 3 corresponde a quantidade de variaveis)
   */  
  IloRangeArray* bridge_constraint;

  /*
na árvore de branch, quando fixamos uma variável x_u_v em 1, a variável que 
representa o caminho de tamanho 1 entre u e v deve ser fixada em 1, pois 
ao forçar a aresta como parte da solução, na solução ótima esta aresta fará 
parte do caminho entre u e v, caso contrário, esta aresta não faria parte da 
solução ótima.
*/  
  /* IloRangeArray single_edge_path_constraint; */
  
  IloRangeArray* path_zero_constraint;
  double lower_bound;
  #ifdef TREE_OBJEC_FUNC
  double real_lower_bound;
  #endif
  //xe ?
  OurEdge* branch_edge;
  list<OurEdge*> zero_var;//variaveis que estao com valor fixo em zero na arvore de decisao
  list<OurEdge*> one_var;//variaveis que estao com valor fixo em zero na arvore de decisao
  //list<OurEdge*> bridge_var;
  list<edge2tuple> bridge_var;
  int height;
  list<OurEdge*> zero_trie_var;//arestas que foram removidas para tentar gerar novos caminhos
  BranchTreeNode *xe_zero;
  BranchTreeNode *xe_one;

  BranchTreeNode *ancestor;
  int type;
};


#define NUMBER_OF_BRANCHED_VARIABLES 1
//--------------------------

#define FOR_EACH_EDGE_e for(int indE = 0; indE < input.nEdges; indE++) { if (input.edges[indE]->disabled) continue; OurEdge* e = input.edges[indE];

#define FOR_EACH_DISABLED_EDGE_e for(int indE = 0; indE < input.nEdges; indE++) { if (!input.edges[indE]->disabled) continue; OurEdge* e = input.edges[indE];

#define FOR_EACH_TEMP_EDGE_e for(int indTE = 0; indTE < tempEdgeMap.size(); indTE++) { if (input.edges[indTE]->disabled) continue; OurEdge* e = input.edges[indTE];

/* #define FOR_EACH_EDGE_e for (set<int>::iterator it=input.edges.begin(); it!=input.edges.end(); ++it) { if (input.edges[i]->disabled) continue; OurEdge* e = *it; */
#define FOR_EACH_PAIR_uvLabel for(int uvLabel = 0; uvLabel < input.nK; uvLabel++)
#define FOR_EACH_EDGE_e_IN_PATH_p for(int eip = 0; eip < p->length; eip++) { OurEdge* e = p->edges[eip];
#define FOR_EACH_EDGE_f_IN_PATH_p for(int fip = 0; fip < p->length; fip++) { OurEdge* f = p->edges[fip];
/* #define FOR_EACH_ADDED_COLUMN for (list<SpanPath*>::iterator it = masterLP.colsAdded.begin(); it != masterLP.colsAdded.end(); it++) { */
#define FOR_EACH_ADDED_COLUMN for (std::vector<list<SpanPath*> >::iterator it2 = masterLP.colsAdded.begin() ; it2 != masterLP.colsAdded.end(); ++it2) for (list<SpanPath*>::iterator it = (*it2).begin(); it != (*it2).end(); it++) {
#define FOR_EACH_PATH_p_IN_PAIR_uvLabel for (list<SpanPath*>::iterator itP = masterLP.colsAdded[uvLabel].begin(); itP != masterLP.colsAdded[uvLabel].end(); itP++) { SpanPath* p = (*itP);

#ifdef TREE_CONSTRAINT
#define TREE_CONS (*(masterLP.tree_constraint))
/* #define TREE_CONSTRAINT_LE (*(masterLP.tree_constraint_le)) */
#endif
#define EDGE_CONSTRAINTS (*(masterLP.edge_constraints))
#define PATH_CONSTRAINTS (*(masterLP.path_constraints))
#define BOUND_CONSTRAINTS (*(masterLP.bound_constraints))
#define BRANCH_ONE_CONSTRAINT (*(curNode->branch_one_constraint))
/* #define BRANCH_ONE_CONSTRAINT (curNode->branch_one_constraint) */
#define BRANCH_ZERO_CONSTRAINT (*(curNode->branch_zero_constraint))
/* #define BRANCH_ZERO_CONSTRAINT (curNode->branch_zero_constraint) */
#define BRANCH_PATH_ZERO_CONSTRAINT (*(curNode->path_zero_constraint))
#define BRIDGE_CONSTRAINT (*(curNode->bridge_constraint))
/* #define BRIDGE_CONSTRAINT (curNode->bridge_constraint) */
/* #define SINGLE_EDGE_PATH_CONSTRAINT (*(curNode->single_edge_path_constraint)) */
#define SINGLE_EDGE_PATH_CONSTRAINT (curNode->single_edge_path_constraint)
//#define FOR_EACH_ARC_pair for(int z = 0; z < input.nEdges; z++) { EdgeArcs* pair = input.arcs[z];

#endif
