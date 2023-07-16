#include "spanner.h"
//#include <lemon/concepts/graph.h> //OutArcIt

#include <algorithm> //std::make_heap, std::pop_heap, std::push_heap, std::sort_heap, copy
#include <queue> //priority queue
#include <list> //linked list to be used by CSPP algorithm
#include <vector> //std:vector
//#include <functional> //definir compare function
#include <set>

#include <limits> //INT_MAX
#include <math.h> //floor
#include <cmath> //abs
#include <cstdlib> //rand

#include <lemon/connectivity.h>//stronglyConnected
#include <lemon/list_graph.h>
#include <lemon/concepts/graph.h>
#include <lemon/adaptors.h>

//#include <lemon/floyd_warshall.h>
// #include "trie_for_path.h"
// #include <bitset>
// #include <algorithm> //for std::reverse
// #include <climits> //for CHAR_BIT

#include <tuple>
#include<iostream>

namespace {
  unsigned const DEBUG = 27;
  unsigned const SET_CONS_NAMES = 1;
  unsigned const SET_VAR_NAMES = 1;
  unsigned const SET_FIND_TUPLE_OF_BRIDGES = 1;
  unsigned const SAVE_VAR_BEF_MIP = 1;
  unsigned const DISABLE_INITIAL_CONS = 1;

  int const BRANCH_FINISHED = 1;
  int const TIME_EXCEEDED = -1;
  int const BRANCH_NOT_FINISHED = 0;
  int const NO_LP_SOLUTION = -2;

  int const NO_GREEDY_SOLUTION = -3;

  double const	BRANCH_HEURISTIC_HEIGHT_PERC = 0.3;
  int const	BRANCH_HEURISTIC_MIP_PROB = 0;
  int const	BRANCH_HEURISTIC_GREEDY_PROB = 100;

  #define BRANCH_HEURISTIC_MIP_INTERVAL (int) ceil((double)input.nEdges * BRANCH_HEURISTIC_HEIGHT_PERC);
};
using namespace std;

int *edgeConsFlag;
double maxNumber;			//sum of edge costs plus 1
MasterLP masterLP;
InputData input;
clock_t initialClock;
Digraph digraph;			//represents the input graph
LengthMap weight(digraph);		//weight associated with the input graph

vector<Digraph::Node> digraphNodes;	//vector of nodes of the input graph
EdgeMap edgeMap;			//map from the arc (both directions) to the edge that represents the arc

int globalId = 0;			//para teste

//-----------statistics------------------
IloNum qtZeroBranchNodes = 0;
IloNum qtOneBranchNodes = 0;
//masterLP.colsAdded.size();
double price_total_time = 0;
IloNum qtPrice = 0;

//------------store lp values---------
std::map<OurEdge*, double > lpBefMIPValues;
bool MIPFlag;

//store solution
list<OurEdge*> edgesSol;
IloNum solVal;
//-----------------------------------------

//--------------------------------------
//-------------------------------------


//----------------Headers-----------------------
void initMasterLP(IloEnv _env, IloModel* _model, /*IloRangeArray* _edge_constraints,*/ IloRangeArray* _path_constraints, IloRangeArray* _bound_constraints,
                  IloObjective* _obj,IloCplex* _solver);
void buildInitialMasterLP();
int solvePricing(LengthMap& tempWeight, EdgeMap& tempEdgeMap);
SpanPath* solvePricing_uv(int uvLabel, LengthMap const& c, double B, double M, Digraph::Node* s, Digraph::Node* d, LengthMap& tempWeight, EdgeMap& tempEdgeMap);

void removeZeroEdgesFromGraph(BranchTreeNode* curNode);
void reAddZeroEdgesToTheGraph(BranchTreeNode* curNode);
void calcDistances();
void findBridges(list<OurEdge*>& removedEdges, list<OurEdge*>& bridges);
void setCplexParam();
void printPath(SpanPath* p);
void printRemovedEdges(BranchTreeNode* curNode);
void printFixedEdges(BranchTreeNode* curNode);
void printLPSolution(bool xFlag, bool yFlag, bool value);
void printSolution(list<OurEdge*>& sol);
void printListEdges(list<OurEdge*>& edges);
void initializeBranchTreeNode(BranchTreeNode** node);
OurEdge* getFracVarWithLowestIntGap(double& frac_val);
double getPathWLowestIntGap(list<OurEdge*>& removedEdges);
void printViolatedConstraints();
void printSpecificConstraints();
void getFixedEdges(BranchTreeNode* curNode, list<OurEdge*>& fixedEdges);
void getRemovedEdges(BranchTreeNode* curNode, list<OurEdge*>& removedEdges);
void findTupleOfBridges(BranchTreeNode* curNode, list<OurEdge*>& removedEdges, list<edge2tuple>& bridges);
void setLinearCoefForXInEdgeConstraints(OurEdge* e, int uv);
void setLinearCoefForYInEdgeConstraints(OurEdge* e, int uv, SpanPath* p);
double getDualPiForEdgeConstraint(OurEdge* e, int uv);
void startEdgeConstraints();
void removeEdgeConstraints();
void afterRemovingConstraints();
bool checkViolationEdgeConstraints();
int callCplexLPSolver(bool checkViolation);
//-----------------------------------------------

void resetTime() {
    initialClock = clock();
}

float getTime() {
  return (float) (clock()) / CLOCKS_PER_SEC;
}

float getSpentTime() {
    return (float)(clock() - initialClock) / CLOCKS_PER_SEC;
}

int getIndexInPi(OurEdge* e, int uvLabel) {
    return input.nK * e->label + uvLabel;
}

int getIndexInPathConstraints(OurEdge* e, int uvLabel) {
    return getIndexInPi(e, uvLabel);
}

int getIndexInEdgeConstraints(OurEdge* e, int uvLabel) {
    return getIndexInPi(e, uvLabel);
}

void calcDistances() {
  input.dist = new double*[input.nNodes];
    Dijkstra<Digraph, LengthMap> dijkstra(digraph, weight);

    for (int i = 0; i < input.nNodes; i++) {
      input.dist[i] = new double[input.nNodes];
    }
    for (int i = 0; i < input.nNodes; i++) {
        dijkstra.run(digraphNodes[i]);

        for (int j = (i+1); j < input.nNodes; j++) {
            input.dist[i][j] = input.dist[j][i] = dijkstra.dist(digraphNodes[j]);
        }
    }
}

void recalcDistances() {
  for (int i = 0; i < input.nNodes; i++) {
    delete input.dist[i];
  }
  delete input.dist;
  calcDistances();  
}

/*
Armazena a solução do LP (que corresponde ao RMP). Esta solução é inteira.
*/
IloNum setSolutionBasedOnRMP() {
  if (DEBUG == 17)
    cout << "dentro de setSolution" << endl;
  IloNum sum = 0;
  edgesSol.clear();
  FOR_EACH_EDGE_e
    IloNum val = masterLP.solver->getValue(masterLP.varXArray[e->label]);
    if ( val > masterLP.tol ) {
      if (DEBUG == 17)
	cout << "aresta: " << e->u << "-" << e->v << endl;
      edgesSol.push_back(e);
      if (DEBUG == 14)
	cout << "custo: " << e->cost << endl;
      sum += e->cost;
    }
  }

  if (DEBUG == 14)
    cout << "fim de setSolution, sum: " << sum << endl;
  return sum;
}

//---------------compare functions--------------------------
/*A ser utilizada na priority_queue para ordenar os lower bounds. 
A função normalmente retorna o parâmetro com menor valor. priority_queue sabe 
disso e, para que no heap tenha sempre o maior valor, inverterá os elementos 
retornados por compareLB para respeitar a característica do heap. 
Como estamos querendo um heap cujo topo tenha exatamente o elemento de menor 
valor, alteramos a forma como a função de comparação compara os elementos. 
Nesta nossa função, retornamos o maior elemento.
*/
typedef bool (*compLB)(BranchTreeNode*, BranchTreeNode*);
bool compareLB(BranchTreeNode* a, BranchTreeNode* b)
{
  return a->lower_bound > b->lower_bound; 
}
//ordena as arestas pelo custo
bool compareEdgesCost (OurEdge* e, OurEdge* f) { return (e->cost < f->cost); }
bool compareFracValMap(std::pair<OurEdge*,IloNum> a, std::pair<OurEdge*,IloNum> b) {
  if (a.second == b.second) {
    return (a.first->flag > b.first->flag);
  } else
    return (a.second > b.second);
}

bool covNodeValMap(std::pair<Digraph::Node,int> a, std::pair<Digraph::Node,int> b) {
  return (a.second > b.second);
}
//---------------------------------------------------------

//-------------------Printing functions----------------------
void printFixedVariables() {
  cout << "dentro de printFixedVariables" << endl;

  FOR_EACH_EDGE_e
    IloNum lb = masterLP.varXArray[e->label].getLB();
    IloNum ub = masterLP.varXArray[e->label].getUB();

    if (lb == ub)
      cout << e->u << "-" << e->v << " fixado em " << lb << endl;
  }  
}

void printConstraints() {
  int nRows = masterLP.solver->getNrows();
  cout << "Building constraint list ... " << nRows << flush << endl; 
  for (IloModel::Iterator it(masterLP.solver->getModel()); it.ok(); ++it)
  { 
    IloExtractable e = *it;
    // if ((e.isVariable()))
    //   cout << "variable: " << e.getName() << endl;
    // else if ((e.isConstraint()))
    //   cout << "constraint: " << e.getName() << endl;
    // else if ((e.isObjective()))
    //   cout << "objective: " << e.getName() << endl;

    if ((e.isConstraint())) {
      //cout << "cons: " << e.asConstraint() << endl;
      IloConstraint con = e.asConstraint();      
      if (con.getName() != NULL) {
	std::string str(con.getName());
	if (
	    (str.find("single_edge_cons") !=std::string::npos)  ||
	    (str.find("edge_cons") !=std::string::npos)  ||
	    (str.find("branch_one_cons") !=std::string::npos)  ||
	    (str.find("branch_zero_cons") !=std::string::npos)  ||
	    (str.find("path_cons") !=std::string::npos) ||
	    (str.find("bridge_cons") !=std::string::npos))
	  cout << "cons: " << e.asConstraint() << endl;	
      }
    } 
  }

  cout << "chegou ao fim de printConstraints" << endl;
}

void printSpecificConstraints() {
  int nRows = masterLP.solver->getNrows();
  cout << "Building specific constraint list ... " << flush << endl; 
  for (IloModel::Iterator it(masterLP.solver->getModel()); it.ok(); ++it)
  { 
    IloExtractable e = *it; 
    if ((e.isConstraint())) {
      //cout << "cons: " << e.asConstraint() << endl;
      IloConstraint con = e.asConstraint();
      if (con.getName() != NULL) {
	std::string str(con.getName());
	if (
	    // (str.find("single_edge_cons") !=std::string::npos)  ||
	    // (str.find("edge_cons") !=std::string::npos)  ||
	    (str.find("branch_one_cons") !=std::string::npos)  ||
	    (str.find("branch_zero_cons") !=std::string::npos)  ||
	    // (str.find("path_cons") !=std::string::npos) ||
	    (str.find("bridge_cons") !=std::string::npos))
	  cout << "cons: " << e.asConstraint() << endl;	
      }
    } 
  }

  cout << "chegou ao fim de printSpecificConstraints" << endl;
}


void printViolatedConstraints() {
  // Perform conflict analysis. 
  // We allow any constraint to be part of the conflict. 

  // Build a list of constraints that can be part of the conflict. 
  // Since bounds of variables may also render the model infeasible 
  // we also add bounds to the set of constraints that are considered 
  // for a conflict. 
  cout << "Building violated constraint list ... " << flush; 
  IloConstraintArray cons(masterLP.env); 
  IloNumArray prefs(masterLP.env); 
  for (IloModel::Iterator it(masterLP.solver->getModel()); it.ok(); ++it)
  {
    //cout << "vai iterar no for" << endl;
    IloExtractable e = *it; 
    if (e.isVariable()) {
      //cout << "vai analisar variavel" << endl;
      IloNumVar v = e.asVariable(); 
      if (v.getLB() > -IloInfinity) { 
	cons.add(IloBound(v, IloBound::Lower)); 
	prefs.add(1.0); 
      } 
      if (v.getUB() < IloInfinity) { 
	cons.add(IloBound(v, IloBound::Upper)); 
	prefs.add(1.0); 
      } 
    } else if (e.isConstraint()) {
      //cout << "vai analisar restricao" << endl;
      IloConstraint con = e.asConstraint();
      if (con.getName() != NULL) {
	//cout << "converteu para constraint" << endl;
	std::string str(con.getName());
	//cout << "restricao, nome: " << str << endl;
	if (
	    (str.find("single_edge_cons") !=std::string::npos)  ||
	    (str.find("edge_cons") !=std::string::npos)  ||
	    (str.find("branch_one_cons") !=std::string::npos)  ||
	    (str.find("branch_zero_cons") !=std::string::npos)  ||
	    (str.find("path_cons") !=std::string::npos) ||
	    (str.find("bridge_cons") !=std::string::npos))	  	
	  //cout << "cons: " << e.asConstraint() << endl;
	//cout << "vai adicionar a restrição" << endl;
	cons.add(e.asConstraint());
	//cout << "adicionou a restrição" << endl;
	prefs.add(1.0);
	//cout << "adicionou ao vetor prefs" << endl;	
      }
    } 
  }  
  cout << cons.getSize() << " elements." << endl; 
  // Refine the conflict. std::cout << "Refine the conflict ..." << std::endl; 
  if ( !masterLP.solver->refineConflict(cons, prefs) ) { 
    cout << "No conflict found!" << endl; 
  } else { 
    // Print out minimal conflict. 
    cout << "Conflict found. Minimal conflict is:" << endl; 
    int count = 0; 
    for (IloInt i = 0; i < cons.getSize(); ++i) { 
      if (masterLP.solver->getConflict(cons[i]) == IloCplex::ConflictMember) { 
	cout << " " << cons[i] << endl; 
	++count; 
      } 
    } 
    cout << count << " constraints in minimal conflict." << endl; 
  }   
}

void printTreePath(BranchTreeNode* node) {
  BranchTreeNode* cur = node;
  while(cur->ancestor) {
    cout << "edge: " << cur->branch_edge->u << "-" << cur->branch_edge->v << ", tipo: " << cur->type << endl;
    cur = cur->ancestor;
  }
}

void printListEdges(list<OurEdge*>& edges) {
    int i = 0;
      cout << "--------imprimindo lista de arestas ---------------" << endl;
    for (list<OurEdge*>::iterator it = edges.begin(); it != edges.end(); it++) {
	OurEdge* edge = *it;
	cout << "aresta: " << edge->u << "-" << edge->v << endl;
    }
      cout << "-------------------------" << endl;
}

void printPath(SpanPath* p) {
    cout << "-----imprimindo o caminho cujo label eh: " << p->uvLabel << "------"<< endl;
  FOR_EACH_EDGE_e_IN_PATH_p
    cout << "aresta: " << e->u << "-" << e->v << ", " << endl;
  }
  cout << "................" << endl;
}

void printSolution(list<OurEdge*>& sol) {
  for (list<OurEdge*>::iterator it = sol.begin(); it != sol.end(); it++) {
    OurEdge* e = *it;

    cout << e->u << "-" << e->v << "(" << e->cost << ")" << endl;
  }

  
}

void printRemovedEdges(BranchTreeNode* curNode) {
  cout << "printing removed Edge" << endl;
  list<OurEdge*> removedEdges = list<OurEdge*>();    
  getRemovedEdges(curNode, removedEdges);

  
  for (list<OurEdge*>::iterator it=removedEdges.begin(); it != removedEdges.end(); ++it) {
    OurEdge* edge = *it;
    cout << edge->u << "-" << edge->v << endl;
  }
}

void printFixedEdges(BranchTreeNode* curNode) {
  cout << "printing fixed Edge" << endl;
  
  list<OurEdge*> fixedEdges = list<OurEdge*>();    
  getFixedEdges(curNode, fixedEdges);
  
  
  for (list<OurEdge*>::iterator it=fixedEdges.begin(); it != fixedEdges.end(); ++it) {
    OurEdge* edge = *it;
    cout << edge->u << "-" << edge->v << endl;
  }
}


void printLPSolution(bool xFlag, bool yFlag, bool value) {
  cout << "dentro de printLPSolution" << endl;
  if (xFlag) {
    FOR_EACH_EDGE_e
      double val;
      if (MIPFlag) {
	val = lpBefMIPValues[e];
      }
      else {
	val = masterLP.solver->getValue(masterLP.varXArray[e->label]);	
      }
      
      //if ( val > masterLP.tol ) {
	cout << e->u << "-" << e->v;
	if (value)
	  cout << ":: " << val << "(" << e->cost << ")" << endl;
	else
	  cout << endl;
	//}
    }
  }

  if (yFlag) {
    FOR_EACH_ADDED_COLUMN
      SpanPath* path = *it;
	double val = masterLP.solver->getValue(path->varYArray[0]);
	  cout << "val:: " << val << endl;
	  printPath(path);
    }
  }
}

void printLabel(CSPPLabel* label) {
	  cout << "label mais barato: " << endl;
	  cout << "vertice: " << label->n->lemonNodeRef << endl;
	  printListEdges(label->p);
	  cout << "w (t*c(uv)): " << label->w << ", c: " << label->c << endl;  
}
//----------------------------------------------------------


//-------------new columns of the model-----------------------
SpanPath* newPath(int uvLabel, list<OurEdge*>& edges) {
  SpanPath* path = new SpanPath;
    path->uvLabel = uvLabel;
    path->edges = new OurEdge*[edges.size()];
    path->length = edges.size();
    
    int i = 0;
    for (list<OurEdge*>::iterator it = edges.begin(); it != edges.end(); it++) {      
      path->edges[i] = *it;     
      i++;
    }
    
    return path;
}

void addVarsXToMasterLP() {
  if (DEBUG == 19)
    cout << "dentro de addVarsXToMasterLP" << endl;
  char varName[100];
  masterLP.varXArray = IloNumVarArray(masterLP.env, input.nEdges, 0, 1, ILOFLOAT);
  // IloExpr minExpr(masterLP.env);
    FOR_EACH_EDGE_e
      masterLP.obj->setLinearCoef(masterLP.varXArray[e->label], e->cost);
      if (SET_VAR_NAMES) {
	sprintf(varName, "x.%d_%d", e->u, e->v);    
	masterLP.varXArray[e->label].setName(varName);      
      }
    }

    masterLP.model->add(masterLP.varXArray);

    FOR_EACH_EDGE_e
        for(int uv = 0; uv < input.nK; uv++) {
	  // setLinearCoefForXInEdgeConstraints(e, uv);
	  // *(EDGE_CONSTRAINTS[getIndexInEdgeConstraints(e, uv)]).setLinearCoef(masterLP.varXArray[e->label], 1.0);
        }
    }
}

void addYVarToMasterLP(SpanPath* p) {
  if (DEBUG == 26)
    cout << "dentro de addYVarToMasterLP, para adicionar o caminho cujo label eh: " << p->uvLabel << endl;
  
  char varName[100];
  strcpy (varName, "y.");
  p->varYArray = IloNumVarArray(masterLP.env, 1, 0, 1, ILOFLOAT);
  masterLP.colsAdded.at(p->uvLabel).push_back(p);
  masterLP.nAddedCols++;

  // if (p->length == 1) {
  //   OurEdge* edge = p->edges[0];
  //   edge->single_edge_path = p;
  // }

  FOR_EACH_EDGE_e_IN_PATH_p
    // masterLP.colsAddedPerEdge.at(e->label).push_back(p);
    char temp[20];
    sprintf(temp, "%d_%d.", e->u, e->v);
    strcat (varName, temp);
    if (DEBUG == 19)
      cout << "e: " << e->u << "-" << e->v << ", uv: " << p->uvLabel << ", indEdgeConst: " << getIndexInEdgeConstraints(e, p->uvLabel) << endl;
    // setLinearCoefForYInEdgeConstraints(e, p);
    // *(EDGE_CONSTRAINTS[getIndexInEdgeConstraints(e, p->uvLabel)]).setLinearCoef(p->varYArray[0] , -1.0);
  }
  if (DEBUG == 19)
    cout << "name caminho: " << varName << endl;
  if (SET_VAR_NAMES) {
    p->varYArray[0].setName(varName);
  }

  PATH_CONSTRAINTS[p->uvLabel].setLinearCoef(p->varYArray[0], 1.0);

  masterLP.model->add(p->varYArray);
  if (DEBUG == 26)
    cout << "fim de addYVarToMasterLP" << endl;
}

/*
Adiciona um mínimo de colunas Y para criar um modelo viável
*/
void buildInitialMasterLP() {
  addVarsXToMasterLP();

  Dijkstra<Digraph, LengthMap> dijkstra(digraph, weight);

  // add the y_p, where p is the minimum uv-path, for each (u,v) in K
  FOR_EACH_PAIR_uvLabel {      
    pair<int,int> pair = input.pairs[uvLabel];
    int u = pair.first;
    int v = pair.second;

    dijkstra.run(digraphNodes[u]);
    list<OurEdge*> edges = list<OurEdge*>();
    Digraph::Node nod = digraphNodes[v];

    while (dijkstra.predArc(nod) != INVALID) {
	Digraph::Arc arc = dijkstra.predArc(nod);
	edges.push_back(edgeMap[arc]);
	nod = digraph.source(arc);
    }

    SpanPath* p = newPath(uvLabel, edges);
    addYVarToMasterLP(p);
    // if (DEBUG == 26)
    //   cout << "criou um novo caminho, val: " << masterLP.solver->getValue(p->varYArray[0]);
    
  }
}
//---------------------------------------------------

void setLinearCoefForXInEdgeConstraints(OurEdge* e, int uv) {
  if (DEBUG == 26)
    cout << "setLinearCoefForXInEdgeConstraints" << endl;
  if (edgeConsFlag[getIndexInEdgeConstraints(e, uv)]) {      
  // if (EDGE_CONSTRAINTS[getIndexInEdgeConstraints(e, uv)] != NULL)
    EDGE_CONSTRAINTS[getIndexInEdgeConstraints(e, uv)][0].setLinearCoef(masterLP.varXArray[e->label], 1.0);
    if (DEBUG == 27) {
      cout << "vai adicionar coef de x na inequação num: " << getIndexInEdgeConstraints(e, uv) << endl;
      cout << "a inequação eh: " << EDGE_CONSTRAINTS[getIndexInEdgeConstraints(e, uv)][0].asConstraint() << endl;
    }
    
  }
}

void setLinearCoefForYInEdgeConstraints(OurEdge* e, int uv, SpanPath* p) {
  if (DEBUG == 26)
    cout << "setLinearCoefForYInEdgeConstraints" << endl;
  if (edgeConsFlag[getIndexInEdgeConstraints(e, uv)]) {    
  // if (EDGE_CONSTRAINTS[getIndexInEdgeConstraints(e, uv)] != NULL)
    EDGE_CONSTRAINTS[getIndexInEdgeConstraints(e, uv)][0].setLinearCoef(p->varYArray[0] , -1.0);
    if (DEBUG == 27) {
      cout << "vai adicionar coef de y na inequação num: " << getIndexInEdgeConstraints(e, p->uvLabel) << endl;
      cout << "a inequação eh: " << EDGE_CONSTRAINTS[getIndexInEdgeConstraints(e, uv)][0].asConstraint() << endl;
    }    
  }
}

double getDualPiForEdgeConstraint(OurEdge* e, int uv) {
  if (DEBUG == 26)
    cout << "getDualPiForEdgeConstraint" << endl;
  if (edgeConsFlag[getIndexInEdgeConstraints(e, uv)])
  // if (EDGE_CONSTRAINTS[getIndexInEdgeConstraints(e, uv)] != NULL)
    return masterLP.solver->getDual(EDGE_CONSTRAINTS[getIndexInEdgeConstraints(e, uv)][0]);
  else
    return 0;
}

void startEdgeConstraints() {
  edgeConsFlag = new int[input.nEdges*input.nK];
  FOR_EACH_EDGE_e
      for(int uv = 0; uv < input.nK; uv++) {
	edgeConsFlag[getIndexInEdgeConstraints(e, uv)] = 0;
	// EDGE_CONSTRAINTS[getIndexInEdgeConstraints(e, uv)] = NULL;
      }
  }  
}

void removeEdgeConstraints() {
  FOR_EACH_EDGE_e
      for(int uv = 0; uv < input.nK; uv++) {
	if (DEBUG == 27)
	  cout << "vai testar se remove inequação " << getIndexInEdgeConstraints(e, uv) << endl;
	if (edgeConsFlag[getIndexInEdgeConstraints(e, uv)]) {
	// if (EDGE_CONSTRAINTS[getIndexInEdgeConstraints(e, uv)] != NULL) {
	  //masterLP.model->remove(EDGE_CONSTRAINTS[getIndexInEdgeConstraints(e, uv)][0]);
	  EDGE_CONSTRAINTS[getIndexInEdgeConstraints(e, uv)][0].end();
	  edgeConsFlag[getIndexInEdgeConstraints(e, uv)] = 0;
	  // EDGE_CONSTRAINTS[getIndexInEdgeConstraints(e, uv)] = NULL;

	  if (DEBUG == 27)
	    cout << "removeu inequação " << getIndexInEdgeConstraints(e, uv) << ": " << endl;
	  
	}
      }
  }  
}

void afterRemovingConstraints() {
  callCplexLPSolver(false);
  if (DEBUG == 27) {
    cout << "depois de remover as restrições do modelo" << endl;
    printConstraints();
    printLPSolution(true, true, true);
  }
}

bool checkViolationEdgeConstraints() {
  //double* xVals = new double[input.nEdges];
  if (DEBUG == 27) {
    cout << "dentro de checkViolationEdgeConstraints" << endl;
    printConstraints();
    printLPSolution(true, false, true);
    //printLPSolution(true, true, true, xVals);
    //printConstraints();
  }
  bool ret = false;
  double paths_edges[input.nK][input.nEdges];
  double xe[input.nEdges];
  
    //FOR_EACH_EDGE_e
    FOR_EACH_PAIR_uvLabel {
      if (DEBUG == 26)
	cout << "uvLabel: " << uvLabel << endl;
      //double paths_edges[masterLP.colsAdded[uvLabel].size()][input.nEdges];
      
      // for (int j = 0; j < input.nEdges; j++)
      // 	paths_edges[j] = 0;
      // std::fill( &paths_edges[0], &paths_edges[0] + sizeof(paths_edges) /* / sizeofxb(flags[0][0]) */, 0 );
      // int i = 0;

      FOR_EACH_EDGE_e
	paths_edges[uvLabel][e->label] = 0;
	xe[e->label] = masterLP.solver->getValue(masterLP.varXArray[e->label]);
	if (DEBUG == 26)
	  cout << "e: " << e->u << "-" << e->v << endl;
	FOR_EACH_PATH_p_IN_PAIR_uvLabel
	  if (DEBUG == 26)
	    printPath(p);
	    // cout << "tamanho da lista de colunas de uvLabel " << uvLabel << ": " << masterLP.colsAdded[uvLabel].size() << endl;;
	  FOR_EACH_EDGE_f_IN_PATH_p
	    if (DEBUG == 26)
	      cout << "f: " << f->u << "-" << f->v << endl;
	    if (e->label == f->label) {
	      paths_edges[uvLabel][e->label] += masterLP.solver->getValue(p->varYArray[0]);
	      if (DEBUG == 26)
		cout << "valor de y para p: " << masterLP.solver->getValue(p->varYArray[0]) << endl;
	      break;
	    }
	  }
	}
//------------
      }
    }
//}


  FOR_EACH_PAIR_uvLabel {
    FOR_EACH_EDGE_e
      FOR_EACH_PATH_p_IN_PAIR_uvLabel
	// if (DEBUG == 27)
	//   printPath(p);
        FOR_EACH_EDGE_f_IN_PATH_p
	  if (e->label == f->label) {

	    if ((paths_edges[uvLabel][e->label] > xe[e->label]) &&
		(abs(paths_edges[uvLabel][e->label] - xe[e->label]) > masterLP.tol)) {
	    // if (abs(paths_edges[uvLabel][e->label] - xe[e->label]) > masterLP.tol) {
	      //if (paths_edges[uvLabel][e->label] > xe[e->label]) {//violated
	      if (DEBUG == 27) {
		int u = input.pairs[uvLabel].first;
		int v = input.pairs[uvLabel].second;
		cout << "violou a restrição para u: " << u << " v: " << v << ", e: " << e->u << "-" << e->v << endl;
		cout << "sum_paths: " << paths_edges[uvLabel][e->label] << ", xe: " << xe[e->label] << endl;
	      }
	      // masterLP.edge_constraints[getIndexInEdgeConstraints(e, uvLabel)] = IloAdd(model, IloRangeArray(env, 1, 0, IloInfinity));
	      if (DEBUG == 27)
		cout << "vai definir os coef de edge constraints, e: " << e->u << "-" << e->v << ", uvLabel: " << uvLabel << endl;
	      masterLP.edge_constraints[getIndexInEdgeConstraints(e, uvLabel)] = IloAdd(*(masterLP.model), IloRangeArray(masterLP.env, 1, 0, IloInfinity));
	      edgeConsFlag[getIndexInEdgeConstraints(e, uvLabel)] = 1;
	      if (SET_CONS_NAMES) {
		char temp[20];
		sprintf(temp, "edge_cons-%d.", getIndexInEdgeConstraints(e, uvLabel));
		// masterLP.edge_constraints[getIndexInEdgeConstraints(e, uvLabel)].setNames("edge_cons");
		masterLP.edge_constraints[getIndexInEdgeConstraints(e, uvLabel)].setNames(temp);
	      }
	      setLinearCoefForXInEdgeConstraints(e, uvLabel);
	      setLinearCoefForYInEdgeConstraints(e, uvLabel, p);
	      // FOR_EACH_PATH_p_IN_PAIR_uvLabel
	      // 	setLinearCoefForYInEdgeConstraints(e, p);
	      // }
	      ret = true;
	    }
	    
	    break;
	  }      
        }
      }

    }
  }


  return ret;
}

/*
Chama o solver para o programa linear.
Em seguida, armazena as variáveis duais.
*/
int callCplexLPSolver(bool checkViolation) {
  if (DEBUG == 26)
    cout << "dentro de callCplexLPSolver" << endl;
  MIPFlag = false;
  
  masterLP.solver->solve();

  IloAlgorithm::Status solStatus = masterLP.solver->getStatus();
  if (solStatus != IloAlgorithm::Status::Optimal) {
    if (DEBUG == 27) {
      cout << "status: " << solStatus << ", obj: " << masterLP.solver->getObjValue() << endl;
      printViolatedConstraints();
    }
    return 0;
  } else {
    if (DEBUG == 26)
      cout << "valor da func objetivo: " << masterLP.solver->getObjValue() << endl;
    //check if edge constraints are satisfied
    if (checkViolation) {
      checkViolationEdgeConstraints();
      if (DEBUG == 27)
	printConstraints();
      masterLP.solver->solve();      
    }

    // load dual variables values
    FOR_EACH_EDGE_e    
      for(int uv = 0; uv < input.nK; uv++) {
	masterLP.pi[getIndexInPi(e, uv)] = getDualPiForEdgeConstraint(e, uv);
	// masterLP.pi[getIndexInPi(e, uv)] = masterLP.solver->getDual(*(EDGE_CONSTRAINTS[getIndexInEdgeConstraints(e, uv)]));
      }
    }

    // load dual variables values
    for(int uv = 0; uv < input.nK; uv++) {
	masterLP.sigma[uv] = masterLP.solver->getDual(PATH_CONSTRAINTS[uv]);
    }
    
    if (DEBUG == 26) {
      cout << "status: optimal" << ", obj: " << masterLP.solver->getObjValue() << endl;	  
    }
  }

  return 1;
}


// -----------------------------------------------------------------------------\----------------
// ---------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------

//--------------branch tree manipulation--------------------------
/*Remove edges with zero value in the branch tree from the input graph*/
void removeZeroEdgesFromGraph(BranchTreeNode* curNode) {
  int flag = 0;

  list<OurEdge*> removedEdges = list<OurEdge*>();    
  getRemovedEdges(curNode, removedEdges);

  
  //for (list<OurEdge*>::iterator it=curNode->zero_var.begin(); it != curNode->zero_var.end(); ++it) {
  for (list<OurEdge*>::iterator it=removedEdges.begin(); it != removedEdges.end(); ++it) {

    OurEdge* edge = *it;

    edge->disabled = true;

    edgeMap.erase(edge->uv);
    edgeMap.erase(edge->vu);

    //eu deveria saber como remover elemento do map ao invés de simplesmente deixar lá
    // weight.erase(edge->uv);
    // weight.erase(edge->vu);
    
    digraph.erase(edge->uv);
    digraph.erase(edge->vu);
    
    flag = 1;
  }
}

/*(Re)add edges with zero value in the branch tree to the input graph*/
void reAddZeroEdgesToTheGraph(BranchTreeNode* curNode) {
  int flag = 0;

  list<OurEdge*> removedEdges = list<OurEdge*>();    
  getRemovedEdges(curNode, removedEdges);
  
  //for (list<OurEdge*>::iterator it=curNode->zero_var.begin(); it != curNode->zero_var.end(); ++it) {
  for (list<OurEdge*>::iterator it=removedEdges.begin(); it != removedEdges.end(); ++it) {
    OurEdge* e = *it;

    e->disabled = false;

    e->uv = digraph.addArc(digraphNodes[e->u], digraphNodes[e->v]);
    e->vu = digraph.addArc(digraphNodes[e->v], digraphNodes[e->u]);

    edgeMap[e->uv] = e;
    edgeMap[e->vu] = e;

    weight[e->uv] = e->cost;
    weight[e->vu] = e->cost;

    flag = 1;
  }
  
}

void initializeBranchTreeNode(BranchTreeNode** node) {
  (*node) = new BranchTreeNode;
  (*node)->branch_edge = NULL;
  (*node)->xe_zero = NULL;
  (*node)->xe_one = NULL;
  (*node)->zero_var = list<OurEdge*>();
  (*node)->one_var = list<OurEdge*>();
  (*node)->bridge_var = list<edge2tuple>();

  //(*node)->branch_one_constraint = NULL;
  //(*node)->branch_zero_constraint = NULL;
  //(*node)->bridge_constraint = NULL;

  (*node)->label = globalId++;
}

bool isRemovedEdge(BranchTreeNode* node, OurEdge* e) {
  list<OurEdge*> removedEdges = list<OurEdge*>();    
  getRemovedEdges(node, removedEdges);
    
  for (list<OurEdge*>::iterator it=removedEdges.begin(); (it != removedEdges.end()); it++) {
    OurEdge* remEdge = *it;

    if (remEdge->label == e->label)
      return true;
  }
  return false;
}

/*
Escolhe a variável mais fracionária, isto é, a variável cujo valor está 
mais próximo de 0.5
*/
OurEdge* getMostFracVar(BranchTreeNode* node) {
  double min = 2;
  OurEdge* var = NULL;

  FOR_EACH_EDGE_e
    double val;
    if (MIPFlag) {
      val = lpBefMIPValues[e];
    }
    else {
      val = masterLP.solver->getValue(masterLP.varXArray[e->label]);
      if (DEBUG == 10)
	cout << "aresta: " << e->u << "-" << e->v << ", val: " << val << endl;	
    }
    double diff = abs(0.5 - val);
    if (DEBUG == 7)
      cout << "diff: " << diff << endl;
    if ((floor(val) != val) && (diff < min)) {
      if (DEBUG == 7)
	cout << "encontrou uma nova variável" << endl;
      min = diff;
      var = e;
    }
  }

  return var;
}

/*
Return edge whose value in the LP is the greatest fractional value.
*/
OurEdge* getFracVarWithMaxValue(BranchTreeNode* node) {  
  if (DEBUG == 7)
    cout << "dentro de getFracVarWithMaxValue" << endl;
  if (DEBUG == 7) {
    if (MIPFlag)
      cout << "o MIPFlag está ativado" << endl;
  }
  double max = -1;
  OurEdge* var = NULL;

  FOR_EACH_EDGE_e

    double val;
    if (MIPFlag) {
      val = lpBefMIPValues[e];
      if (DEBUG == 4)
	cout << "valor da variavel fracionaria " << e->u << "-" << e->v << ": " << val << endl;      
    }
    else {
      val = masterLP.solver->getValue(masterLP.varXArray[e->label]);
      if (DEBUG == 7)
	cout << "aresta: " << e->u << "-" << e->v << ", val: " << val << endl;	
    }
    if ((floor(val) != val) && (val > max)) {
      if (DEBUG == 4)
	cout << "novo max eh : " << val << endl;
      max = val;
      var = e;
    }

  }

  return var;
}

OurEdge* getFracVarWithLowestIntGap(BranchTreeNode* node, double& frac_val) {
  if (DEBUG == 7)
    cout << "dentro de getFracVarWithLowestIntGap" << endl;
  OurEdge* edge;
  double min_int_gap = maxNumber;
  FOR_EACH_EDGE_e
    //double val = masterLP.solver->getValue(e->varX);
    double val;
    if (MIPFlag) {
      val = lpBefMIPValues[e];
    }
    else {
      //val = masterLP.solver->getValue(e->varX);
      val = masterLP.solver->getValue(masterLP.varXArray[e->label]);
    }
    if (DEBUG == 6)
      cout << "val: " << val << endl;
    double ub = masterLP.varXArray[e->label].getUB();
    if (DEBUG == 6)
      cout << "val: " << val << ", ub: " << ub << endl;
    if (( val > masterLP.tol ) && (val < ub)) {
      double int_gap = (e->cost - (e->cost * val));
      if (int_gap < min_int_gap) {
	edge = e;
	min_int_gap = int_gap;
      }
    }
  }
  frac_val = min_int_gap;
  if (min_int_gap < maxNumber) {
    //frac_val = min_val;
    return edge;
    //return min_cost - (min_cost * min_val);
  } else {    
    return NULL;
  }
}

bool pointer_is_equal(OurEdge& edge, OurEdge* p) {
  return (edge == (*p));
}

int createBranchNodes(BranchTreeNode* node, priority_queue<BranchTreeNode*, std::vector<BranchTreeNode*>, compLB>& unexploredNodes) {
  if (DEBUG == 21)
    cout << "dentro de createBranchNodes" << endl;
  OurEdge* branch_edge = getMostFracVar(node);
  if (branch_edge != NULL) {

    if (DEBUG == 21)
    cout << "variavel de branch escolhida: " << branch_edge->u << "-" << branch_edge->v << ", valor: " <<  masterLP.solver->getValue(masterLP.varXArray[branch_edge->label]) << endl;

    BranchTreeNode* zeroNode;// = (BranchTreeNode*) malloc(sizeof(BranchTreeNode));
    initializeBranchTreeNode(&zeroNode);
    BranchTreeNode* oneNode;// = (BranchTreeNode*) malloc(sizeof(BranchTreeNode));
    initializeBranchTreeNode(&oneNode);
    node->xe_zero = zeroNode;
    node->xe_one = oneNode;
    zeroNode->lower_bound = node->lower_bound;
    oneNode->lower_bound = node->lower_bound;
    oneNode->height = node->height + 1;
    zeroNode->height = node->height + 1;
    oneNode->ancestor = node;
    zeroNode->ancestor = node;

    if (DEBUG == 21)
      cout << "inicializou os nos" << endl;
    
    Digraph digraphWoZeroEdges;
    LengthMap woZeroEdgesLength(digraphWoZeroEdges);
    Digraph::NodeMap<Digraph::Node> nr(digraph);
    Digraph::NodeMap<Digraph::Node> ncr(digraph);
    Digraph::NodeMap<Digraph::Arc> ar(digraph);
    Digraph::NodeMap<Digraph::Arc> acr(digraph);

    digraphCopy(digraph, digraphWoZeroEdges).
      nodeRef(nr).
      nodeCrossRef(ncr).
      run();

    if (DEBUG == 21)
      cout << "copiou o grafo" << endl;

    //defining edge weights
    FOR_EACH_EDGE_e
      Digraph::Arc arc = e->uv;
      Digraph::Node s = digraph.source(e->uv);
      Digraph::Node t = digraph.target(e->uv);
      Digraph::Arc arcRef = findArc(digraphWoZeroEdges, nr[s], nr[t]);
      Digraph::Arc oppArcRef = findArc(digraphWoZeroEdges, nr[t], nr[s]);
      woZeroEdgesLength[arcRef] = weight[arc];
      woZeroEdgesLength[oppArcRef] = weight[e->uv];
    }

    if (DEBUG == 21)
      cout << "adicionou as arestas ao grafo temporario" << endl;


    //removing the edges that are forced to be zero
    //Also adding zero vars of the path started at the root
    list<OurEdge*> removedEdges = list<OurEdge*>();    
    getRemovedEdges(node, removedEdges);
    
    //for (list<OurEdge*>::iterator it=node->zero_var.begin(); it != node->zero_var.end(); ++it) {
    for (list<OurEdge*>::iterator it=removedEdges.begin(); it != removedEdges.end(); ++it) {
      OurEdge* e = *it;

      Digraph::Node s = digraph.source(e->uv);
      Digraph::Node t = digraph.target(e->uv);
      Digraph::Arc arc = findArc(digraphWoZeroEdges, nr[s], nr[t]);
      Digraph::Arc oppArc = findArc(digraphWoZeroEdges, nr[t], nr[s]);
      digraphWoZeroEdges.erase(arc);
      digraphWoZeroEdges.erase(oppArc);

      // list<OurEdge*>::iterator compIter = std::find (curNode->zero_var->begin(), my_list.end(), some_value);

      // list<OurEdge*>::iterator matching_iter =
      // 	std::find_if(node->zero_var.begin(), node->zero_var.end(),
      // 		     std::bind1st(pointer_is_equal, *e));


      /*
	removedEdges pode conter outras arestas alem das arestas de curNode->zero_var (as arestas fixadas em zero desde o início, ou arestas inúteis). Só vou adicionar às listas oneNode->zero_var e zeroNode->zero_var as 
arestas que pertencem a node->zero_var.
       */
      OurEdgeComparator cinst = { *e };
      list<OurEdge*>::iterator matching_iter = std::find_if(node->zero_var.begin(), node->zero_var.end(), cinst);
      
      if ( node->zero_var.end() != matching_iter ) {
	oneNode->zero_var.push_back(e);
	zeroNode->zero_var.push_back(e);	
      }      

      if (DEBUG == 21) {
	cout << "removeu o arco " << digraph.id(s) << "-" << digraph.id(t) << endl;
      }      
    }

    if (DEBUG == 21) {
      cout << "removeu do grafo temp as zero var e fez a copia para os novos vertices" << endl;
    }

    //removing the branch edge
    Digraph::Node s = digraph.source(branch_edge->uv);
    Digraph::Node t = digraph.target(branch_edge->uv);
    Digraph::Arc arc = findArc(digraphWoZeroEdges, nr[s], nr[t]);
    Digraph::Arc oppArc = findArc(digraphWoZeroEdges, nr[t], nr[s]);
    if (DEBUG == 21) {
      if (arc == INVALID)
	cout << "arco " << digraph.id(s) << "-" << digraph.id(t) << " eh invalido" << endl;
      if (oppArc == INVALID)
	cout << "arco " << digraph.id(t) << "-" << digraph.id(s) << " eh invalido" << endl;      
    }
    digraphWoZeroEdges.erase(arc);
    digraphWoZeroEdges.erase(oppArc);
    
    if (DEBUG == 21)
      cout << "acabei de remover algumas arestas do grafo temporario" << endl;

  //adding one vars of the path started at the root
    for (list<OurEdge*>::iterator it=node->one_var.begin(); it != node->one_var.end(); ++it) {
      OurEdge* e = *it;
      if (DEBUG == 21)
	cout << "edge: " << e->u << "-" << e->v << endl;
      zeroNode->one_var.push_back(e);
      oneNode->one_var.push_back(e);
    }
    if (DEBUG == 4)
      cout << "vai buscar uma aresta fracionaria" << endl;
    
    zeroNode->branch_edge = branch_edge;
    oneNode->branch_edge = branch_edge;
    zeroNode->zero_var.push_back(branch_edge);
    oneNode->one_var.push_back(branch_edge);
    zeroNode->type = 0;
    oneNode->type = 1;

    if (DEBUG == 4)
      cout << "vai buscar arcos no grafo temporario" << endl;

    unexploredNodes.push(oneNode);
    qtOneBranchNodes++;
    if (DEBUG == 21)
      cout << "criou no one, label: " << oneNode->label << endl;
    

    //checking if spanner property is violated after removing some edges        
    double len = input.dist[branch_edge->u][branch_edge->v];
    Dijkstra<Digraph, LengthMap> dijkstra(digraphWoZeroEdges, woZeroEdgesLength);
    bool reached = dijkstra.run(nr[s], nr[t]);
    if ((reached) && ((input.strFactor * len) >= dijkstra.dist(t))) {
      unexploredNodes.push(zeroNode);
      qtZeroBranchNodes++;
      if (DEBUG == 21)
	cout << "criou no zero, label: " << zeroNode->label << endl;
    }

    if (DEBUG == 21)
      cout << "fim do createBranchNodes" << endl;
    
    return 1;    
  } else {//if (branch_edge != NULL)
    if (DEBUG == 10)
      cout << "não encontrou variável fracionária" << endl;
    return 0;
  }    
}
//------------------------------------------------------------


//------------ the pricing algorithm ------------------------------
/*
Obtém a primeira solução (fracionária) para o RMP inicial
*/
IloNum getFirstSolution(/*IloEnv _env, IloModel* _model, IloCplex* _solver, IloRangeArray* _edge_constraints, IloRangeArray* _path_constraints, IloRangeArray* _bound_constraints, IloObjective* _obj*/) {
  //------------finding the bridges of the input graph-----------------------
    list<OurEdge*> emptySet = list<OurEdge*>();
    list<OurEdge*> bridgeSet = list<OurEdge*>();
    findBridges(emptySet, bridgeSet);

    for (list<OurEdge*>::iterator it=bridgeSet.begin(); it != bridgeSet.end(); it++) {
      OurEdge* edge = *it;
      edge->bridge = true;
      if (DEBUG == 27)
	cout << "a aresta " << edge->u << "-" << edge->v << " eh bridge" << endl;
      masterLP.varXArray[edge->label].setBounds(1,1);
    }
    //-------------------------------------------------------------------
    

    int lpSolvCod = 1;
    while (lpSolvCod == 1) {
      lpSolvCod = solvePricing(weight, edgeMap);
    }

    if (DEBUG == 27) {
      cout << "depois de gerar uma solução (dentro de getFirstSolution), tempo: " << getSpentTime() << endl;
      //printConstraints();
    }
    
    if (getSpentTime() >= TIME_LIMIT)
      return TIME_EXCEEDED;
    
    IloNum lb = masterLP.solver->getObjValue();
    if (DEBUG == 27) {
      cout << "depois de resolver o LP, lb: " << lb << endl;
      printConstraints();
    }

    removeEdgeConstraints();
    afterRemovingConstraints();
    return lb;
}

/*
Call the pricing algorithm until no columns are necessary to be generated in 
order to improve the solution
*/
IloNum price(BranchTreeNode* curNode) {
  if (DEBUG == 7)
    cout << "antes de callCplexLPSolver, price" << endl;

  //-----------------
  //----------------------

  // column generation for solving master LP
  removeZeroEdgesFromGraph(curNode);
  LengthMap tempWeight(digraph);
  EdgeMap tempEdgeMap;
  
  // Node2BoolMap node2BoolMap;
  // list<OurEdge*> fixedEdges = list<OurEdge*>();
  // getFixedEdges(curNode, fixedEdges);
  // for (list<OurEdge*>::iterator it=fixedEdges.begin(); it != fixedEdges.end(); it++) {
  //   OurEdge* fixEdge = *it;

  //   node2BoolMap[digraph.source(fixEdge->uv)] = 1;
  //   node2BoolMap[digraph.target(fixEdge->uv)] = 1;    
  // }    

  
  FOR_EACH_EDGE_e //iterating over the edges that are not disabled (see the MACRO)    
    // list<OurEdge*> fixedEdges = list<OurEdge*>();
    // getFixedEdges(curNode, fixedEdges);
    // int flag = 0;
    // for (list<OurEdge*>::iterator it=fixedEdges.begin(); (it != fixedEdges.end()) && !flag; it++) {
    //   OurEdge* fixEdge = *it;

    //   if (fixEdge->label == e->label) {
    // 	tempWeight[e->uv] = -maxNumber;
    // 	tempWeight[e->vu] = -maxNumber;
    // 	flag = 1;
    //   }
    // }    
    
    //definindo vetores somente para o conjunto de arestas que não estão desabilitadas
    // if (!flag) {
    //   tempWeight[e->uv] = e->cost;
    //   tempWeight[e->vu] = e->cost;      
    // }    
    tempWeight[e->uv] = e->cost;
    tempWeight[e->vu] = e->cost;  
    tempEdgeMap[e->uv] = e;
    tempEdgeMap[e->vu] = e;
  }

  int lpSolvCod = 1;
  while (lpSolvCod == 1) {
    lpSolvCod = solvePricing(tempWeight, tempEdgeMap);
  }

  reAddZeroEdgesToTheGraph(curNode);

  if (DEBUG == 12)
    cout << "fim de price" << endl;

    if (getSpentTime() >= TIME_LIMIT)
      return TIME_EXCEEDED;

  if (DEBUG == 21)
    cout << "fim de price" << endl;

  if (lpSolvCod == -1)
    return NO_LP_SOLUTION;
  else
    return masterLP.solver->getObjValue();
}

/*
Call the pricing algorithm at least once for each pair of vertices of K
*/
int solvePricing(LengthMap& tempWeight, EdgeMap& tempEdgeMap) {
  double amount_time = getTime();

  int qtColumns = 0;
  bool generatedAtLeastOneColumn;
  do {
    generatedAtLeastOneColumn = false;

    if (DEBUG == 27)
      cout << "vai chamar o callCplexLPSolver dentro de solvePricing" << endl;
    if (!callCplexLPSolver(true)) {
      // checkViolationEdgeConstraints();
      // callCplexLPSolver();
      // masterLP.solver->solve();
      
      return -1;
    }

    if (DEBUG == 27) {
      cout << "vai chamar o printConstraints dentro de solvePricing" << endl;
      printConstraints();
      printLPSolution(true, true, true);
    }

    FOR_EACH_PAIR_uvLabel {

      int u = input.pairs[uvLabel].first;
      int v = input.pairs[uvLabel].second;

      if (DEBUG == 27)
	cout << "vai chamar o solvePricing para " << u << " e " << v << endl;
      
      
      SpanPath* path = NULL;
      //do {
      if (getSpentTime() >= TIME_LIMIT)
	return false;

      LengthMap costs(digraph);

      FOR_EACH_EDGE_e
	costs[e->vu] = costs[e->uv] = masterLP.pi[getIndexInPi(e, uvLabel)];
      }

      path = solvePricing_uv(uvLabel, costs, input.strFactor * input.dist[u][v], masterLP.sigma[uvLabel], &(digraphNodes[u]), &(digraphNodes[v]), tempWeight, tempEdgeMap);

      if(path != NULL) {
	generatedAtLeastOneColumn = true;
	qtColumns++;
	addYVarToMasterLP(path);
      }	  
    }

  } while(generatedAtLeastOneColumn);

  if (DEBUG == 21)
    cout << "fim do solvePricing" << endl;

  qtPrice++;
  price_total_time+= (getTime() - amount_time);
  if (qtColumns)
    return 1;
  else
    return 0;
}

/*função para comparar os elementos do heap. No heap, sempre o elemento com
maior valor é escolhido
A função de comparação foi invertida a comparação para poder escolher sempre a 
maior
*/
typedef bool (*comp)(CSPPLabel*, CSPPLabel*);
bool compare(CSPPLabel* a, CSPPLabel* b)
{
  return a->c > b->c; //I'm interested in the cheapest path
}

// Pricing problem: Constrained Shortest Path Problem
// Algorithm: Labeling...
SpanPath* solvePricing_uv(int uvLabel, LengthMap const& c, double B, double M, Digraph::Node* s, Digraph::Node* d, LengthMap& tempWeight, EdgeMap& tempEdgeMap) {

  bool foundPath = false;
  CSPPLabel* theLabel = NULL;

  int uId = digraph.id(*s);
  int vId = digraph.id(*d);
  
  //step 1
  Dijkstra<Digraph, LengthMap> wTree(digraph, tempWeight);
  wTree.run(*d);
  Dijkstra<Digraph, LengthMap> cTree(digraph, c);
  cTree.run(*d);

  NodeMap nodeMap;
  //step 2
  CSPPNode* sCSPPNode = new CSPPNode;
  sCSPPNode->lemonNodeRef = digraph.id(*s);
  CSPPLabel* lab = new CSPPLabel;
  lab->p = list<OurEdge*>();
  lab->n = sCSPPNode;
  lab->w = 0;
  lab->c = 0;
  lab->disabled = false;
  sCSPPNode->labels = list<CSPPLabel*>();
  sCSPPNode->labels.push_back(lab);
  nodeMap[*s] = sCSPPNode;
  priority_queue<CSPPLabel*, std::vector<CSPPLabel*>, comp> heap(compare);
    
  heap.push(lab);

  while(!heap.empty()) {
    CSPPLabel* label = heap.top();
    heap.pop();

    if (label->disabled) {
      continue;
    }

    //step 3
    if (label->n->lemonNodeRef == digraph.id(*d)) {
      theLabel = label;
      break;
    }     
    //step 4
    else {
      for (int i = 0; i < input.nNodes; i++) {
	//só vale se encontrar um vizinho
	if (digraphNodes[label->n->lemonNodeRef] == digraphNodes[i]) //encontrou o proprio vertice
	  continue;
	Digraph::Arc o = findArc(digraph, digraphNodes[label->n->lemonNodeRef], digraphNodes[i]);
	if (o == INVALID)//encontrou um vertice que não é um vizinho
	  continue;
	//---------------------------
	double newLab_w = label->w + tempWeight[o];
	double newLab_c = label->c + c[o];
	     
	Digraph::Node neigh = digraphNodes[i];

	NodeMap::iterator it3 = nodeMap.find(neigh);
	CSPPNode* neighCSPPNode;

	if (it3 == nodeMap.end()) {//nao encontrou
	  neighCSPPNode = new CSPPNode;
	  neighCSPPNode->lemonNodeRef = digraph.id(digraph.target(o));
	  neighCSPPNode->labels = list<CSPPLabel*>();
	  nodeMap[neigh] = neighCSPPNode;
	} else {
	  neighCSPPNode = it3->second;
	}

	bool badNewLabel = false;

	//discarding new labels
	//if ((newLab_w + wTree.dist(neigh) > B) || (newLab_c + cTree.dist(neigh) >= M))
	if ((newLab_w + wTree.dist(neigh) - B > masterLP.tol) || (newLab_c + cTree.dist(neigh) - M > masterLP.tol)) {
	  //nao armazenar newLab
	  badNewLabel = true;
	} else {

	  //discarding dominated labels
	  for (list<CSPPLabel*>::iterator it2 = neighCSPPNode->labels.begin(); it2 != neighCSPPNode->labels.end(); ) {
	    CSPPLabel* lab = *it2;

	    //if ((newLab_w <= lab->w) && (newLab_c <= lab->c)) {
	    if ((lab->w - newLab_w >= - masterLP.tol) && (lab->c - newLab_c >= - masterLP.tol)) {                    
	      //discarding labels generated earlier that are dominated by the new labels
	      lab->disabled = true;
	      it2 = neighCSPPNode->labels.erase(it2);
	    } else {
	      //if ((newLab_w >= lab->w) && (newLab_c >= lab->c)) {
	      if ((newLab_w - lab->w >= - masterLP.tol) && (newLab_c - lab->c >= - masterLP.tol)) {
		//discarding new labels which are dominated by labels generated earlier			
		//nao armazenar newLab
		badNewLabel = true;
	      }
	      it2++;
	    }
	  }

	}
		
	if (!badNewLabel) {

	  //creating and extending a new label
	  CSPPLabel* newLab = new CSPPLabel;
	  newLab->disabled = false;

	  newLab->p = list<OurEdge*>();

	  //coping edges
	  for (list<OurEdge*>::iterator it = label->p.begin(); it != label->p.end(); it++) {
	    OurEdge* edge = *it;
	    newLab->p.push_back(edge);
	  }		  
		  
	  newLab->p.push_back(tempEdgeMap[o]);
				
	  newLab->w = newLab_w;
	  newLab->c = newLab_c;

		  
	  neighCSPPNode->labels.push_back(newLab);
		  
	  newLab->n = neighCSPPNode;		  

	  heap.push(newLab);
	}	      
	//---------------------------
		
      }
    }
    if (DEBUG == 2)
      cout << "depois de step4" << endl;	
  }

  if (DEBUG == 6)
    cout << "final da função price" << endl;

  SpanPath* thePath = NULL;
  if (theLabel != NULL) {
    thePath = newPath(uvLabel, theLabel->p);      
  }

  //deleting elements
  for (NodeMap::iterator it= nodeMap.begin(); it!=nodeMap.end(); ++it) {
    CSPPNode* node = it->second;
    for (list<CSPPLabel*>::iterator it2 = node->labels.begin(); it2 != node->labels.end(); it2++) {
      CSPPLabel* lab = *it2;
      lab->p.clear();
      delete lab;
    }
    node->labels.clear();
  }
  nodeMap.clear();
  //heap.clear();

  return thePath;
}
//----------------------------------------------------------------

//----------------Initialization----------------------------------
void initMasterLP(IloEnv _env, IloModel* _model, /*IloRangeArray* _edge_constraints,*/ IloRangeArray* _path_constraints, IloRangeArray* _bound_constraints,
                  IloObjective* _obj,IloCplex* _solver) {
    masterLP.env = _env;
    masterLP.model = _model;
    //masterLP.edge_constraints = _edge_constraints;
    masterLP.path_constraints = _path_constraints;
    masterLP.bound_constraints = _bound_constraints;
    masterLP.obj = _obj;
    masterLP.solver = _solver;
    masterLP.pi = new double[input.nEdges * input.nK];
    masterLP.sigma = new double[input.nK];
    masterLP.nAddedCols = 0;

    if (SET_CONS_NAMES) {
      //masterLP.edge_constraints->setNames("edge_cons");
      masterLP.path_constraints->setNames("path_cons");
      masterLP.obj->setName("objective");
    }
}

void initLemonAndInputVariables() {
    digraphNodes.reserve(input.nNodes);
    
    for (int i = 0; i < input.nNodes; i++) {
      digraphNodes[i] = digraph.addNode();
    }

    FOR_EACH_EDGE_e
        e->uv = digraph.addArc(digraphNodes[e->u], digraphNodes[e->v]);
        e->vu = digraph.addArc(digraphNodes[e->v], digraphNodes[e->u]);
        weight[e->uv] = e->cost;
        weight[e->vu] = e->cost;
        edgeMap[e->uv] = e;
        edgeMap[e->vu] = e;	
    }
}

void initializeEdge(OurEdge* edge, int u, int v, IloNum lab, IloNum cost) {
  edge->disabled = false;
  edge->bridge = false;
  edge->unallowed = false;
  edge->label = lab;
  edge->cost = cost;
  // edge->single_edge_path = NULL;
  edge->u = u;
  edge->v = v;
}
                                             
// ****** File format ********
// nNodes nEdges
// u v cost, for each uv in E
// nK
// u v, for each (u,v) in K
// ***************************
IloNum readInput(char *fileName, char *strFactor) {
  
  std::ifstream infile(fileName);
  if (infile.fail()) {
    return -1;
  }
  std::string line;
  getline(infile, line);
  istringstream issHead(line);

  issHead >> input.nNodes >> input.nEdges;
  input.strFactor = atof(strFactor);

  input.unit = 1;
  input.edges.reserve(input.nEdges);

  IloNum sum = 0;
    
    // read edges
    for(int i = 0; i < input.nEdges; i++) {
      IloNum cost;
      int u, v;
      getline(infile, line);
      OurEdge* edge = new OurEdge;
	istringstream iss(line);
	if (!(iss >> u >> v >> cost)) { break; }
	initializeEdge(edge, u, v, i, cost);
	sum += edge->cost;
	if (edge->cost > 1)
	  input.unit = 0;
        input.edges[edge->label] = edge;
    }

    // int ind = 0;
    input.nK = input.nEdges;
    input.pairs.reserve(input.nK);    
    FOR_EACH_EDGE_e
      input.pairs[e->label] = make_pair(e->u, e->v);
      // ind++;
    }

  return sum;
}
//-----------------------------------------------

//-------------------rounding algorithms---------------------
/*
Gera uma MST que liga as componentes induzidas pelas arestas fixadas em 1.
*/
double getMSTbasedOnFixedEdges(list<OurEdge*>& fixedEdges, list<OurEdge*>& removedEdges, list<OurEdge*>& solution) {
  
  typedef Undirector<Digraph> Adaptor;
  typedef Undirector<Digraph> Graph;
  
  IloNum sum = 0;
  
  if (DEBUG == 12)
    cout << "dentro de getMSTbasedOnFixedEdges" << endl;

  //desnecessário
  if (fixedEdges.size() == 0)
    return -1;

  for (list<OurEdge*>::iterator it=fixedEdges.begin(); it != fixedEdges.end(); it++) {
    OurEdge* fixed = *it;
    if (DEBUG == 10)
      cout << "aresta fixa: " << fixed->u << "-" << fixed->v << endl;
    solution.push_back(fixed);
    sum+= fixed->cost;
  }

  Digraph compDigraph;
  //LengthMap compLength(compDigraph);
  Digraph::NodeMap<Digraph::Node> nrComp(digraph);
  Digraph::NodeMap<Digraph::Node> ncrComp(digraph);

  digraphCopy(digraph, compDigraph).
    nodeRef(nrComp).
    nodeCrossRef(ncrComp).
    run();
  
  vector<OurEdge*> notFixedNorRemovedEdges;
  int remainEdgesQt = input.nEdges - removedEdges.size() - fixedEdges.size();
  notFixedNorRemovedEdges.reserve(remainEdgesQt);
  int j = 0;

  if (DEBUG == 12)
    cout << "remainEdgesQt: " << remainEdgesQt << endl;
  
  FOR_EACH_EDGE_e

    Digraph::Node s = digraph.source(e->uv);
    Digraph::Node t = digraph.target(e->uv);

    Digraph::Arc arcComp = findArc(compDigraph, nrComp[s], nrComp[t]);      
    Digraph::Arc arc2Comp = findArc(compDigraph, nrComp[t], nrComp[s]);
    
    if (DEBUG == 12)
      cout << "ponto 1" << endl;
    
    int fixedEdgeFlag = 0;
    for (list<OurEdge*>::iterator it=fixedEdges.begin(); (it != fixedEdges.end()) && !fixedEdgeFlag; it++) {
      OurEdge* fixed = *it;

      if (fixed->label == e->label) {
	if (DEBUG == 12)
	  cout << "fixedEdge: " << e->u << "-" << e->v << endl;
	fixedEdgeFlag = 1;	  
      }
    }

    if (DEBUG == 12)
      cout << "ponto 2" << endl;

    if (fixedEdgeFlag) {
      // compLength[arcComp] = e->cost;
      // compLength[arc2Comp] = e->cost;
      
    } else { //not fixed
      int removedEdgeFlag = 0;
      for (list<OurEdge*>::iterator it=removedEdges.begin(); (it != removedEdges.end()) && !removedEdgeFlag; it++) {
	OurEdge* remEdge = *it;

	if (remEdge->label == e->label) {

	  if (DEBUG == 12)
	    cout << "removedEdge: " << e->u << "-" << e->v << endl;
	  
	  removedEdgeFlag = 1;
	}
      }

      compDigraph.erase(arcComp);
      compDigraph.erase(arc2Comp);      

      if (DEBUG == 12)
	cout << "ponto 3" << endl;

      if (removedEdgeFlag) {
      } else { //not removed nor fixed
	notFixedNorRemovedEdges[j] = e;
	j++;	
      }
    }
    
  }

  if (DEBUG == 9)
    cout << "ponto 12" << endl;

  Undirector<Digraph> uCompDigraph(compDigraph);

  Graph::NodeMap<int> compMap(uCompDigraph);
  int nComp = connectedComponents (uCompDigraph, compMap);
  int edgesBetComp[nComp][nComp];

  int comp[input.nNodes];
  for (int i = 0; i < input.nNodes; i++)
    comp[i] = compMap[nrComp[digraphNodes[i]]];

  std::sort(notFixedNorRemovedEdges.begin(), notFixedNorRemovedEdges.end(), compareEdgesCost);

  if (DEBUG == 12)
    cout << "ponto 5, nComp: " << nComp << endl;


  for (int i = 0; i < remainEdgesQt; i++) {
    
    Undirector<Digraph> uCompDigraph(compDigraph);
    
    OurEdge* edge = notFixedNorRemovedEdges[i];

    Digraph::Node s = digraph.source(edge->uv);
    Digraph::Node t = digraph.target(edge->uv);

    if (DEBUG == 12)
      cout << "ponto 5.6" << endl;
    
    int uComp = comp[edge->u];
    int vComp = comp[edge->v];

    if (uComp != vComp) {

      if (DEBUG == 10)
	cout << "ponto 5.8" << endl;
      
      for (int j = 0; j < input.nNodes; j++) {
	if (comp[j] == uComp)
	  comp[j] = vComp;
	if (DEBUG == 10)
	  cout << "comp[" << j << "]: " << comp[j] << endl;
      }

      solution.push_back(edge);
      sum+= edge->cost;      
    }    
  }

  if (DEBUG == 12)
    cout << "fim de getMSTbasedOnFixedEdges" << endl;

  return sum;
}

//estratégia de arrendondamento do LB
IloNum roundLB(BranchTreeNode* curNode) {
  if (DEBUG == 7)
    cout << "dentro de roundLB" << endl;
  //simple strategy
  return ceil(curNode->lower_bound);

  /*more complex: what is the smallest integer greater than lb that represents a valid set of edges. 
    For a fractional path p, maybe rounding to ceil(lb) do not represents a valid set of edges, as 
    the costs of the edges are different.
    => bin packing problem.
   */  
}
//-------------------------------------------------

int checkSolution(list<OurEdge*>& sol) {
  if (DEBUG == 6)
    cout << "dentro de checkSolution" << endl;
  Digraph tempDigraph;
  LengthMap tempLength(tempDigraph);
  Digraph::NodeMap<Digraph::Node> nr(digraph);
  Digraph::NodeMap<Digraph::Node> ncr(digraph);

  digraphCopy(digraph, tempDigraph).
    nodeRef(nr).
    nodeCrossRef(ncr).
    run();

  //removing edges from the solution graph
  FOR_EACH_EDGE_e
    Digraph::Node s = digraph.source(e->uv);
    Digraph::Node t = digraph.target(e->uv);
    
    Digraph::Arc arc = findArc(tempDigraph, nr[s], nr[t]);
    tempDigraph.erase(arc);
    Digraph::Arc arc2 = findArc(tempDigraph, nr[t], nr[s]);
    tempDigraph.erase(arc2);
  }

//adding the arcs to the solution graph
  for (list<OurEdge*>::iterator it = sol.begin(); it != sol.end(); it++) {
    OurEdge* edge = *it;
    if (DEBUG == 6)
      cout << "edge: " << edge->u << "-" << edge->v << endl;

    Digraph::Node s = digraph.source(edge->uv);
    Digraph::Node t = digraph.target(edge->uv);

    Digraph::Arc arcIn = tempDigraph.addArc(nr[s], nr[t]);
    Digraph::Arc arcOut = tempDigraph.addArc(nr[t], nr[s]);
    IloNum len = edge->cost;
    tempLength[arcIn] = len;
    tempLength[arcOut] = len;
  }

//checking if, for each edge, the spanner property is valid
  FOR_EACH_EDGE_e
    Digraph::Node s = digraph.source(e->uv);
    Digraph::Node t = digraph.target(e->uv);
  
    Dijkstra<Digraph, LengthMap> dijkstra(tempDigraph, tempLength);
    bool reached = dijkstra.run(nr[s], nr[t]);
    if ((!reached) || ((input.strFactor * e->cost) < dijkstra.dist(nr[t]))) {
      if (DEBUG == 20)
	cout << "violou a restricao de span para " << e->u << " e " << e->v << endl;
      return 0;
    } else {
    }
  }

  return 1;
}

/*VER DEPOIS*/
IloNum basicPart(list<OurEdge*>& fixedEdges, int k, list<OurEdge*>& tempSol) {
  typedef std::map<Digraph::Node,int> Node2IntMap;  
  // typedef std::map<Digraph::Node,bool> Node2BoolMap;
  if (DEBUG == 20)
    cout << "dentro de basicPart" << endl;
  IloNum solVal = 0;
  typedef Digraph::NodeMap<bool> NodeFilter;
  typedef Digraph::ArcMap<bool> ArcFilter;
  typedef SubDigraph<Digraph, NodeFilter, ArcFilter> Adaptor;

  Node2BoolMap nDelMap;
  
  Digraph tempDigraph;			//represents the input graph

  Digraph::NodeMap<Digraph::Node> nr(digraph);
  Digraph::NodeMap<Digraph::Node> ncr(digraph);

  
  digraphCopy(digraph, tempDigraph).
    nodeRef(nr).
    nodeCrossRef(ncr).
    run();

  Node2IntMap node2Int;

  list<OurEdge*>::iterator edgeIt=fixedEdges.begin();      
  for (int i = 0; (i < fixedEdges.size()); i++, edgeIt++) {
    OurEdge* fixEdge = *edgeIt;
    if (DEBUG == 20)
      cout << "fixedEdge: " << fixEdge->u << "-" << fixEdge->v << endl;
    // Node2IntMap::iterator it = node2Int.find(nr[digraph.source(fixEdge->uv)]);
    // if (!it == node2Int.end())
    node2Int[nr[digraph.source(fixEdge->uv)]] = 1;
    // it = node2Int.find(nr[digraph.target(fixEdge->uv)]);
    // if (!it == node2Int.end())
    node2Int[nr[digraph.target(fixEdge->uv)]] = 1;
  }

  /*arestas que não estão fixas em 1 (arestas não fixadas e as arestas que 
    estão fixas em zero) terão o valor configurado em node2Int para zero.
*/
  for (int i = 0; i < input.nNodes; i++) {
    Node2IntMap::iterator it = node2Int.find(nr[digraphNodes[i]]);

    if (it == node2Int.end()) { //nao foi encontrado
      node2Int[nr[digraphNodes[i]]] = 0;
    }
  }  
  
  //list<OurEdge*> tempSol = list<OurEdge*>();
  list<OurEdge*> interEdges = list<OurEdge*>();

  int qtRemovedNodes = 0;
  while(qtRemovedNodes < input.nNodes) {

    /*covNodes dará prioridade aos vértices que são extremos de alguma aresta 
      fixada em 1
     */
    std::vector<std::pair<Digraph::Node,int> > covNodes(node2Int.begin(), node2Int.end());
    std::sort(covNodes.begin(), covNodes.end(), covNodeValMap);    

    /*true - vértices que já foram eliminados
      false - caso contrário
     */
    NodeFilter node_filter(tempDigraph);
    ArcFilter arc_filter(tempDigraph);
    /*
      adator terá 
     */
    Adaptor adaptor(tempDigraph, node_filter, arc_filter);
    
    for (int i = 0; i < input.nNodes; i++) {

      Node2BoolMap::iterator it = nDelMap.find(digraphNodes[i]);

      if (it == nDelMap.end()) { //nao foi removido
	node_filter[nr[digraphNodes[i]]] = false;
	//adaptor.status(nr[digraphNodes[i]], false);
      } else {//os outros vertices nao fazem parte de tempDigraph, logo não preciso me preocupar
      }
    }
    
    if (DEBUG == 20)
      cout << "dentro de while(qtRemovedNodes < input.nNodes)" << endl;
    list<Digraph::Node> S = list<Digraph::Node>();
    
    //selecting an arbitrary vertex
    // Digraph::NodeIt nIt(tempDigraph);
    // Digraph::Node s = nIt;
    Digraph::Node s = covNodes[0].first;
    adaptor.status(s, true);
    if (DEBUG == 20)
      cout << "novo centro: " << digraph.id(ncr[s]) << ", val cov: " << covNodes[0].second << endl;

    list<Digraph::Node> SNeigh = list<Digraph::Node>();

    S.push_back(s);
    
    int qtNewLevel = 0;
    list<OurEdge*> newLayerEdges = list<OurEdge*>();

    if (DEBUG == 19) {
      cout << "1/k: " << 1.0/k << endl;
      cout << "while(" << (S.size() + SNeigh.size()) << " > (pow(" << input.nNodes << ", " << (1.0/k) << ") * " << S.size() << "))" << endl;
    }
    do {
      for (list<Digraph::Node>::iterator it=SNeigh.begin(); it != SNeigh.end(); it++) {
	if (DEBUG == 20)
	  cout << "S terá adicionado o elemento: " << digraph.id(ncr[*it]) << endl;
	S.push_back(*it);
      }
      
      if (DEBUG == 19) {
	cout << "a vizinhanca ainda nao eh esparsa" << endl;
	cout << "while(SNeigh.size() > (pow(input.nNodes, (1.0/k)) * S.size()))" << endl;
	cout << "while(" << SNeigh.size() << " > (pow(" << input.nNodes << ", " << (1.0/k) << ") * " << S.size() << "))" << endl;
      }


      qtNewLevel = 0;
      newLayerEdges.clear();
      SNeigh.clear();
      for (list<Digraph::Node>::iterator it=S.begin(); it != S.end(); it++) {
	Digraph::Node itNode2 = *it;
	if (DEBUG == 20)
	  cout << "*it: " << digraph.id(ncr[itNode2]) << endl;

	  for (Digraph::OutArcIt a(tempDigraph, *it); a != INVALID; ++a) {
	    if (!adaptor.status(tempDigraph.target(a))) { //vizinho novo
	      SNeigh.push_back(tempDigraph.target(a));
	      if (DEBUG == 20) {
		cout << "tempDigraph.source(a): " <<  digraph.id(ncr[tempDigraph.source(a)]) << endl;
		cout << "mudando o status do vertice: " << digraph.id(ncr[tempDigraph.target(a)]) << endl;
	      }
	      adaptor.status(tempDigraph.target(a), true);
	      if (DEBUG == 20)
		cout << "vizinho novo de S: " << digraph.id(ncr[tempDigraph.target(a)]) << endl;

	      Digraph::Arc arc = findArc(digraph, ncr[tempDigraph.source(a)], ncr[tempDigraph.target(a)]);
	      OurEdge* e = edgeMap[arc];
	      solVal += e->cost;
	      tempSol.push_back(e);
	      newLayerEdges.push_back(e);
	      qtNewLevel++;
	    }
	  }	  	  
      }
      if (DEBUG == 20)
	cout << "fim do for" << endl;
    } while((S.size() + SNeigh.size()) > (pow(input.nNodes, (1.0/k)) * S.size()));

    //removing last layer of edges
    while (qtNewLevel--) {
      OurEdge*e = tempSol.back();
      solVal -= e->cost;
      tempSol.pop_back();
    }

    //inserting inter cluster edges into solution
    for (list<OurEdge*>::iterator it=newLayerEdges.begin(); it != newLayerEdges.end(); it++) {
      OurEdge* e1 = *it;
      bool flag = false;
      for (list<OurEdge*>::iterator it2=interEdges.begin(); (it2 != interEdges.end() && !flag); it2++) {
	OurEdge* e2 = *it2;
	if ((e1->u == e2->u) && (e1->v == e2->v)) {
	  flag = true;
	}
      }

      if (!flag) { //inserir aresta na solucao
	if (DEBUG == 20)
	  cout << "inter edge: " << e1->u << " - " << e1->v << endl;
	interEdges.push_back(e1);
	tempSol.push_back(e1);
	solVal += e1->cost;
      }
    }

    qtRemovedNodes += S.size();
    //removing S from graph
    for (list<Digraph::Node>::iterator it=S.begin(); it != S.end(); it++) {
      nDelMap[ncr[*it]] = true;
      if (DEBUG == 19)
	cout << "removeu o vertice " << digraph.id(ncr[*it]) << endl;
      node2Int.erase(*it);
      tempDigraph.erase(*it);      
    }
    if (DEBUG == 20)
      cout << "removeu os nos de S" << endl;
    
  }

  return solVal;
  
}

void getEdgeValsFromLP(vector<OurEdge*>& edges, int vecSize, int& fracSize, Edge2FracMap& edge2Frac) {
  double ub = masterLP.varXArray[0].getUB(); //ub de uma variavel associada a aresta

  double val;      
  int j = 0;
  int last2FirstInd = vecSize - 1;
  
  for (int i = 0; i < vecSize; i++) {
    OurEdge* e = edges[i];
      if (MIPFlag) {
	val = lpBefMIPValues[e];
      }
      else {
	val = masterLP.solver->getValue(masterLP.varXArray[e->label]);
      }
      
      if ( (val > masterLP.tol) && (val < ub) ) {
	edge2Frac[e] = val;
	j++;
	
      } else {
	if (val == ub)
	  edge2Frac[e] = ub ;
	else
	  edge2Frac[e] = 0;
      }
  }
  
  fracSize = j;  
}

/*
Removed edges must not be part of the set of available edges.
FixedEdges will be forced to be part of solution.
*/
IloNum greedySpanner(list<OurEdge*>& removedEdges, list<OurEdge*>& fixedEdges, list<OurEdge*>& solution) {
  if (DEBUG == 22)
    cout << "dentro de greedySpanner" << endl;

  IloNum ub = 0;
  Digraph tempDigraph;
  LengthMap tempLength(tempDigraph);
  Digraph::NodeMap<Digraph::Node> nr(digraph);
  Digraph::NodeMap<Digraph::Node> ncr(digraph);

  digraphCopy(digraph, tempDigraph).
    nodeRef(nr).
    nodeCrossRef(ncr).
    run();

  FOR_EACH_EDGE_e
    e->flag = 0;
    Digraph::Node s = digraph.source(e->uv);
    Digraph::Node t = digraph.target(e->uv);
    
    Digraph::Arc arc = findArc(tempDigraph, nr[s], nr[t]);

    if (arc == INVALID) {
      cout << "deu erro no arc" << endl;
    }
    
    tempDigraph.erase(arc);
    Digraph::Arc arc2 = findArc(tempDigraph, nr[t], nr[s]);

    if (arc2 == INVALID) {
      cout << "deu erro no arc2" << endl;
    }    
    
    tempDigraph.erase(arc2);
  }


/*fixedEdges fazem parte da solução
Estas arestas não correm o risco de fazer parte da solução duas vezes.
 */
//list<OurEdge*>::iterator it2=fixedEdges.begin(); 
  for (list<OurEdge*>::iterator it=fixedEdges.begin(); it != fixedEdges.end(); it++) {
    OurEdge* fixEdge = *it;
    fixEdge->flag = 1;
    //cout << "analisando aresta: " << remEdge->label << endl;

    Digraph::Node s = digraph.source(fixEdge->uv);
    Digraph::Node t = digraph.target(fixEdge->uv);
    Digraph::Arc arcIn = tempDigraph.addArc(nr[s], nr[t]);
    Digraph::Arc arcOut = tempDigraph.addArc(nr[t], nr[s]);
    IloNum len = fixEdge->cost;
    tempLength[arcIn] = len;
    tempLength[arcOut] = len;

    solution.push_back(fixEdge);
    ub += len;    
  }

  for (list<OurEdge*>::iterator it=removedEdges.begin(); it != removedEdges.end(); it++) {
    OurEdge* remEdge = *it;
    remEdge->flag = -1;
    //cout << "analisando aresta: " << remEdge->label << endl;
  }

    
//----------------
  vector<OurEdge*> edges;
  edges.reserve(input.nEdges);
  if (DEBUG == 12)
    cout << "input.nEdges: " << input.nEdges << ", removedEdges: " << removedEdges.size() << ", tamanho de edges: " << edges.size() << endl;
  int j = 0;

  FOR_EACH_EDGE_e
    edges[j] = input.edges[e->label];
    j++;
  }

  Edge2FracMap edge2Frac;
  int fracSize;

  getEdgeValsFromLP(edges, input.nEdges, fracSize, edge2Frac);

  //fracEdges contém todas as arestas armazenadas em edges (não somente fracionárias). Ordena de forma não-crescence.
  std::vector<std::pair<OurEdge*,IloNum> > fracEdges(edge2Frac.begin(), edge2Frac.end());
  std::sort(fracEdges.begin(), fracEdges.end(), compareFracValMap);

  for (int i = 0; i < input.nEdges; i++) {
    OurEdge* e = fracEdges[i].first;
  if (DEBUG == 22) {
    cout << "aresta de greedy: " << e->u << "-" << e->v << ", val: " <<
      masterLP.solver->getValue(masterLP.varXArray[e->label]) << ", flag: " << e->flag << endl;
      
  }
    
    if (DEBUG == 12)
      cout << "aresta que ficou: " << e-> u << "-" << e->v << endl;
    Digraph::Node s = digraph.source(e->uv);
    Digraph::Node t = digraph.target(e->uv);
    
    Dijkstra<Digraph, LengthMap> dijkstra(tempDigraph, tempLength);
    bool reached = dijkstra.run(nr[s], nr[t]);
    if ((!reached) || ((input.strFactor * e->cost) < dijkstra.dist(nr[t]))) {
      if (e->flag != -1) {
	Digraph::Arc arcIn = tempDigraph.addArc(nr[s], nr[t]);
	Digraph::Arc arcOut = tempDigraph.addArc(nr[t], nr[s]);
	IloNum len = e->cost;
	tempLength[arcIn] = len;
	tempLength[arcOut] = len;

	solution.push_back(e);
	ub += len;	
      } else {//ao remover esta aresta, nao é possível encontrar solução
	return NO_GREEDY_SOLUTION;
      }
    }    
  }

  return ub;

}

IloNum callCplexMIPSolver(IloNum& globalUB) {
  int flag = 0;
  if (DEBUG == 7)
    cout << "dentro de callCplexMIPSolver" << endl;  
  IloNum objVal;
  //eu posso remover as restrições de branch (xe == 0, xe == 1) da heurística. Como a heurística tem um novo conjunto de colunas,
  //as restrições nas variáveis não são mais importantes.
    // solve IP
  if (SAVE_VAR_BEF_MIP) {
    MIPFlag = true;
    FOR_EACH_EDGE_e
      lpBefMIPValues[e] = masterLP.solver->getValue(masterLP.varXArray[e->label]);
    }    
  }
  
  FOR_EACH_EDGE_e
    e->conv = new IloConversion(masterLP.env, masterLP.varXArray[e->label], ILOBOOL);
    masterLP.model->add(*(e->conv));
  }

  IloAlgorithm::Status solStatus = masterLP.solver->getStatus();
if (solStatus == IloAlgorithm::Status::Optimal) {
  // if (masterLP.solver->solve()) {
    objVal = (IloNum)masterLP.solver->getObjValue();
    if (objVal < globalUB) {
      globalUB = setSolutionBasedOnRMP();
      if (DEBUG == 17)
	cout << "nova solução encontrada (MIP solver): " << globalUB << endl;
    }  
  } else {
    flag = 1;
  }
    
  // re-solve the LP
  FOR_EACH_EDGE_e
    e->conv->end();
    if (DEBUG == 13)
      cout << "antes do delete 11" << endl;
      delete e->conv;
    if (DEBUG == 13)
      cout << "depois do delete 11" << endl;

  }
  callCplexLPSolver(true);

   //poderia guardar a solução ao invés de calcular a solução do LP novamente
/*Não preciso chamar o solver pois as informações que eu preciso já foram
já foram armazenadas em vetores
 */
//callCplexLPSolver();
  // if (DEBUG == 2)
  //   cout << "resolveu o modelo agora com variaveis fracionarias" << endl;

  if (flag)
    return -1;
  else
    return objVal;
}

int solveMIP(int height) {
  int flag;
  int periodQtEdges = BRANCH_HEURISTIC_MIP_INTERVAL;
  if (DEBUG == 7)
    cout << "periodQtEdges: " << periodQtEdges << ", height: " << height << endl;

  int v2 = rand() % 100 + 1; //random number between 1 and 100
  if (v2 <= BRANCH_HEURISTIC_MIP_PROB)
    flag = 1;
  else
    flag = 0;
  if (flag && (height > 0) && (height % periodQtEdges == 0))
    return 1;
  else
    return 0;
}

int solveGreedy() {
  int v2 = rand() % 100 + 1; //random number between 1 and 100
  if (v2 <= BRANCH_HEURISTIC_GREEDY_PROB)
    return 1;
  else
    return 0;  
}

void getFixedEdges(BranchTreeNode* curNode, list<OurEdge*>& fixedEdges) {

    FOR_EACH_EDGE_e
      if (e->bridge) {
	fixedEdges.push_back(e);
	if (DEBUG == 22)
	  cout << "aresta inserida em fixedEdges (bridge): " << e->u << "-" << e->v << endl;	
      }
    }
  
    //fixing edges fixed by the branch tree to be part of solution
    list<OurEdge*>::iterator oneIt=curNode->one_var.begin();
    for (int i = 0; i < curNode->one_var.size(); i++, oneIt++) {
      OurEdge* edge = *oneIt;      
      fixedEdges.push_back(edge);
      if (DEBUG == 22)
	cout << "aresta inserida em fixedEdges (one var): " << edge->u << "-" << edge->v << endl;
    }  
}

void getRemovedEdges(BranchTreeNode* curNode, list<OurEdge*>& removedEdges) {

    FOR_EACH_EDGE_e
      // IloNum lb = masterLP.varXArray[e->label].getLB();
      // IloNum ub = masterLP.varXArray[e->label].getUB();

      // if ((lb == ub) && (ub == 0))
      // 	removedEdges.push_back(e);
      if (e->unallowed)
	removedEdges.push_back(e);
    }
  
    //fixing edges fixed by the branch tree to be part of solution
    list<OurEdge*>::iterator zeroIt=curNode->zero_var.begin();
    for (int i = 0; i < curNode->zero_var.size(); i++, zeroIt++) {
      OurEdge* edge = *zeroIt;
      removedEdges.push_back(edge);
    }
}


IloNum executeBasicPart(BranchTreeNode* curNode, IloNum& globalUB) {
  list<OurEdge*> tempSol = list<OurEdge*>();
  list<OurEdge*> fixedEdges = list<OurEdge*>();
    
  getFixedEdges(curNode, fixedEdges);


    /*
      O cálcula do UB só leva em consideração as arestas com valor fixado em 
1. Sendo assim, nós não precisamos levar em consideração os nós que, ao longo 
do seu caminho na árvore de branch, tem variáveis fixadas em zero.
     */
    IloNum tempUB;
    if (curNode->zero_var.size() == 0) {
      int k;
      k = floor((input.strFactor + 1)/2);
  
      tempUB = basicPart(fixedEdges, k, tempSol);
      if (DEBUG == 21)
	cout << "basicPart UB: " << tempUB << endl;
      if (DEBUG == 20) {
	if (checkSolution(tempSol)) {	
	  cout << "solução de basicPart está ok" << endl;
	  //printSolution();
	} else
	  cout << "solução de basicPart não é spanner" << endl;      	
      }      
    }
    else
      tempUB = globalUB;
    
  if (tempUB < globalUB) {
    edgesSol.clear();
    for (list<OurEdge*>::iterator it = tempSol.begin(); it != tempSol.end(); it++) {
      OurEdge* edge = *it;
      edgesSol.push_back(edge);
    }      
  }

  return tempUB;
  
}


IloNum executeGreedyHeuristic(BranchTreeNode* curNode, IloNum& globalUB) {
    list<OurEdge*> tempSol = list<OurEdge*>();
    list<OurEdge*> fixedEdges = list<OurEdge*>();

    if (DEBUG == 22)
      cout << "dentro de executeGreedyHeuristic" << endl;
    
    getFixedEdges(curNode, fixedEdges);

    IloNum tempUB;
      tempUB = greedySpanner(curNode->zero_var, fixedEdges, tempSol); //heuristic

      if (tempUB == NO_GREEDY_SOLUTION) {
	if (DEBUG == 22)
	  cout << "solução greedy não é spanner" << endl;
	tempUB = globalUB;
      }

    if (DEBUG == 17)
      cout << "depois de greedySpanner, sol: " << tempUB << endl;    
    
    if (tempUB < globalUB) {
      edgesSol.clear();
      for (list<OurEdge*>::iterator it = tempSol.begin(); it != tempSol.end(); it++) {
	OurEdge* edge = *it;
	if (DEBUG == 17)
	  cout << edge->u << "-" << edge->v << endl;
	edgesSol.push_back(edge);
      }
    }

    if (DEBUG == 12)
      cout << "fim de executeGreedyHeuristic" << endl;

    
    return tempUB;
}

void calculateUpperBound(BranchTreeNode* curNode, IloNum& globalUB) {
  if (DEBUG == 18)
    cout << "dentro de calculateUpperBound" << endl;
  IloNum tempUB = globalUB;
  if (solveMIP(curNode->height)) {
    tempUB = callCplexMIPSolver(globalUB); //heuristic to find a feasible solution
  } else if (solveGreedy()){
    IloNum tempUB2;
    tempUB2 = executeGreedyHeuristic(curNode, globalUB);

    if (tempUB2 < globalUB) {
      if (DEBUG == 17)
	cout << "nova solução encontrada1: " << tempUB2 << endl;
      
      globalUB = tempUB2;
      masterLP.solver->setParam(IloCplex::CutUp, globalUB);
    }

    if (input.unit) {
      if (DEBUG == 18)
	cout << "vai chamar executeBasicPart " << endl;      
      tempUB = executeBasicPart(curNode, globalUB);
    } else
      tempUB = tempUB2;
    
  }
  if (DEBUG == 12)
    cout << "depois de executar heurística" << endl;
  
  if (tempUB < globalUB) {
    globalUB = tempUB;
    masterLP.solver->setParam(IloCplex::CutUp, globalUB);
  }
  if (DEBUG == 12)
    cout << "fim de calculateUpperBound" << endl;
}

int getUVLabel(OurEdge* edge) {
  FOR_EACH_PAIR_uvLabel {
    int u = input.pairs[uvLabel].first;
    int v = input.pairs[uvLabel].second;

    if (edge->u == u && edge->v == v)
      return uvLabel;
  }
  return -1;
}

int branchOnX(priority_queue<BranchTreeNode*, std::vector<BranchTreeNode*>, compLB>& unexploredNodes, IloNum& globalUB) {
    
  BranchTreeNode* curNode = unexploredNodes.top();
  if (DEBUG == 23) {
    cout << "dentro de branchOnX, label de curNode: " << curNode->label << ", height: " << curNode->height << ", lb: " << curNode->lower_bound << ", UB: " << globalUB << endl;
  }

    if (roundLB(curNode) < globalUB) {      
      //--------------------------------------

    //---------------------adding branch (and bridge) restrictions-----------
      IloRangeArray branch_one_constraint;
      if (curNode->one_var.size() > 0) {
	// IloRangeArray single_edge_path_constraint = IloAdd(*(masterLP.model), IloRangeArray(masterLP.env, 1, curNode->one_var.size()*1, curNode->one_var.size()*1)); 
	branch_one_constraint = IloAdd(*(masterLP.model), IloRangeArray(masterLP.env, 1, curNode->one_var.size()*1, curNode->one_var.size()*1));

	//variaveis so terao valor 1 (na propria inequacao ja impoem esta restricao nas variaveis)
	//curNode->branch_one_constraint = &branch_one_constraint;
	curNode->branch_one_constraint = &branch_one_constraint;

	if (SET_CONS_NAMES)
	  curNode->branch_one_constraint->setNames("branch_one_cons");

	// curNode->single_edge_path_constraint = IloAdd(*(masterLP.model), IloRangeArray(masterLP.env, 1, curNode->one_var.size()*1, curNode->one_var.size()*1)); 
	// if (SET_CONS_NAMES)
	//   curNode->single_edge_path_constraint.setNames("single_edge_cons");
      }      

      //checking what edges are bridge based on the (temporary) removed edges
      list<edge2tuple> tempBridgeSet = list<edge2tuple>();
      if (SET_FIND_TUPLE_OF_BRIDGES) {
	list<OurEdge*> removedEdges = list<OurEdge*>();
	getRemovedEdges(curNode, removedEdges);
	//findTupleOfBridges(curNode, curNode->zero_var, tempBridgeSet);
	findTupleOfBridges(curNode, removedEdges, tempBridgeSet);

	for (list<edge2tuple>::iterator tempIt=tempBridgeSet.begin(); tempIt != tempBridgeSet.end(); tempIt++) {
	  edge2tuple tup = *tempIt;
	  curNode->bridge_var.push_back(make_tuple(get<0>(tup), get<1>(tup)));
	}	  
      }
	
      tempBridgeSet.clear();

      IloRangeArray bridge_constraint;
      if (curNode->bridge_var.size() > 0) {
	bridge_constraint = IloAdd(*(masterLP.model), IloRangeArray(masterLP.env, curNode->bridge_var.size(), 1, IloInfinity));
	
      	curNode->bridge_constraint = &bridge_constraint;
	if (SET_CONS_NAMES)
	  curNode->bridge_constraint->setNames("bridge_cons");	
      }

      IloRangeArray branch_zero_constraint;
      if (curNode->zero_var.size() > 0) {	
      	//variaveis so terao valor 0 (na propria inequacao ja impoem esta restricao nas variaveis)
	branch_zero_constraint = IloAdd(*(masterLP.model), IloRangeArray(masterLP.env, 1, 0, 0));
      	curNode->branch_zero_constraint = &branch_zero_constraint;
      	if (SET_CONS_NAMES)
      	  curNode->branch_zero_constraint->setNames("branch_zero_cons");

      	//it's redundant as we have the edge constraints (when xe = 0, all paths that contain that edge have to be zero
      	//removeYpColumns(curNode);
      }

      for (list<OurEdge*>::iterator it=curNode->zero_var.begin(); it != curNode->zero_var.end(); it++) {
      	OurEdge* e = *it;
	
      	//BRANCH_ZERO_CONSTRAINT[0].setLinearCoef(e->varX, 1.0);
      	BRANCH_ZERO_CONSTRAINT[0].setLinearCoef(masterLP.varXArray[e->label], 1.0);
      }

      for (list<OurEdge*>::iterator oneIt=curNode->one_var.begin(); oneIt != curNode->one_var.end(); oneIt++) {
	OurEdge* e = *oneIt;

	// if (e->single_edge_path == NULL) {
	//   list<OurEdge*> edges = list<OurEdge*>();
	//   edges.push_back(e);
	//   SpanPath* p = newPath(getUVLabel(e), edges);
	//   e->single_edge_path = p;
	//   addYVarToMasterLP(p);
	// }
	// SINGLE_EDGE_PATH_CONSTRAINT[0].setLinearCoef(e->single_edge_path->varYArray[0], 1.0);
	
	BRANCH_ONE_CONSTRAINT[0].setLinearCoef(masterLP.varXArray[e->label], 1.0);
      }

      int i = 0;
      for (list<edge2tuple>::iterator bridgeIt=curNode->bridge_var.begin(); bridgeIt != curNode->bridge_var.end(); bridgeIt++) {
	edge2tuple tup = *bridgeIt;
      	//OurEdge* e = *bridgeIt;
	OurEdge* e = get<0>(tup);
	OurEdge* f = get<1>(tup);
      	BRIDGE_CONSTRAINT[i].setLinearCoef(masterLP.varXArray[e->label], 1.0);
	BRIDGE_CONSTRAINT[i].setLinearCoef(masterLP.varXArray[f->label], 1.0);
	i++;
      }
      /*-----------------------------------------------------*/

      //pricing     
      curNode->lower_bound = price(curNode/*, trieRoot, masks*/);
      // printLPSolution(true, true, true);
      /*NO_LP_SOLUTION: o conjunto de arestas fixadas em zero e em um impossilitam a existência de uma solução spanner válida.
       */
      if (curNode->lower_bound != NO_LP_SOLUTION) {
	//-------

	list<OurEdge*> fixedEdges = list<OurEdge*>();
	getFixedEdges(curNode, fixedEdges);
	list<OurEdge*> removedEdges = list<OurEdge*>();
	getRemovedEdges(curNode, removedEdges);	
	list<OurEdge*> tempSol = list<OurEdge*>();
	//IloNum otherLB = getMSTbasedOnFixedEdges(fixedEdges, curNode->zero_var, tempSol);
	IloNum otherLB = getMSTbasedOnFixedEdges(fixedEdges, removedEdges, tempSol);
	if (curNode->lower_bound < otherLB) {
	  curNode->lower_bound = otherLB;
	}

	if (getSpentTime() >= TIME_LIMIT)
	  return TIME_EXCEEDED;

	//o upper bound só deve ser calculado se o conjunto de arestas fixas é válido
	calculateUpperBound(curNode, globalUB);

	if (roundLB(curNode) < globalUB) {	
	  if (!createBranchNodes(curNode, unexploredNodes)) {//nao tem variavel fracionaria, logo as variaveis sao todas inteiras
	    if (DEBUG == 22)
	      cout << "não criou nós filhos" << endl;
	    if (masterLP.solver->getObjValue() < globalUB) {//so atualiza o gloal UB se o novo globalUB é menor
	      IloNum tempUB = setSolutionBasedOnRMP();
	      globalUB = tempUB;	    
	    }
	  }
	}
	
	if (DEBUG == 27) {
	  cout << "id: " << curNode->label << " anc: " << curNode->ancestor->label << " lb: " << curNode->lower_bound << " anc_lb: " << curNode->ancestor->lower_bound << " ub: " << globalUB << " altura: " << curNode->height << endl;
	  //printTreePath(curNode);
	}
	
	//else - time to prune

	//---------time to remove branch restrictions (just 2 restrictions)--------
	if (DEBUG == 20)
	  cout << "depois de createBranchNodes, hora de remover branch restrictions" << endl;

	//---------	
      } else { //NO_LP_SOLUTION
	if (DEBUG == 25) {
	  cout << "NO_LP_SOLUTION" << endl;	  
	  printFixedEdges(curNode);
	  printRemovedEdges(curNode);
	  printViolatedConstraints();
	  printConstraints();	  
	}
      }
      
      if (curNode->one_var.size() > 0) {
	// masterLP.model->remove(BRANCH_ONE_CONSTRAINT[0]);
	masterLP.model->remove(BRANCH_ONE_CONSTRAINT);
	// masterLP.model->remove(SINGLE_EDGE_PATH_CONSTRAINT[0]);
	BRANCH_ONE_CONSTRAINT.endElements();
	// SINGLE_EDGE_PATH_CONSTRAINT.endElements();
	curNode->one_var.clear();
      }

      if (curNode->bridge_var.size() > 0) {
	// for (int i = 0; i < curNode->bridge_var.size(); i++) {
	//   masterLP.model->remove(BRIDGE_CONSTRAINT[i]);
	//   //i++;
	// }
	masterLP.model->remove(BRIDGE_CONSTRAINT);
	BRIDGE_CONSTRAINT.endElements();
	curNode->bridge_var.clear();
      }      
      
      if (curNode->zero_var.size() > 0) {	
      	//masterLP.model->remove(BRANCH_ZERO_CONSTRAINT[0]);
	masterLP.model->remove(BRANCH_ZERO_CONSTRAINT);
      	BRANCH_ZERO_CONSTRAINT.endElements();
      	curNode->zero_var.clear();
      	//it's redundant as we have the edge constraints (when xe = 0, all paths that contain that edge have to be zero
      }

      removeEdgeConstraints();
      afterRemovingConstraints();
      // callCplexLPSolver();
      //-----------------------------------------------------    
    }

    //unexploredNodes.pop_front();
    //unexploredNodes.erase(unexploredNodes.begin());
    unexploredNodes.pop();

    if ((unexploredNodes.size() == 0)) {
      if (DEBUG == 23)
	cout << "esgotaram os nós" << endl;
      return BRANCH_FINISHED;
    } else {
      return BRANCH_NOT_FINISHED;
    }
}

/*
Encontra arestas que não podem fazer parte da solução final
*/
void findUnAllowedEdges() {
  if (DEBUG == 10)
    cout << "dentro de findUnAllowedEdges" << endl;
  FOR_EACH_EDGE_e
    if (e->cost > input.strFactor * input.dist[e->u][e->v]) {
      e->unallowed = true;
      masterLP.varXArray[e->label].setBounds(0,0);
    }
  }
}

/*
Find a pair of related bridges of the graph when the removedEdges set is 
eliminated from the input graph. If the remotion of two edges turns the graph 
a non t-spanner, then this pair of edges are considered related bridges.
Output: the bridge set disregarding the bridges of the original input graph 
*/
void findTupleOfBridges(BranchTreeNode* curNode, list<OurEdge*>& removedEdges, list<edge2tuple>& bridges) {

  Digraph tempDigraph;
  LengthMap tempLength(tempDigraph);
  Digraph::NodeMap<Digraph::Node> nr(digraph);
  Digraph::NodeMap<Digraph::Node> ncr(digraph);
  Digraph::NodeMap<Digraph::Arc> ar(digraph);
  Digraph::NodeMap<Digraph::Arc> acr(digraph);

  digraphCopy(digraph, tempDigraph).
    nodeRef(nr).
    nodeCrossRef(ncr).
    run();

  int remainEdgesQt = input.nEdges - removedEdges.size();
  vector<OurEdge*> edges;
  edges.reserve(remainEdgesQt);

  /*creating the temporary graph with non-zero edges (i.e., edges that were
not fixed to zero in the branch tree)
   */
  int j = 0;
  FOR_EACH_EDGE_e
    int flag = 0;
    for (list<OurEdge*>::iterator it=removedEdges.begin(); (it != removedEdges.end()) && !flag; it++) {
      OurEdge* remEdge = *it;

      if (remEdge->label == e->label)
	flag = 1;
    }

    Digraph::Arc arc = e->uv;
    Digraph::Node s = digraph.source(e->uv);
    Digraph::Node t = digraph.target(e->uv);
    Digraph::Arc arcRef = findArc(tempDigraph, nr[s], nr[t]);
    Digraph::Arc oppArcRef = findArc(tempDigraph, nr[t], nr[s]);
    
    if (!flag) {//edge is not a zero edge

      tempLength[arcRef] = weight[arc];
      tempLength[oppArcRef] = weight[e->uv];
      
      if (DEBUG == 5)
	cout << "adicionou a aresta: " << e->u << "-" << e->v << endl;
      
      edges[j] = input.edges[e->label];
      j++;
    } else {//zero edge
      tempDigraph.erase(arcRef);
      tempDigraph.erase(oppArcRef);
    }
  }
  //--------------------------------

  //iterating over the non-zero edges
  for(int i = 0; i < remainEdgesQt; i++) {

    OurEdge* e = edges[i];

    //---------
    list<OurEdge*> fixedEdges = list<OurEdge*>();
    getFixedEdges(curNode, fixedEdges);
    int flag = 0;
    for (list<OurEdge*>::iterator it=fixedEdges.begin(); (it != fixedEdges.end()) && !flag; it++) {
      OurEdge* fixEdge = *it;

      if (fixEdge->label == e->label)
	flag = 1;
    }

    if (!flag) {//we have only to consider edges that are not fixed to 1. 

      IloNum len = weight[e->uv];
      Digraph::Node s = digraph.source(e->uv);
      Digraph::Node t = digraph.target(e->uv);
      Digraph::Arc arc = findArc(tempDigraph, nr[s], nr[t]);

      if (arc == INVALID) {
	continue;
      }

      tempDigraph.erase(arc);
      Digraph::Arc arc2 = findArc(tempDigraph, nr[t], nr[s]);
      tempDigraph.erase(arc2);    

      for(int j = (i+1); j < remainEdgesQt; j++) {

	OurEdge* f = edges[j];

	int flag_f = 0;
	list<OurEdge*> fixedEdges_f = list<OurEdge*>();
	getFixedEdges(curNode, fixedEdges_f);
	
	for (list<OurEdge*>::iterator it_f=fixedEdges_f.begin(); (it_f != fixedEdges_f.end()) && !flag_f; it_f++) {
	  OurEdge* fixEdge_f = *it_f;

	  if (fixEdge_f->label == f->label)
	    flag_f = 1;
	}

	if (!flag_f) {//we have only to consider edges that are not fixed to 1.

	  IloNum len_f = weight[f->uv];
	  Digraph::Node s_f = digraph.source(f->uv);
	  Digraph::Node t_f = digraph.target(f->uv);
	  Digraph::Arc arc_f = findArc(tempDigraph, nr[s_f], nr[t_f]);

	  if (arc_f == INVALID) {
	    continue;
	  }

	  tempDigraph.erase(arc_f);
	  Digraph::Arc arc2_f = findArc(tempDigraph, nr[t_f], nr[s_f]);
	  tempDigraph.erase(arc2_f);


	  Dijkstra<Digraph, LengthMap> dijkstra(tempDigraph, tempLength);
	  bool reached = dijkstra.run(nr[s], nr[t]);
	  bool reached_f = dijkstra.run(nr[s_f], nr[t_f]);
	  if ( ((!reached) || ((input.strFactor * len) < dijkstra.dist(nr[t]))) ||
	       ((!reached_f) || ((input.strFactor * len_f) < dijkstra.dist(nr[t_f]))) ){

	    if (!e->bridge && !f->bridge) {
	      bridges.push_back(make_tuple(e, f));
	    }
	  }

	  Digraph::Arc arcIn = tempDigraph.addArc(nr[s_f], nr[t_f]);
	  Digraph::Arc arcOut = tempDigraph.addArc(nr[t_f], nr[s_f]);
	  tempLength[arcIn] = len_f;
	  tempLength[arcOut] = len_f;	  
	}	
      }

      Digraph::Arc arcIn = tempDigraph.addArc(nr[s], nr[t]);
      Digraph::Arc arcOut = tempDigraph.addArc(nr[t], nr[s]);
      tempLength[arcIn] = len;
      tempLength[arcOut] = len;
    }
        
  }
}


/*
Find bridges of the graph when the removedEdges set is eliminated from the 
input graph.
Output: the bridge set disregarding the bridges of the original input graph 
(when the removedEdges set is empty)
*/
void findBridges(list<OurEdge*>& removedEdges, list<OurEdge*>& bridges) {

  Digraph tempDigraph;
  LengthMap tempLength(tempDigraph);
  Digraph::NodeMap<Digraph::Node> nr(digraph);
  Digraph::NodeMap<Digraph::Node> ncr(digraph);
  Digraph::NodeMap<Digraph::Arc> ar(digraph);
  Digraph::NodeMap<Digraph::Arc> acr(digraph);

  digraphCopy(digraph, tempDigraph).
    nodeRef(nr).
    nodeCrossRef(ncr).
    run();

  int remainEdgesQt = input.nEdges - removedEdges.size();
  vector<OurEdge*> edges;
  edges.reserve(remainEdgesQt);

  /*creating the temporary graph with non-zero edges (i.e., edges that were
not fixed to zero in the branch tree)
   */
  int j = 0;
  FOR_EACH_EDGE_e
    int flag = 0;
    for (list<OurEdge*>::iterator it=removedEdges.begin(); (it != removedEdges.end()) && !flag; it++) {
      OurEdge* remEdge = *it;

      if (remEdge->label == e->label)
	flag = 1;
    }

    Digraph::Arc arc = e->uv;
    Digraph::Node s = digraph.source(e->uv);
    Digraph::Node t = digraph.target(e->uv);
    Digraph::Arc arcRef = findArc(tempDigraph, nr[s], nr[t]);
    Digraph::Arc oppArcRef = findArc(tempDigraph, nr[t], nr[s]);
    
    if (!flag) {//edge is not a zero edge

      tempLength[arcRef] = weight[arc];
      tempLength[oppArcRef] = weight[e->uv];
      
      if (DEBUG == 5)
	cout << "adicionou a aresta: " << e->u << "-" << e->v << endl;
      
      edges[j] = input.edges[e->label];
      j++;
    } else {//zero edge
      if (DEBUG == 7)
	cout << "dentro de findBridges, a aresta: " << e->u << "-" << e->v << " eh zero edge" << endl;
      tempDigraph.erase(arcRef);
      tempDigraph.erase(oppArcRef);
    }
  }
  //--------------------------------

  //iterating over the non-zero edges
  for(int i = 0; i < remainEdgesQt; i++) {
    OurEdge* e = edges[i];
    IloNum len = weight[e->uv];
    Digraph::Node s = digraph.source(e->uv);
    Digraph::Node t = digraph.target(e->uv);
    Digraph::Arc arc = findArc(tempDigraph, nr[s], nr[t]);


    if (arc == INVALID) {
      continue;
    }

    tempDigraph.erase(arc);
    Digraph::Arc arc2 = findArc(tempDigraph, nr[t], nr[s]);
    tempDigraph.erase(arc2);
  

    Dijkstra<Digraph, LengthMap> dijkstra(tempDigraph, tempLength);
    bool reached = dijkstra.run(nr[s], nr[t]);
    if ((!reached) || ((input.strFactor * len) < dijkstra.dist(nr[t]))) {
      
      if (!e->bridge) {
	bridges.push_back(e);
	if (DEBUG == 4)
	  cout << "aresta " << e->u << "-" << e->v << " eh ponte" << endl;
      }
    }

    Digraph::Arc arcIn = tempDigraph.addArc(nr[s], nr[t]);
    Digraph::Arc arcOut = tempDigraph.addArc(nr[t], nr[s]);
    tempLength[arcIn] = len;
    tempLength[arcOut] = len;
  }
}

void setCplexParam() {
  //eu realmente tenho que desabilitar o o pre-processamento?
  masterLP.solver->setParam(IloCplex::Param::Preprocessing::Presolve, false);
  //IloNum tol = masterLP.solver->getParam(IloCplex::EpInt);
  masterLP.tol = masterLP.solver->getParam(IloCplex::EpInt);
  //masterLP.solver->setParam(IloCplex::CutLo, masterLP.lb);
  masterLP.solver->setParam(IloCplex::TiLim, TIME_LIMIT);
  //1 - cpu time
  //2 - physical elapsed time 
  masterLP.solver->setParam(IloCplex::ClockType, 1);
  masterLP.solver->setParam(IloCplex::Threads, 1);
  //masterLP.solver->setOut("log.txt");
  //masterLP.solver->setWarning("warning.txt");
  //masterLP.solver->exportModel("CGmodel.lp");
  //masterLP.solver->setParam(IloCplex::Param::RootAlgorithm, 1);

  // masterLP.solver->setParam(IloCplex::MIPDisplay, 0);
  // masterLP.solver->setParam(IloCplex::MIPInterval, 1500);
  
  masterLP.solver->setOut(masterLP.env.getNullStream());
}

void deleteObjects(BranchTreeNode* branchRoot, IloEnv* env, IloModel* model, IloCplex* solver) {
  if (DEBUG == 13)
    cout << "dentro de deleteObjects" << endl;  
  //generated paths
  FOR_EACH_ADDED_COLUMN
    SpanPath* path = *it;
    delete path->edges;
  if (DEBUG == 13)
    cout << "delete 0.5" << endl;    
    delete path;
  }
  if (DEBUG == 13)
    cout << "delete 1" << endl;

  delete masterLP.pi;
  delete masterLP.sigma;
  if (DEBUG == 13)
    cout << "delete 2" << endl;

  //edges
  FOR_EACH_EDGE_e
    delete e;
  }
  input.edges.clear();
  if (DEBUG == 13)
    cout << "delete 3" << endl;


  //dist array
  for (int i = 0; i < input.nNodes; i++) {
    delete input.dist[i];
  }
  delete input.dist;
  if (DEBUG == 13)
    cout << "delete 4" << endl;

  //pairs (set K)
  input.pairs.clear();

  //deleting branch tree nodes
  list<BranchTreeNode*> nodes = list<BranchTreeNode*>();
  nodes.push_back(branchRoot);

  while(nodes.size() > 0) {
    BranchTreeNode* node = nodes.front();
    BranchTreeNode* left = node->xe_zero;
    BranchTreeNode* right = node->xe_one;

    node->zero_var.clear();
    node->one_var.clear();
    node->bridge_var.clear();

    if (left != NULL)
      nodes.push_back(left);
    if (right != NULL)
      nodes.push_back(right);

    nodes.erase(nodes.begin());
    delete node;
    if (DEBUG == 13)
      cout << "delete 5" << endl;    
  }
  if (DEBUG == 13)
    cout << "delete 6" << endl;

  //cplex objects
  (*solver).end();
  (*model).end();
  (*env).end();

  //edge constraints
  delete[] edgeConsFlag;

}

int main(int argc, char* argv[]) {
  double total_time;  
  IloNum globalUB;

    IloEnv env;
    IloModel model(env);
    IloCplex solver(model);

    
  ofstream timeOutFile;

  BranchTreeNode* branchRoot;

  edgesSol = list<OurEdge*>();

  resetTime();
  
  try {

    IloNum temp = readInput(argv[1], argv[2]);
    if (temp == -1)
      return -1;
    maxNumber = temp + 1;
    globalUB = maxNumber;

    masterLP.colsAdded.reserve(input.nK);
    // masterLP.colsAddedPerEdge.reserve(input.nEdges);
    // for (int i = 0; i < input.nEdges; i++)
    //   masterLP.colsAddedPerEdge.push_back(list<SpanPath*>());

    
    for (int i = 0; i < input.nK; i++)
      //masterLP.colsAdded[i] = list<SpanPath*>();
      masterLP.colsAdded.push_back(list<SpanPath*>());

    masterLP.edge_constraints.reserve(input.nK * input.nEdges);
    //masterLP.edge_constraints = new IloRange[input.nK * input.nEdges];
    startEdgeConstraints();

    if (DEBUG == 26)
      cout << "depois de readInput, tempo: " << getSpentTime() << endl;
    
    // IloRangeArray edge_constraints = IloAdd(model, IloRangeArray(env, input.nK * input.nEdges, 0, IloInfinity));
    IloRangeArray path_constraints = IloAdd(model, IloRangeArray(env, input.nK, 1, IloInfinity));
    IloRangeArray bound_constraints;
    IloObjective obj = IloAdd(model, IloMinimize(env)); //nao deveria ser IloMaximize e colocar o sinal de menos para estar na forma padrao?
    
    initLemonAndInputVariables();
    calcDistances();
    
    if (DEBUG == 26)
      cout << "depois de calcDistances, tempo: " << getSpentTime() << endl;

    //---------local lower bound-------------
    initMasterLP(env, &model, /*&edge_constraints,*/ &path_constraints, &bound_constraints, &obj, &solver);

    if (DEBUG == 26)
      cout << "depois de initMasterLP, tempo: " << getSpentTime() << endl;
    
    setCplexParam();
    if (DEBUG == 10)
      cout << "depois de initMasterLP" << endl;
    //---------Constroi o RMP, resolve o modelo-------------------
    buildInitialMasterLP();
    
    if (!input.unit) {
      findUnAllowedEdges();
    }
 
    if (DEBUG == 26) {
      cout << "antes de getFirstSolution" << endl;
      //printConstraints();
    }
    IloNum localLB = getFirstSolution(/*env, &model, &solver, &edge_constraints, &path_constraints, &bound_constraints, &obj*/);
    
    //------------------------------------------------------------
    if (localLB != TIME_EXCEEDED) {

      if (DEBUG == 26)
	cout << "depois de getFirstSolution" << endl;
      initializeBranchTreeNode(&branchRoot);
      branchRoot->height = 0;
      branchRoot->lower_bound = localLB;
      branchRoot->ancestor = NULL;
      //-----------------------------------

      //---------updating upper bound-----------
      globalUB = executeGreedyHeuristic(branchRoot, globalUB);
      if (DEBUG == 19)
	cout << "nova solução encontrada6: " << globalUB << endl;
      masterLP.solver->setParam(IloCplex::CutUp, globalUB);
      if (input.unit) {
	IloNum tempUB = executeBasicPart(branchRoot, globalUB);
	if (tempUB < globalUB) {
	  if (DEBUG == 19)
	    cout << "nova solução encontrada7: " << tempUB << endl;
	  globalUB = tempUB;
	  masterLP.solver->setParam(IloCplex::CutUp, globalUB);	
	}	
      }
    
      //---------------------------------------

      if (roundLB(branchRoot) < globalUB) {
	priority_queue<BranchTreeNode*, std::vector<BranchTreeNode*>, compLB> unexploredNodes(compareLB);

	if (DEBUG == 11)
	  cout << "antes de createBranchNodes" << endl;
	
	if (!createBranchNodes(branchRoot, unexploredNodes)) {//nao tem variavel fracionaria, logo as variaveis sao todas inteiras
	  if (DEBUG == 14)
	    cout << "antes de setSolution (main)" << endl;

	  /*
	    Apesar da solução ser inteira, algumas arestas tiveram seus 
	    valores fixados e não sabemos se necessariamente esta solução 
	    corresponde a uma solução ótima
	   */
	  if (masterLP.solver->getObjValue() < globalUB) {
	    IloNum tempUB = setSolutionBasedOnRMP();
	    globalUB = tempUB;	    
	  }

        } else {
	  while (!branchOnX(unexploredNodes, globalUB/*, &bestLBNode, trieRoot, masks*/));
	
        }

      } else {
	if (DEBUG == 10)
	  cout << "nao precisou fazer branch and bound" << endl;
      }
      
    }    

    if (getSpentTime() < TIME_LIMIT) {
      cout << "solução ótima: " << globalUB << endl;
      if (checkSolution(edgesSol)) {
	cout << "solução está ok" << endl;
	//printSolution(edgesSol);
      } else
	cout << "solução não é spanner" << endl;      
    }

  } catch (const IloException& e) {
      cerr << "Exception caught: " << e << endl;
  }
  catch (...) {
    cerr << "Unknown exception caught!" << endl;
  }


  int val = 1;
  //armazenando estatísticas
  total_time = getSpentTime();
  cout << "total time: " << total_time << endl;
  timeOutFile.open (argv[3], ios::out | ios::app);
  double qtPriceVal = (qtPrice > 0) ? (price_total_time / qtPrice) : -1;
  if (total_time >= TIME_LIMIT) {
    val = 0;
    timeOutFile << argv[4] << " " << 0 << " " << total_time << " " << qtZeroBranchNodes << " " << qtOneBranchNodes << " " << masterLP.nAddedCols << " " << qtPrice << " " << qtPriceVal << endl;
  } else {
    timeOutFile << argv[4] << " " << globalUB << " " << total_time << " " << qtZeroBranchNodes << " " << qtOneBranchNodes << " " << masterLP.nAddedCols << " " << qtPrice << " " << qtPriceVal << endl;
  }
  timeOutFile.close();

  deleteObjects(branchRoot, &env, &model, &solver);

  
  return val;
}
