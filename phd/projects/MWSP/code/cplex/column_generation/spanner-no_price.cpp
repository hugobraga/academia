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

// #include "trie_for_path.h"
// #include <bitset>
// #include <algorithm> //for std::reverse
// #include <climits> //for CHAR_BIT

namespace {
  unsigned const DEBUG = 14;

  unsigned const BRANCH_FINISHED = 1;
  unsigned const TIME_EXCEEDED = -1;
  unsigned const BRANCH_NOT_FINISHED = 0;

  double const	BRANCH_HEURISTIC_HEIGHT_PERC = 0.3;
  int const	BRANCH_HEURISTIC_MIP_PROB = 30;
  int const	BRANCH_HEURISTIC_GREEDY_PROB = 30;

  #define BRANCH_HEURISTIC_MIP_INTERVAL (int) ceil((double)input.nEdges * BRANCH_HEURISTIC_HEIGHT_PERC);
};

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

typedef bool (*compLB)(BranchTreeNode*, BranchTreeNode*);
bool compareLB(BranchTreeNode* a, BranchTreeNode* b)
{
  return a->lower_bound > b->lower_bound; 
}

//------------store lp values---------
std::map<OurEdge*, double > lpBefMIPValues;
bool MIPFlag;

//store solution
list<OurEdge*> edgesSol;
IloNum solVal;
//-----------------------------------------

//----------------Headers-----------------------
void initMasterLP(IloEnv _env, IloModel* _model, IloRangeArray* _edge_constraints, IloRangeArray* _path_constraints, IloRangeArray* _bound_constraints,
                  IloObjective* _obj,IloCplex* _solver);
void buildInitialMasterLP();
bool solvePricing(LengthMap& tempWeight, EdgeMap& tempEdgeMap);
SpanPath* solvePricing_uv(int uvLabel, LengthMap const& c, double B, double M, Digraph::Node* s, Digraph::Node* d, LengthMap& tempWeight, EdgeMap& tempEdgeMap);

void removeZeroEdgesFromGraph(BranchTreeNode* curNode);
void reAddZeroEdgesToTheGraph(BranchTreeNode* curNode);
void calcDistances();
void findBridges(list<OurEdge*>& removedEdges, list<OurEdge*>& bridges);
void setCplexParam();
void printPath(SpanPath* p);
void printLPSolution(bool xFlag, bool yFlag, bool value);
void printSolution();
void printListEdges(list<OurEdge*>& edges);
void initializeBranchTreeNode(BranchTreeNode** node);
OurEdge* getFracVarWithLowestIntGap(double& frac_val);
double getPathWLowestIntGap(list<OurEdge*>& removedEdges);
void printViolatedConstraints();
void getFixedEdges(BranchTreeNode* curNode, list<OurEdge*>& fixedEdges);
//void floydWarshall (IloNum **dist, int **next, list<OurEdge*> edgesToDel);
void floydWarshall (IloNum **dist, int **next, list<OurEdge*> edgesToDel, list<OurEdge*> fixedEdges);
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

IloNum setSolution(list<OurEdge*> &tempSol) {
  IloNum sum = 0;
  edgesSol.clear();
  for (list<OurEdge*>::iterator it=tempSol.begin(); it != tempSol.end(); it++) {
    OurEdge* e = *it;
    edgesSol.push_back(e);
    sum += e->cost;    
  }
  return sum;
}

//---------------compare functions--------------------------
bool compareEdgesCost (OurEdge* e, OurEdge* f) { return (e->cost < f->cost); }
bool compareMapObjects(std::pair<OurEdge*,int> a, std::pair<OurEdge*,int> b) {
  return (a.second < b.second);
}
//---------------------------------------------------------

//-------------------Printing functions----------------------
void printConstraints() {
  int nRows = masterLP.solver->getNrows();
  cout << "Building constraint list ... " << flush; 
  for (IloModel::Iterator it(masterLP.solver->getModel()); it.ok(); ++it)
  { 
    IloExtractable e = *it; 
    if ((e.isConstraint())) {
      IloConstraint con = e.asConstraint();
      if (con.getName() != NULL) {
	std::string str(con.getName());
	if (
	    (str.find("single_edge") !=std::string::npos)  ||
	    (str.find("branch_cons") !=std::string::npos)  ||
	    (str.find("path_cons") !=std::string::npos) ) 
	  cout << "cons: " << e.asConstraint() << endl;	
      }
    } 
  }

  cout << "chegou ao fim de printConstraints" << endl;
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
  cout << endl;
//cout << "---------------------" << endl;
}

void printSolution() {
  for (list<OurEdge*>::iterator it = edgesSol.begin(); it != edgesSol.end(); it++) {
    OurEdge* e = *it;

    cout << e->u << "-" << e->v << "(" << e->cost << ")" << endl;
  }  
}

void printLPSolution(bool xFlag, bool yFlag, bool value) {
  cout << "dentro de printSolution" << endl;
  //IloNum tol = masterLP.solver->getParam(IloCplex::EpInt);
  if (xFlag) {
    FOR_EACH_EDGE_e
      //double val = masterLP.solver->getValue(e->varX);
      double val;
      if (MIPFlag) {
	val = lpBefMIPValues[e];
      }
      else {
	//val = masterLP.solver->getValue(e->varX);
	//cout << "dual: " << masterLP.solver->getDual(PATH_CONSTRAINTS[0]) << endl;
	val = masterLP.solver->getValue(masterLP.varXArray[e->label]);
	//cout << "aresta: " << e->u << "-" << e->v << ", val: " << val << endl;
      }
      
      //double val = masterLP.solver->getValue(masterLP.varXArray[e->label]);
      if ( val > masterLP.tol ) {
	cout << e->u << "-" << e->v;
	if (value)
	  cout << ":: " << val << "(" << e->cost << ")" << endl;
	else
	  cout << endl;
      }
    }
  }

if (yFlag) {
    FOR_EACH_ADDED_COLUMN
      SpanPath* path = *it;
    //if (path->uvLabel == 0) {
	//double val = masterLP.solver->getValue(path->varY);
	double val = masterLP.solver->getValue(path->varYArray[0]);
	double val2 = masterLP.solver->getValue(path->varYArray[0]);
	//if ( val > masterLP.tol ) {
	  cout << "val:: " << val << endl;
	  printPath(path);
	  //}	
	//}
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
      
      masterLP.paths_for_edge[path->edges[i]].push_back(path);
      i++;
    }
    
    return path;
}

void addVarsXToMasterLP() {
  char varName[100];
  masterLP.varXArray = IloNumVarArray(masterLP.env, input.nEdges, 0, 1, ILOFLOAT);
  IloExpr minExpr(masterLP.env);
    FOR_EACH_EDGE_e
      masterLP.obj->setLinearCoef(masterLP.varXArray[e->label], e->cost);
      // sprintf(varName, "x.%d_%d", e->u, e->v);    
      // masterLP.varXArray[e->label].setName(varName);
    }

    masterLP.model->add(masterLP.varXArray);

    FOR_EACH_EDGE_e
        for(int uv = 0; uv < input.nK; uv++) {
	  EDGE_CONSTRAINTS[getIndexInEdgeConstraints(e, uv)].setLinearCoef(masterLP.varXArray[e->label], 1.0);
        }
    }
}

void addYVarToMasterLP(SpanPath* p) {
  char varName[100];
  strcpy (varName, "y.");
  // p->varY = IloNumVar(masterLP.env, 0, IloInfinity, ILOFLOAT);    
  // masterLP.model->add(p->varY);
  p->varYArray = IloNumVarArray(masterLP.env, 1, 0, 1, ILOFLOAT);
  //masterLP.model->add(p->varYArray);
  masterLP.colsAdded.push_back(p);
  
  if (p->length == 1) {
    OurEdge* edge = p->edges[0];
    edge->single_edge_path = p;
  }

    FOR_EACH_EDGE_e_IN_PATH_p
      char temp[20];
      sprintf(temp, "%d_%d.", e->u, e->v);
      strcat (varName, temp);
      //EDGE_CONSTRAINTS[getIndexInEdgeConstraints(e, p->uvLabel)].setLinearCoef(p->varY , -1.0);
      EDGE_CONSTRAINTS[getIndexInEdgeConstraints(e, p->uvLabel)].setLinearCoef(p->varYArray[0] , -1.0);
    }
    //p->varY.setName(varName);
    // p->varYArray[0].setName(varName);

    //PATH_CONSTRAINTS[p->uvLabel].setLinearCoef(p->varY, 1.0);
    PATH_CONSTRAINTS[p->uvLabel].setLinearCoef(p->varYArray[0], 1.0);

    masterLP.model->add(p->varYArray);
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

        addYVarToMasterLP(newPath(uvLabel, edges));
    }
}
//---------------------------------------------------

/*
Chama o solver para o programa linear.
Em seguida, armazena as variáveis duais.
*/
void callCplexLPSolver() {
  if (DEBUG == 7)
    cout << "dentro de callCplexLPSolver" << endl;
  MIPFlag = false;
  
    masterLP.solver->solve();
    
    if (DEBUG == 7) {
      cout << "status: " << masterLP.solver->getStatus() << ", obj: " << masterLP.solver->getObjValue() << endl;
    }
    // load dual variables values
    FOR_EACH_EDGE_e    
        for(int uv = 0; uv < input.nK; uv++) {
	  masterLP.pi[getIndexInPi(e, uv)] = masterLP.solver->getDual(EDGE_CONSTRAINTS[getIndexInEdgeConstraints(e, uv)]);
        }
	
    }

    // load dual variables values
    for(int uv = 0; uv < input.nK; uv++) {
        masterLP.sigma[uv] = masterLP.solver->getDual(PATH_CONSTRAINTS[uv]);
    }
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
  //int count = 0;
  for (list<OurEdge*>::iterator it=curNode->zero_var.begin(); it != curNode->zero_var.end(); ++it) {
    //count++;
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

  //input.nEdges -= count;
  // if (flag)
  //   recalcDistances();
}

/*(Re)add edges with zero value in the branch tree to the input graph*/
void reAddZeroEdgesToTheGraph(BranchTreeNode* curNode) {
  int flag = 0;
  
  for (list<OurEdge*>::iterator it=curNode->zero_var.begin(); it != curNode->zero_var.end(); ++it) {
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
  
  // if (flag)
  //   recalcDistances();
}

void initializeBranchTreeNode(BranchTreeNode** node) {
  //node = (BranchTreeNode*) malloc(sizeof(BranchTreeNode));  
  *node = new BranchTreeNode;
  (*node)->branch_edge = NULL;
  (*node)->xe_zero = NULL;
  (*node)->xe_one = NULL;
  (*node)->zero_var = list<OurEdge*>();
  (*node)->one_var = list<OurEdge*>();
  (*node)->bridge_var = list<OurEdge*>();
  //(*node)->height = 0;

  (*node)->branch_one_constraint = NULL;
  (*node)->branch_zero_constraint = NULL;
  (*node)->bridge_constraint = NULL;

  (*node)->label = globalId++;
}

/*
Escolhe a variável mais fracionária, isto é, a variável cujo valor está 
mais próximo de 0.5
*/
OurEdge* getMostFracVar(BranchTreeNode* node) {
  double min = 2;
  OurEdge* var = NULL;

  // list<OurEdge*> fixedEdges = list<OurEdge*>();
  // getFixedEdges(node, fixedEdges);
  FOR_EACH_EDGE_e
    // int flag = 0;
    // for (list<OurEdge*>::iterator it=fixedEdges.begin(); (it != fixedEdges.end()) && !flag; it++) {
    //   OurEdge* briEdge = *it;

    //   if (briEdge->label == e->label)
    // 	flag = 1;
    // }

    // if (!flag) {//a aresta nao está fixa em 1

    double val;
    if (MIPFlag) {
      val = lpBefMIPValues[e];
      if (DEBUG == 4)
	cout << "valor da variavel fracionaria " << e->u << "-" << e->v << ": " << val << endl;      
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

    // }
    //--------
            
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

  // list<OurEdge*> fixedEdges = list<OurEdge*>();
  // getFixedEdges(node, fixedEdges);
  FOR_EACH_EDGE_e
    // int flag = 0;
    // for (list<OurEdge*>::iterator it=fixedEdges.begin(); (it != fixedEdges.end()) && !flag; it++) {
    //   OurEdge* briEdge = *it;

    //   if (briEdge->label == e->label)
    // 	flag = 1;
    // }

    // if (!flag) {//a aresta nao está fixa em 1

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

    // }
    //--------
  }

  if (DEBUG == 7) {
    if (var == NULL)
      cout << "nao encontrou aresta fracionaria" << endl;
    else
      cout << "aresta frac: " << var->u << "-" << var->v << ", val: " << max << endl;
  }
  return var;
}

bool checkSpannerProperty(list<OurEdge*> edgesToDel, OurEdge* edge) {
  if (DEBUG == 13)
    cout << "dentro de checkSpannerProperty" << endl;
  Digraph tempDigraph;
  LengthMap tempLength(tempDigraph);
  Digraph::NodeMap<Digraph::Node> nr(digraph);
  Digraph::NodeMap<Digraph::Node> ncr(digraph);

  digraphCopy(digraph, tempDigraph).
    nodeRef(nr).
    nodeCrossRef(ncr).
    run();

  FOR_EACH_EDGE_e

    Digraph::Node s = digraph.source(e->uv);
    Digraph::Node t = digraph.target(e->uv);
    Digraph::Arc arc = findArc(tempDigraph, nr[s], nr[t]);      
    Digraph::Arc arc2 = findArc(tempDigraph, nr[t], nr[s]);    
    
    tempLength[arc] = e->cost;
    tempLength[arc2] = e->cost;
  }
  
  for (list<OurEdge*>::iterator it=edgesToDel.begin(); it != edgesToDel.end(); ++it) {
    OurEdge* e = *it;

    Digraph::Node s = digraph.source(e->uv);
    Digraph::Node t = digraph.target(e->uv);
    Digraph::Arc arc = findArc(tempDigraph, nr[s], nr[t]);      
    Digraph::Arc arc2 = findArc(tempDigraph, nr[t], nr[s]);
    
    tempDigraph.erase(arc);
    tempDigraph.erase(arc2);
  }

if (DEBUG == 13)
  cout << "apagou as arestas do grafo temporário" << endl;

    Digraph::Node s = digraph.source(edge->uv);
    Digraph::Node t = digraph.target(edge->uv);
//Digraph::Arc arc = findArc(tempDigraph, nr[s], nr[t]);

    IloNum len = input.dist[edge->u][edge->v];
    Dijkstra<Digraph, LengthMap> dijkstra(tempDigraph, tempLength);
bool reached = dijkstra.run(nr[s], nr[t]);
    if ((!reached) || ((input.strFactor * len) < dijkstra.dist(nr[t]))) {
      if (DEBUG == 13) 
	cout << "violou a span property para o par " << edge->u << "-" << edge->v << endl;      
      return false;
    } else {
      if (DEBUG == 13)
	cout << "nao violou a span property para o par " << edge->u << "-" << edge->v << endl;      
      return true;
    }
}

//acho que tenho que remover edgesToDel antes de calcular floyWarshall
int findBranchEdge(list<OurEdge*>& edgesToDel, list<OurEdge*>& fixedEdges, OurEdge** branchEdge) {
  if (DEBUG == 13)
    cout << "dentro de findBranchEdge" << endl;
  OurEdge* chosenEdge = NULL;
  list<OurEdge*> emptySet = list<OurEdge*>();

  Edge2IntMap edge2Int;
  // vector<OurEdge*> usedEdges;
  // usedEdges.reserve(input.nEdges - edgesToDel.size());
  
  //-------------
  
  Digraph tempDigraph;
  LengthMap tempLength(tempDigraph);
  Digraph::NodeMap<Digraph::Node> nr(digraph);
  Digraph::NodeMap<Digraph::Node> ncr(digraph);

  digraphCopy(digraph, tempDigraph).
    nodeRef(nr).
    nodeCrossRef(ncr).
    run();

  FOR_EACH_EDGE_e
    Digraph::Node s = digraph.source(e->uv);
    Digraph::Node t = digraph.target(e->uv);
    Digraph::Arc arc = findArc(tempDigraph, nr[s], nr[t]);
    Digraph::Arc arc2 = findArc(tempDigraph, nr[t], nr[s]);

    tempLength[arc] = e->cost;
    tempLength[arc2] = e->cost;
  }
  
  for (list<OurEdge*>::iterator it=edgesToDel.begin(); it != edgesToDel.end(); ++it) {
    OurEdge* e = *it;

    if (DEBUG == 13)
      cout << "vai apagar a aresta " << e->u << "-" << e->v << endl;
    
    Digraph::Node s = digraph.source(e->uv);
    Digraph::Node t = digraph.target(e->uv);
    Digraph::Arc arc = findArc(tempDigraph, nr[s], nr[t]);      
    Digraph::Arc arc2 = findArc(tempDigraph, nr[t], nr[s]);
    
    tempDigraph.erase(arc);
    tempDigraph.erase(arc2);
  }

  for (list<OurEdge*>::iterator it=fixedEdges.begin(); it != fixedEdges.end(); ++it) {
    OurEdge* e = *it;

    if (DEBUG == 13)
      cout << "vai fixar a aresta " << e->u << "-" << e->v << endl;    

    Digraph::Node s = digraph.source(e->uv);
    Digraph::Node t = digraph.target(e->uv);
    Digraph::Arc arc = findArc(tempDigraph, nr[s], nr[t]);
    Digraph::Arc arc2 = findArc(tempDigraph, nr[t], nr[s]);

    tempLength[arc] = 0;
    tempLength[arc2] = 0;
  }

  FOR_EACH_EDGE_e
        int u = e->u;
	int v = e->v;
	Digraph::Node nodu = nr[digraphNodes[u]];
//dijkstra.run(nodu);
//list<OurEdge*> edges = list<OurEdge*>();
        Digraph::Node nod = nr[digraphNodes[v]];

	Dijkstra<Digraph, LengthMap> dijkstra(tempDigraph, tempLength);
	bool reached = dijkstra.run(nodu, nod);


	if (reached) {
	  if (dijkstra.dist(nod) > 0) {

	    IloNum amountDist = 0;

	    while (dijkstra.predArc(nod) != INVALID) {
	      Digraph::Arc arc = dijkstra.predArc(nod);
	      //amountDist += templength[arc];
	      if (DEBUG == 13)
		cout << "entre o no " << e->u << " e " << e->v; 

	      if (tempLength[arc] > 0) {

		Digraph::Arc origArc = findArc(digraph, ncr[tempDigraph.source(arc)], ncr[tempDigraph.target(arc)]);
		
		
		OurEdge* edge = edgeMap[origArc];

		if (DEBUG == 13)
		  cout << " esta o arco " << edge->u << " e " << edge->v << endl; 		
		Edge2IntMap::iterator it;
		it = edge2Int.find(edge);

		if (it == edge2Int.end()) {
		  if (DEBUG == 13)
		    cout << "aresta inserida pela primeira vez no vetor" << endl;
		  edge2Int[edge] = 0;
		} else {
		  if (DEBUG == 13)
		    cout << "aresta inserida de novo no vetor" << endl;	
		  edge2Int[edge]++;
		}

		
	      }
	      
	      nod = tempDigraph.source(arc);
	    }  
	    
	    
	  }
	} else {
	  if (DEBUG == 13)
	    cout << "nao achou caminho entre os vertices: " << e->u << " e " << e->v << endl; 
	  *branchEdge = NULL;
	  return 0;	  
	}        
  }

//--------------------

  

//----------daqui

  if (edge2Int.size() == 0) {

    FOR_EACH_EDGE_e

      Edge2IntMap::iterator mapIt;
      mapIt = edge2Int.find(e);
      if (mapIt == edge2Int.end()) {

	int flag = 0;
	for (list<OurEdge*>::iterator it=edgesToDel.begin(); (it != edgesToDel.end()) && !flag; it++) {
	  OurEdge* remEdge = *it;
	  //cout << "analisando aresta: " << remEdge->label << endl;

	  if (remEdge->label == e->label) {
	    if (DEBUG == 6)
	      cout << "aresta removida: " << remEdge->u << "-" << remEdge->v << endl;	
	    flag = 1;
	  }
	}

	int fixedFlag = 0;
	if (!flag) {//if the edge is in removedEdges, it cannot be in fixedEdges
	  list<OurEdge*>::iterator it2=fixedEdges.begin();      
	  for (int k = 0; (k < fixedEdges.size()) && !fixedFlag; k++, it2++) {
	    OurEdge* fixEdge = *it2;
	    //cout << "analisando aresta: " << remEdge->label << endl;

	    if (fixEdge->label == e->label) {
	      if (DEBUG == 13)
		cout << "aresta fixa: " << fixEdge->u << "-" << fixEdge->v << endl;

	      fixedFlag = 1;
	    }
	  }      
	}

	if (!flag && !fixedFlag) {
	  edge2Int[e] = 0;
	}

      }    
    }

    
  }

//----------ate aqui

  std::vector<std::pair<OurEdge*,int> > usedEdges(edge2Int.begin(), edge2Int.end());

  std::sort(usedEdges.begin(), usedEdges.end(), compareMapObjects);


//   Edge2IntMap::iterator it;
//   FOR_EACH_EDGE_e

//     it = edge2Int.find(e);
// if (it != edge2Int.end()) {//I will only consider edges that are not in edgesToDel set
//       if ((chosenEdge == NULL) || (edge2Int[e] < edge2Int[chosenEdge]))
// 	chosenEdge = e;
//     }
//   }

//   if (chosenEdge != NULL) {
//     if (DEBUG == 13)
//       cout << "escolheu aresta: " << chosenEdge->u << "-" << chosenEdge->v << endl;
//     edgesToDel.push_back(chosenEdge);
//   }

  bool spanViolated = true;

  int i = 0;

  // while(spanViolated && (i < usedEdges.size())) {
    
  //   spanViolated = !checkSpannerProperty(edgesToDel, usedEdges[i].first);
  //   if (spanViolated)
  //     i++;  
  // }

if (usedEdges.size() == 0) {
  if (DEBUG == 13)
    cout << "usedEdges.size() == 0" << endl;
  *branchEdge = NULL;
} else {
  *branchEdge = usedEdges[i].first;
  if (DEBUG == 13)
    cout << "achou uma aresta de brach, e esta aresta eh: " << (*branchEdge)->u << "-" << (*branchEdge)->v << endl;  
  edgesToDel.push_back(*branchEdge);
  spanViolated = !checkSpannerProperty(edgesToDel, *branchEdge);
}

  if (spanViolated) {
    //*branchEdge = NULL;
    return 0;
  } else {
    //*branchEdge = usedEdges[i].first;
    return 1;
  }

// do {
  
//   *branchEdge = chosenEdge;
//   OurEdge* edge = usedEdges[i].first;

//   spanViolated = !checkSpannerProperty(edgesToDel, chosenEdge);
//   if (checkSpannerProperty(edgesToDel, chosenEdge)) {
//     if (DEBUG == 13)
//       cout << "spanner property foi respeitada" << endl;
//     return 1;
//   } else
//     return 0;
  
//   i++;
// } while (!spanViolated);


}

int createBranchNodes(BranchTreeNode* node, priority_queue<BranchTreeNode*, std::vector<BranchTreeNode*>, compLB>& unexploredNodes) {
  if (DEBUG == 13)
    cout << "dentro de createBranchNodes" << endl;
  
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

  list<OurEdge*> edgesToDel = list<OurEdge*>();

  //Also adding zero vars of the path started at the root
  for (list<OurEdge*>::iterator it=node->zero_var.begin(); it != node->zero_var.end(); ++it) {
    OurEdge* e = *it;

    if (DEBUG == 13)
      cout << "edge: " << e->u << "-" << e->v << ", fixada em 0 está sendo copiada para zeroNode e oneNode" << endl;
    
    edgesToDel.push_back(e);
      
    oneNode->zero_var.push_back(e);
    zeroNode->zero_var.push_back(e);      
  }


  //adding one vars of the path started at the root
  for (list<OurEdge*>::iterator it=node->one_var.begin(); it != node->one_var.end(); ++it) {
    OurEdge* e = *it;
    if (DEBUG == 13)
      cout << "edge: " << e->u << "-" << e->v << ", fixada em 1 está sendo copiada para zeroNode e oneNode" << endl;
    zeroNode->one_var.push_back(e);
    oneNode->one_var.push_back(e);
  }
  if (DEBUG == 4)
    cout << "vai buscar uma aresta fracionaria" << endl;


  OurEdge* branch_edge;
  list<OurEdge*> fixedEdges = list<OurEdge*>();
  getFixedEdges(node, fixedEdges);
  int spanProperty = findBranchEdge(edgesToDel, fixedEdges, &branch_edge);
  if (branch_edge != NULL) {

    zeroNode->branch_edge = branch_edge;
    oneNode->branch_edge = branch_edge;
    zeroNode->zero_var.push_back(branch_edge);
    oneNode->one_var.push_back(branch_edge);
    if (DEBUG == 13)
      cout << "inserido a branch_edge " << branch_edge->u << "-" << branch_edge->v << " como aresta 1" << endl;
    zeroNode->type = 0;
    oneNode->type = 1;

    
    //unexploredNodes.push_back(oneNode);
    unexploredNodes.push(oneNode);
    qtOneBranchNodes++;
    if (DEBUG == 13)
      cout << "criou um nó 1, label: " << oneNode->label << endl;

    if (spanProperty) {
      unexploredNodes.push(zeroNode);
      if (DEBUG == 13)
	cout << "criou um nó 0, label: " << zeroNode->label << endl;
      
      qtZeroBranchNodes++;
    }
    return 1;
  } else
    return 0;          
}
//------------------------------------------------------------


//------------ the pricing algorithm ------------------------------
/*
Obtém a primeira solução (fracionária) para o RMP inicial
*/
IloNum getFirstSolution(IloEnv _env, IloModel* _model, IloCplex* _solver, IloRangeArray* _edge_constraints, IloRangeArray* _path_constraints, IloRangeArray* _bound_constraints, IloObjective* _obj) {
  //resetTime();

  if (DEBUG == 11)
    cout << "dentro de getFirstSolution" << endl;
    
    // --- initialize master LP info ---
    // IloEnv _env;
    // IloModel _model(_env);
    // IloCplex _solver(_model);    
    // IloRangeArray _edge_constraints = IloAdd(*(_model), IloRangeArray(_env, input.nK * input.nEdges, 0, IloInfinity));
    // IloRangeArray _path_constraints = IloAdd(*(_model), IloRangeArray(_env, input.nK, 1, IloInfinity));
    // IloObjective _obj = IloAdd(*(_model), IloMinimize(_env)); //nao deveria ser IloMaximize e colocar o sinal de menos para estar na forma padrao?

  /*
    _edge_constraints->setNames("edge constraint");
    _path_constraints->setNames("path constraint");
  */
    
    // list<SpanPath*> _colsAdded;
//    _solver.setOut(_env.getNullStream());
//    _solver.setWarning(_env.getNullStream());
//    _solver.setParam(IloCplex::TiLim, TIME_LIMIT);
                                             
    //_solver.setParam(IloCplex::Param::Preprocessing::Presolve, false);
    //initMasterLP(&_env, &_model, &_edge_constraints, &_path_constraints, &_obj, &_solver/*, &_colsAdded*/);

    /*
    initMasterLP(_env, _model, _edge_constraints, _path_constraints, _bound_constraints, _obj, _solver);    
    buildInitialMasterLP();

    setCplexParam();
    */

  //------------finding the bridges of the input graph-----------------------
    list<OurEdge*> emptySet = list<OurEdge*>();
    list<OurEdge*> bridgeSet = list<OurEdge*>();
    findBridges(emptySet, bridgeSet);

    for (list<OurEdge*>::iterator it=bridgeSet.begin(); it != bridgeSet.end(); it++) {
      OurEdge* edge = *it;
      edge->bridge = true;
      if (DEBUG == 7)
	cout << "a aresta " << edge->u << "-" << edge->v << " eh bridge" << endl;
      masterLP.varXArray[edge->label].setBounds(1,1);
    }
    //-------------------------------------------------------------------
    

    if (DEBUG == 7)
      cout << "antes de callCplexLPSolver, getFirstSoxlution" << endl;    
    //callCplexLPSolver();
    if (DEBUG == 7) {
      //printSolution(true, false, true);
      cout << "depois de callCplexLPSolver, getFirstSolution" << endl;
    }

    //printViolatedConstraints();
    // column generation for solving (reduced) master LP
    while (solvePricing(weight, edgeMap));

    if (DEBUG == 12)
      cout << "depois de gerar uma solução (dentro de getFirstSolution), tempo: " << getSpentTime() << endl;
    
    if (getSpentTime() >= TIME_LIMIT)
      return TIME_EXCEEDED;
    
    IloNum lb = masterLP.solver->getObjValue();
    if (DEBUG == 12)
      cout << "depois de resolver o LP, lb: " << lb << endl;
    
    return lb;
}

/*
Call the pricing algorithm until no columns are necessary to be generated in 
order to improve the solution
*/
IloNum price(BranchTreeNode* curNode) {
    if (DEBUG == 7)
      cout << "antes de callCplexLPSolver, price" << endl;
    //testar se é viável
    //callCplexLPSolver();
    
    if (DEBUG == 7) {
      //printSolution(true, false, true);
      cout << "depois de callCplexLPSolver, price" << endl;
    }
    
    // column generation for solving master LP
    removeZeroEdgesFromGraph(curNode);
    LengthMap tempWeight(digraph);
    EdgeMap tempEdgeMap;
    FOR_EACH_EDGE_e //iterating over the edges that are not disabled (see the MACRO)
        tempWeight[e->uv] = e->cost;
        tempWeight[e->vu] = e->cost;
        tempEdgeMap[e->uv] = e;
        tempEdgeMap[e->vu] = e;	
    }
    
    while (solvePricing(tempWeight, tempEdgeMap));
    reAddZeroEdgesToTheGraph(curNode);

  if (DEBUG == 12)
    cout << "fim de price" << endl;

    if (getSpentTime() >= TIME_LIMIT)
      return TIME_EXCEEDED;

  if (DEBUG == 12)
    cout << "fim de price" << endl;

  return masterLP.solver->getObjValue();
}

/*
Call the pricing algorithm at least once for each pair of vertices of K
*/
bool solvePricing(LengthMap& tempWeight, EdgeMap& tempEdgeMap) {
  double amount_time = getTime();
  //IloNum qtColumns = 0;

  
    bool generatedAtLeastOneColumn = false;
    if (DEBUG == 11)
      cout << "dentro de solvePricing" << endl;
    
    FOR_EACH_PAIR_uvLabel {

      /*
      if (getSpentTime() >= TIME_LIMIT)
	return false;
      */
      
        int u = input.pairs[uvLabel].first;
        int v = input.pairs[uvLabel].second;

	/*
        LengthMap costs(digraph);

        FOR_EACH_EDGE_e
	  costs[e->vu] = costs[e->uv] = masterLP.pi[getIndexInPi(e, uvLabel)];
        }
	*/
    
	if (DEBUG == 6) {
	  cout << "vai chamar solvePricing_uv para " << u << " e " << v << endl;
	}

	SpanPath* path = NULL;
	do {
	  if (getSpentTime() >= TIME_LIMIT)
	    return false;

	  if (DEBUG == 12)
	    cout << "antes de callCplexLPSolver" << endl;	  
	  callCplexLPSolver();
	  if (DEBUG == 12)
	    cout << "depois de callCplexLPSolver" << endl;
	  
	  LengthMap costs(digraph);

	  FOR_EACH_EDGE_e
	    costs[e->vu] = costs[e->uv] = masterLP.pi[getIndexInPi(e, uvLabel)];
          }

	  
	  path = solvePricing_uv(uvLabel, costs, input.strFactor * input.dist[u][v], masterLP.sigma[uvLabel], &(digraphNodes[u]), &(digraphNodes[v]), tempWeight, tempEdgeMap);

	  if(path != NULL) {
	    //generatedAtLeastOneColumn = true;
	    addYVarToMasterLP(path);
	    if (DEBUG == 12)
	      cout << "adicionou variável Y ao RMP" << endl;
	    //qtColumns++;
	  }	  
	} while (path != NULL);

	// if (path != NULL) {

	//   if ((DEBUG == 9) && (path->length == 1)) {
	//     OurEdge* edge = path->edges[0];
	//     if (edge->u == 2 && edge->v == 5)
	//       cout << "price gerou um novo caminho de uma aresta entre 2 e 5" << endl;
	//   }
	  
	//   if (DEBUG == 8) {
	//     cout << "depois de adicionar um caminho entre " << u << "-" << v << endl;
	//     printPath(path);
	//   }
	// } else {
	//     if (DEBUG == 6)
	//       cout << "path eh nulo para o par: " << u << "-" << v << endl;
	// }


    /*
        if(path != NULL) {
	  generatedAtLeastOneColumn = true;
	  addYVarToMasterLP(path);
        }
    */
    }

    /*
      É necessário chamar o solver novamente para (caso) o reduced cost 
continue zero e seja necessário executar o pricing novamente. Neste caso, nós 
teremos que atualizar os parâmetros do pricing com os novos valores das 
variáveis duais.
    */
/*
    if (generatedAtLeastOneColumn) {
      // if (DEBUG == 7)
      // 	cout << "antes de callCplexLPSolver, generateAtLeastOneCOlumn (solvePricing)" << endl;  
      callCplexLPSolver();
      // if (DEBUG == 7) {
      // 	//printSolution(true, false, true);
      // 	cout << "depois de callCplexLPSolver, generateAtLeastOneColumn" << endl;
      // }
    }

    if (DEBUG == 11)
      cout << "fim de solvePricing, tempo: " << getSpentTime() << endl;


    if (DEBUG == 6)
      cout << "fim de solvePricing" << endl;
*/

  qtPrice++;
  price_total_time+= (getTime() - amount_time);
    return generatedAtLeastOneColumn;
}

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
  if ((DEBUG == 6)) {
    cout << "dentro de solvePricing_uv, s: " << digraph.id(*s) << ", d: " << digraph.id(*d) << endl;
  }
  
  //step 1
  Dijkstra<Digraph, LengthMap> wTree(digraph, tempWeight);
  if ((DEBUG == 3) && (uId == 4 && vId == 13))    
    cout << "instanciou a  wTree" << endl;            
  wTree.run(*d);
  if ((DEBUG == 3) && (uId == 4 && vId == 13))    
    cout << "calculou a wTree" << endl;        
  Dijkstra<Digraph, LengthMap> cTree(digraph, c);
  cTree.run(*d);

  if ((DEBUG == 3) && (uId == 4 && vId == 13))    
    cout << "depois de step1" << endl;    

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
	if (digraphNodes[label->n->lemonNodeRef] == digraphNodes[i])
	  continue;
	Digraph::Arc o = findArc(digraph, digraphNodes[label->n->lemonNodeRef], digraphNodes[i]);
	if (o == INVALID)
	  continue;
	//---------------------------
	double newLab_w = label->w + tempWeight[o];
	double newLab_c = label->c + c[o];
	     
	// Digraph::Node neigh = digraphNodes[digraph.id(digraph.target(o))];
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
	if ((newLab_w + wTree.dist(neigh) - B > masterLP.tol) || (newLab_c + cTree.dist(neigh) - M > - masterLP.tol)) {
	  //nao armazenar newLab
	  badNewLabel = true;
	} else {

	  //discarding dominated labels
	  for (list<CSPPLabel*>::iterator it2 = neighCSPPNode->labels.begin(); it2 != neighCSPPNode->labels.end(); ) {
	    CSPPLabel* lab = *it2;

	    //if ((newLab_w <= lab->w) && (newLab_c <= lab->c)) {
	    if ((lab->w - newLab_w > - masterLP.tol) && (lab->c - newLab_c > - masterLP.tol)) {                    
	      //discarding labels generated earlier that are dominated by the new labels
	      lab->disabled = true;
	      it2 = neighCSPPNode->labels.erase(it2);
	    } else {
	      //if ((newLab_w >= lab->w) && (newLab_c >= lab->c)) {
	      if ((newLab_w - lab->w > - masterLP.tol) && (newLab_c - lab->c > - masterLP.tol)) {
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
      if (DEBUG == 12)
	cout << "antes do delete 8" << endl;      
      delete lab;
      if (DEBUG == 12)
	cout << "depois do delete 8" << endl;      
    }
    if (DEBUG == 12)
      cout << "antes do delete 9" << endl;          
    node->labels.clear();
    if (DEBUG == 12)
      cout << "depois do delete 9" << endl;    
    //delete node;
  }
  nodeMap.clear();
  if (DEBUG == 12)
    cout << "depois do delete 10" << endl;  
  //heap.clear();

  return thePath;
}
//----------------------------------------------------------------

//----------------Initialization--------------------------------                                             
void initMasterLP(IloEnv _env, IloModel* _model, IloRangeArray* _edge_constraints, IloRangeArray* _path_constraints, IloRangeArray* _bound_constraints,
                  IloObjective* _obj,IloCplex* _solver) {
    masterLP.env = _env;
    masterLP.model = _model;
    masterLP.edge_constraints = _edge_constraints;
    masterLP.path_constraints = _path_constraints;
    masterLP.bound_constraints = _bound_constraints;
    masterLP.obj = _obj;
    masterLP.solver = _solver;
    masterLP.pi = new double[input.nEdges * input.nK];
    masterLP.sigma = new double[input.nK];

    if (DEBUG == 11)
      cout << "antes de dar nome as restrições, tempo: " << getSpentTime() << endl;
    
    // masterLP.edge_constraints->setNames("edge constraint");
    // masterLP.path_constraints->setNames("path_cons");

    if (DEBUG == 11)
      cout << "depois de dar nome as restrições, tempo: " << getSpentTime() << endl;
    
    // masterLP.colsAdded = _colsAdded;
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
  edge->label = lab;
  edge->cost = cost;
  edge->single_edge_path = NULL;
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
	//input.edges.insert(edge);
    }

    int ind = 0;
    input.nK = input.nEdges;
    input.pairs.reserve(input.nK);    
    FOR_EACH_EDGE_e
      input.pairs[ind] = make_pair(e->u, e->v);
      ind++;
    }

  return sum;
}
//-----------------------------------------------

/*
void calcDistances(Digraph* digraph, LengthMap& weight, int nNodes, vector<Digraph::Node>& digraphNodes) {
    double** dist = (double**) malloc(nNodes * sizeof(double*));
    Dijkstra<Digraph, LengthMap> dijkstra(digraph, weight);
   
    for (int i = 0; i < nNodes; i++) {
        dist[i] = (double*) malloc(input.nNodes * sizeof(double));
        dijkstra.run(digraphNodes[i]);

        for (int j = (i+1); j < nNodes; j++) {
            dist[i][j] = dist[j][i] = dijkstra.dist(digraphNodes[j]);
        }
    }    
}
*/

//está errado. Veja o caso do caminho entre 0 e 5
// Solves the all-pairs shortest path problem using Floyd Warshall algorithm
void floydWarshall (IloNum **dist, int **next, list<OurEdge*> edgesToDel, list<OurEdge*> fixedEdges) {
  if (DEBUG == 13)
    cout << "dentro de floydWarshall" << endl;
  
  /* dist[][] will be the output matrix that will finally have the shortest 
     distances between every pair of vertices */
  // IloNum dist[input.nNodes][input.nNodes];
  // Digraph::Node* next[input.nNodes][input.nNodes];
  int i, j, k;
 
  /* Initialize the solution matrix same as input graph matrix. Or 
     we can say the initial values of shortest distances are based
     on shortest paths considering no intermediate vertex. */
  for (i = 0; i < input.nNodes; i++) {
    for (j = (i+1); j < input.nNodes; j++) {
      dist[i][j] = maxNumber;
      dist[j][i] = maxNumber;

      next[i][j] = -1;
      next[j][i] = -1;
    }
    dist[i][i] = 0;
    
    next[i][i] = -1;
  }

  //
  FOR_EACH_EDGE_e
    dist[e->u][e->v] = e->cost;
    next[e->u][e->v] = e->v;

    dist[e->v][e->u] = e->cost;
    next[e->v][e->u] = e->u;    
  }

//removing edges
  for (list<OurEdge*>::iterator it=edgesToDel.begin(); it != edgesToDel.end(); ++it) {
    OurEdge* e = *it;

    if (DEBUG == 13)
      cout << "aresta fixa em 0: " << e->u << "-" << e->v << endl;

    dist[e->u][e->v] = maxNumber;
    next[e->u][e->v] = -1;

    dist[e->v][e->u] = maxNumber;
    next[e->v][e->u] = -1;    
  }  

//fixing edges
  for (list<OurEdge*>::iterator it=fixedEdges.begin(); it != fixedEdges.end(); ++it) {
    OurEdge* e = *it;

    if (DEBUG == 13)
      cout << "aresta fixa em 1: " << e->u << "-" << e->v << endl;
    
    dist[e->u][e->v] = 0;
    next[e->u][e->v] = e->v;

    dist[e->v][e->u] = 0;
    next[e->v][e->u] = e->u;        
  }


  /* Add all vertices one by one to the set of intermediate vertices.
     ---> Before start of a iteration, we have shortest distances between all
     pairs of vertices such that the shortest distances consider only the
     vertices in set {0, 1, 2, .. k-1} as intermediate vertices.
     ----> After the end of a iteration, vertex no. k is added to the set of
     intermediate vertices and the set becomes {0, 1, 2, .. k} */
  for (k = 0; k < input.nNodes; k++)
    {
      // Pick all vertices as source one by one
      for (i = 0; i < input.nNodes; i++)
        {
	  // Pick all vertices as destination for the
	  // above picked source
	  for (j = 0; j < input.nNodes; j++)
            {
	      // If vertex k is on the shortest path from
	      // i to j, then update the value of dist[i][j]
	      if (DEBUG == 12) {
		cout << "dist[" << i << "][" << k << "] (" << dist[i][k] << ") + dist[" << k << "][" << j << "] (" << dist[k][j] << ") < dist[" << i << "][" << j << "] ( " << dist[i][j] << ")" << endl;
	      }
	      //if (dist[k][i] + dist[k][j] < dist[i][j]) {
	      if (dist[i][k] + dist[k][j] < dist[i][j]) {
		dist[i][j] = dist[i][k] + dist[k][j];
		//dist[i][j] = dist[k][i] + dist[k][j];
		next[i][j] = next[i][k];
		//next[i][j] = next[k][i];
	      }
            }
        }
    }
}

//-------------------rounding algorithms---------------------
double getMSTbasedOnFixedEdges(list<OurEdge*>& fixedEdges, list<OurEdge*>& removedEdges, list<OurEdge*>& solution) {
  
  typedef Undirector<Digraph> Adaptor;
  typedef Undirector<Digraph> Graph;
  
  IloNum sum = 0;
  
  if (DEBUG == 13)
    cout << "dentro de getMSTbasedOnFixedEdges" << endl;
  
  // if (fixedEdges.size() == 0)
  //   return -1;

  for (list<OurEdge*>::iterator it=fixedEdges.begin(); it != fixedEdges.end(); it++) {
    OurEdge* fixed = *it;
    if (DEBUG == 13)
      cout << "aresta fixa: " << fixed->u << "-" << fixed->v << endl;
    solution.push_back(fixed);
    sum+= fixed->cost;
  }

  Digraph compDigraph;
  LengthMap compLength(compDigraph);
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

  if (DEBUG == 13)
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
	if (DEBUG == 13)
	  cout << "fixedEdge: " << e->u << "-" << e->v << endl;
	fixedEdgeFlag = 1;	  
      }
    }

    if (DEBUG == 12)
      cout << "ponto 2" << endl;

    if (fixedEdgeFlag) {
      compLength[arcComp] = e->cost;
      compLength[arc2Comp] = e->cost;
      
      // tempDigraph.erase(arc);
      // tempDigraph.erase(arc2);      
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
	// tempDigraph.erase(arc);
	// tempDigraph.erase(arc2);	
      } else { //not removed nor fixed
	// tempLength[arc] = e->cost;
	// tempLength[arc2] = e->cost;	
	notFixedNorRemovedEdges[j] = e;
	j++;	
      }
    }
  }

  if (DEBUG == 12)
    cout << "ponto 12" << endl;

  Undirector<Digraph> uCompDigraph(compDigraph);

  Graph::NodeMap<int> compMap(uCompDigraph);
//Digraph::NodeMap<int> compMap(compDigraph);
  int nComp = connectedComponents (uCompDigraph, compMap);
  int edgesBetComp[nComp][nComp];
// for (int i = 0; i < nComp; i++)
//   for (int j = i+1; j < nComp; j++) {
//     edgesBetComp[i][j] = 0;
//     edgesBetComp[j][i] = 0;
//   }

int comp[input.nNodes];
for (int i = 0; i < input.nNodes; i++)
  comp[i] = compMap[nrComp[digraphNodes[i]]];

  std::sort(notFixedNorRemovedEdges.begin(), notFixedNorRemovedEdges.end(), compareEdgesCost);

  if (DEBUG == 12)
    cout << "ponto 5, nComp: " << nComp << endl;


  for (int i = 0; i < remainEdgesQt; i++) {
    
    Undirector<Digraph> uCompDigraph(compDigraph);
    
    OurEdge* edge = notFixedNorRemovedEdges[i];

    if (DEBUG == 12)
      cout << "aresta nem fixa nem removida: " << edge->u << "-" << edge->v << endl;
    
    if (DEBUG == 12)
      cout << "ponto 5.5, u: " << edge->u << ", v: " << edge->v << endl;
    
    Digraph::Node s = digraph.source(edge->uv);
    Digraph::Node t = digraph.target(edge->uv);

    if (DEBUG == 12)
      cout << "ponto 5.6" << endl;
    
    // Digraph::Node nodi = nr[s];
    // Digraph::Node nodj = nr[t];

    // if (DEBUG == 9)
    //   cout << "ponto 5.7" << endl;

    // int uComp = compMap[nrComp[digraphNodes[edge->u]]];
    // int vComp = compMap[nrComp[digraphNodes[edge->v]]];
    int uComp = comp[edge->u];
    int vComp = comp[edge->v];

    if (DEBUG == 12) {
      cout << "edge->u: " << edge->u << ", edge->v: " << edge->v << endl;
      // cout << "comp[u]" << comp[edge->u] << ", comp[v]: " << comp[edge->v] << endl;
      // cout << "compMap[u]" << compMap[nrComp[digraphNodes[edge->u]]] << ", compMap[v]: " << compMap[nrComp[digraphNodes[edge->v]]] << endl;
    }

    
    if (uComp != vComp) {
      //if ((uComp != vComp) && (!edgesBetComp[uComp][vComp])) {

      if (DEBUG == 12)
	cout << "ponto 5.8" << endl;
      
      for (int j = 0; j < input.nNodes; j++) {
	if (comp[j] == uComp)
	  comp[j] = vComp;
	if (DEBUG == 10)
	  cout << "comp[" << j << "]: " << comp[j] << endl;
      }

      solution.push_back(edge);
      sum+= edge->cost;

      
      // edgesBetComp[uComp][vComp] = 1;
      // edgesBetComp[vComp][uComp] = 1;

      // Dijkstra<Digraph, LengthMap> dijkstra(tempDigraph, tempLength);
      // bool reached = dijkstra.run(nodi, nodj);
    
      // if (DEBUG == 10)
      // 	cout << "ponto 6" << endl;
    
      // if (!reached) {

	// if (DEBUG == 10)
	//   cout << "ponto 6.5" << endl;

	
	// Digraph::Arc arcIn = tempDigraph.addArc(nodi, nodj);
	// Digraph::Arc arcOut = tempDigraph.addArc(nodj, nodi);
	// IloNum len = edge->cost;
	// tempLength[arcIn] = len;
	// tempLength[arcOut] = len;

	// solution.push_back(edge);
	// sum+= len;
      // }	
      
      
    }

 
    if (DEBUG == 12)
      cout << "ponto 7" << endl;
    
  }

  if (DEBUG == 13)
    cout << "fim de getMSTbasedOnFixedEdges, cuja some eh: " << sum << endl;

  return sum;
}


/*
Dado as componentes representadas pelas arestas que têm valor diferente 
de zero
*/
double getPathWLowestIntGap(list<OurEdge*>& removedEdges) {
  Digraph compDigraph;
  LengthMap compLength(compDigraph);
  Digraph::NodeMap<Digraph::Node> nrComp(digraph);
  Digraph::NodeMap<Digraph::Node> ncrComp(digraph);

  digraphCopy(digraph, compDigraph).
    nodeRef(nrComp).
    nodeCrossRef(ncrComp).
    run();
  
  Digraph tempDigraph;
  LengthMap tempLength(tempDigraph);
  Digraph::NodeMap<Digraph::Node> nr(digraph);
  Digraph::NodeMap<Digraph::Node> ncr(digraph);
  
  digraphCopy(digraph, tempDigraph).
    nodeRef(nr).
    nodeCrossRef(ncr).
    run();

  if (DEBUG == 7)
    cout << "ponto 0.5" << endl;

  if (DEBUG == 7)
    cout << "MIPFLAG: " << MIPFlag << endl;

  FOR_EACH_EDGE_e
    if (DEBUG == 6)
      cout << "ponto 0.6" << endl;    
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
      cout << "ponto 0.65" << endl;
    double ub = masterLP.varXArray[e->label].getUB();

    if (DEBUG == 6)
      cout << "ponto 0.65" << endl;
    
    Digraph::Node s = digraph.source(e->uv);
    Digraph::Node t = digraph.target(e->uv);
    Digraph::Arc arc = findArc(tempDigraph, nr[s], nr[t]);      
    Digraph::Arc arc2 = findArc(tempDigraph, nr[t], nr[s]);    

    if (DEBUG == 6)
      cout << "ponto 0.7" << endl;
    
    Digraph::Arc arcComp = findArc(compDigraph, nrComp[s], nrComp[t]);      
    Digraph::Arc arc2Comp = findArc(compDigraph, nrComp[t], nrComp[s]);    
    
    if (val == 1) {
      if (DEBUG == 6)
	cout << "ponto 1" << endl;
      compLength[arcComp] = val;
      compLength[arc2Comp] = val;
      
      tempDigraph.erase(arc);
      tempDigraph.erase(arc2);      
    } else { //zero ou fracionario
      if (DEBUG == 6)
	cout << "ponto 2" << endl;

      
      int flag = 0;
      for (list<OurEdge*>::iterator it=removedEdges.begin(); (it != removedEdges.end()) && !flag; it++) {
	OurEdge* remEdge = *it;
	//cout << "analisando aresta: " << remEdge->label << endl;
	if (remEdge->label == e->label) {	  
	  flag = 1;

	  if (DEBUG == 6)
	    cout << "ponto 3" << endl;
	  
	}
      }

      compDigraph.erase(arcComp);
      compDigraph.erase(arc2Comp);      
      
      
      if (flag) {
        if (DEBUG == 6)
	  cout << "ponto 4" << endl;
	
	tempDigraph.erase(arc);
	tempDigraph.erase(arc2);
      } else {
        if (DEBUG == 6)
	  cout << "ponto 5" << endl;
	
	double int_gap = e->cost - (e->cost * val);
	tempLength[arc] = int_gap;
	tempLength[arc2] = int_gap;
      }
    }
  }

if (DEBUG == 7)
  cout << "antes de stronglyConnectedComponents" << endl;

  Digraph::NodeMap<int> compMap(compDigraph);
  int nComp = stronglyConnectedComponents (compDigraph, compMap);
  double min_int_gap = maxNumber;

if (DEBUG == 7)
  cout << "depois de stronglyConnectedComponents, o num de componentes eh: " << nComp << endl;


  for (int i = 0; i < input.nNodes; i++) {    

    for (int j = i+1; j < input.nNodes; j++) {      

      if (compMap[nrComp[digraphNodes[i]]] != compMap[nrComp[digraphNodes[j]]]) {
	Digraph::Node nodi = nr[digraphNodes[i]];
	Digraph::Node nodj = nr[digraphNodes[j]];

	Dijkstra<Digraph, LengthMap> dijkstra(tempDigraph, tempLength);
	bool reached = dijkstra.run(nodi, nodj);	
	if (reached) {
	  double dist = dijkstra.dist(nodj);
	  if (dist < min_int_gap)
	    min_int_gap = dist;
	}	
      }
    }
  }

  if (min_int_gap < maxNumber) {
    if (DEBUG == 7)
      cout << "min_int_gap: " << min_int_gap << endl;
      return min_int_gap;
  } else
      return 0;

}

OurEdge* getFracVarWithLowestIntGap(double& frac_val) {
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

//estratégia de arrendondamento do LB
IloNum roundLB(BranchTreeNode* curNode) {
  if (DEBUG == 7)
    cout << "dentro de roundLB" << endl;
  //simple strategy
  return ceil(curNode->lower_bound);
  //return ceil(getPathWLowestIntGap(curNode->zero_var) + curNode->lower_bound);
  
  // if (input.unit) {
  //   if (DEBUG == 7)
  //     cout << "o grafo eh unitario" << endl;    
  //   return ceil(curNode->lower_bound);
  // } else {
  //   return ceil(getPathWLowestIntGap(curNode->zero_var) + curNode->lower_bound);
    
  //   // double val;
  //   // OurEdge* edge = getFracVarWithLowestIntGap(val);
  //   // if (edge != NULL)
  //   //   return ceil(curNode->lower_bound + val);
  //   // else
  //   //   return ceil(curNode->lower_bound);
  // }

  /*more complex: what is the smallest integer greater than lb that represents a valid set of edges. 
    For a fractional path p, maybe rounding to ceil(lb) do not represents a valid set of edges, as 
    the costs of the edges are different.
    => bin packing problem.
   */  
}
//-------------------------------------------------

bool compareEdges (OurEdge* e, OurEdge* f) { return (e->label < f->label); }

// bool compareEqualEdges (OurEdge* e, OurEdge* f) { return (e->label == f->label); }

int checkSolution() {
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
  for (list<OurEdge*>::iterator it = edgesSol.begin(); it != edgesSol.end(); it++) {
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
      return 0;
    }
  }

  return 1;  
}

/*
Removed edges must not be part of the set of available edges.
FixedEdges will be forced to be part of solution.
*/
IloNum greedySpanner(list<OurEdge*>& removedEdges, list<OurEdge*>& fixedEdges, list<OurEdge*>& solution) {

  if (DEBUG == 13)
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
  // for (int i = 0; i < input.nEdges; i++) {
  //   OurEdge* e = edges[i];
    Digraph::Node s = digraph.source(e->uv);
    Digraph::Node t = digraph.target(e->uv);
    
    Digraph::Arc arc = findArc(tempDigraph, nr[s], nr[t]);
    tempDigraph.erase(arc);
    Digraph::Arc arc2 = findArc(tempDigraph, nr[t], nr[s]);
    tempDigraph.erase(arc2);
  }
  
  
//----------------
  vector<OurEdge*> edges;
//int remainEdgesQt = input.nEdges - removedEdges.size() - fixedEdges.size();
  int remainEdgesQt = input.nEdges - fixedEdges.size();
  edges.reserve(remainEdgesQt);
  if (DEBUG == 13)
    cout << "input.nEdges: " << input.nEdges << ", removedEdges: " << removedEdges.size() << ", tamanho de edges: " << edges.size() << endl;
  int j = 0;
  FOR_EACH_EDGE_e
    int flag = 0;
    for (list<OurEdge*>::iterator it=removedEdges.begin(); (it != removedEdges.end()) && !flag; it++) {
      OurEdge* remEdge = *it;
      //cout << "analisando aresta: " << remEdge->label << endl;

      if (remEdge->label == e->label) {
	if (DEBUG == 6)
	  cout << "aresta removida: " << remEdge->u << "-" << remEdge->v << endl;	
	flag = 1;
      }
    }

    int fixedFlag = 0;
    if (!flag) {//if the edge is in removedEdges, it cannot be in fixedEdges
      list<OurEdge*>::iterator it2=fixedEdges.begin();      
      for (int k = 0; (k < fixedEdges.size()) && !fixedFlag; k++, it2++) {
	OurEdge* fixEdge = *it2;
	//cout << "analisando aresta: " << remEdge->label << endl;

	if (fixEdge->label == e->label) {
	  if (DEBUG == 13)
	    cout << "aresta fixa: " << fixEdge->u << "-" << fixEdge->v << endl;
	  
	  fixedFlag = 1;

	  Digraph::Node s = digraph.source(e->uv);
	  Digraph::Node t = digraph.target(e->uv);
	  Digraph::Arc arcIn = tempDigraph.addArc(nr[s], nr[t]);
	  Digraph::Arc arcOut = tempDigraph.addArc(nr[t], nr[s]);
	  IloNum len = e->cost;
	  tempLength[arcIn] = len;
	  tempLength[arcOut] = len;

	  solution.push_back(e);
	  ub += len;	  
	}
      }      
    }

    if (!fixedFlag) {
//if (!flag && !fixedFlag) {
      if (DEBUG == 5)
	cout << "adicionou a aresta: " << e->u << "-" << e->v << endl;
      edges[j] = input.edges[e->label];
      j++;
    }
    
    // std::list<OurEdge*>::iterator findIter = find(removedEdges.begin(), removedEdges.end(), e, compareEqualEdges);
    // if (findIter != removedEdges.end())
    //edges[e->label] = input.edges[e->label];
  }
  std::sort(edges.begin(), edges.end(), compareEdges);
  
//estava aqui

//erro - está faltando as arestas que foram removidas. Tem que checar a
//propriedade de spanner para elas
//list<OurEdge*> tempSol = list<OurEdge*>();
  for (int i = 0; i < remainEdgesQt; i++) {
    OurEdge* e = edges[i];
    if (DEBUG == 13)
      cout << "aresta que ficou: " << e-> u << "-" << e->v << endl;
    Digraph::Node s = digraph.source(e->uv);
    Digraph::Node t = digraph.target(e->uv);
    
    Dijkstra<Digraph, LengthMap> dijkstra(tempDigraph, tempLength);
    bool reached = dijkstra.run(nr[s], nr[t]);
    if ((!reached) || ((input.strFactor * e->cost) < dijkstra.dist(nr[t]))) {
      Digraph::Arc arcIn = tempDigraph.addArc(nr[s], nr[t]);
      Digraph::Arc arcOut = tempDigraph.addArc(nr[t], nr[s]);
      IloNum len = e->cost;
      tempLength[arcIn] = len;
      tempLength[arcOut] = len;

      solution.push_back(e);
      ub += len;
    }
  }



  if (DEBUG == 13)
    cout << "valor do ub: " << ub << endl;

  return ub;

}

/*
IloNum callCplexMIPSolver(IloNum globalUB) {
  if (DEBUG == 7)
    cout << "dentro de callCplexMIPSolver" << endl;
  MIPFlag = true;
  IloNum objVal;
  //eu posso remover as restrições de branch (xe == 0, xe == 1) da heurística. Como a heurística tem um novo conjunto de colunas,
  //as restrições nas variáveis não são mais importantes.
    // solve IP
  FOR_EACH_EDGE_e
    //lpBefMIPValues[e] = masterLP.solver->getValue(e->varX);
    lpBefMIPValues[e] = masterLP.solver->getValue(masterLP.varXArray[e->label]);
  }
  
  FOR_EACH_EDGE_e
  //e->conv = new IloConversion(masterLP.env, e->varX, ILOBOOL);
    e->conv = new IloConversion(masterLP.env, masterLP.varXArray[e->label], ILOBOOL);
    masterLP.model->add(*(e->conv));
  }

  if (masterLP.solver->solve()) {
    objVal = (int)masterLP.solver->getObjValue();
    if (objVal < globalUB) {
      if (DEBUG == 7)
	cout << "nova solução encontrada: " << endl;
      globalUB = setSolution();
      //globalUB = objVal;
    }  
  }
    
  // re-solve the LP
  FOR_EACH_EDGE_e
    e->conv->end();
    if (DEBUG == 12)
      cout << "antes do delete 11" << endl;
      delete e->conv;
    if (DEBUG == 12)
      cout << "depois do delete 11" << endl;

  }

	// if (DEBUG == 6)
	//   return objVal;


   //poderia guardar a solução ao invés de calcular a solução do LP novamente
//callCplexLPSolver();
  // if (DEBUG == 2)
  //   cout << "resolveu o modelo agora com variaveis fracionarias" << endl;
    
  return objVal;
}
*/

// void removeYpColumns(BranchTreeNode* curNode) {
//   //variaveis Yp so terao valor 0 (na propria inequacao ja impoem esta restricao nas variaveis)
//   IloRangeArray path_zero_constraint = IloAdd(*(masterLP.model), IloRangeArray(*(masterLP.env), 1, 0, 0));
//   curNode->path_zero_constraint = &path_zero_constraint;
//   for (int i = 0; i < curNode->branch_zero_constraint->getSize(); i++) {//percorrendo as arestas que estao fixadas em zero na arvore de decisao
//     for (int j = 0; j < masterLP.paths_for_edge[(*(curNode->branch_zero_constraint))[i]].size(); j++) {//percorrendo os caminhos (Yp) que contem a aresta do loop acima
//       IloNumVar* varY = masterLP.paths_for_edge[(*(curNode->branch_zero_constraint))[i]].get(j)->varY;
//       BRANCH_PATH_ZERO_CONSTRAINT.setLinearCoef(*(varY), 1.0);
//       //(*(curNode->path_zero_constraint))[i].setLinearCoef(*(varY), 1.0);
//     }     
//   }
// }

int solveMIP(int height) {
  int flag;
  int periodQtEdges = BRANCH_HEURISTIC_MIP_INTERVAL;
  if (DEBUG == 7)
    cout << "periodQtEdges: " << periodQtEdges << ", height: " << height << endl;
  //int periodQtEdges = (int) ceil((double)input.nEdges * BRANCH_HEURISTIC_HEIGHT_PERC);
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
  if (DEBUG == 13)
    cout << "dentro de getFixedEdges" << endl;
  
    FOR_EACH_EDGE_e
      if (e->bridge) {
	fixedEdges.push_back(e);
	if (DEBUG == 13)
	  cout << "aresta inserida em fixedEdges1: " << e->u << "-" << e->v << endl;	
      }
    }
  
  //fixing temporary bridges to be part of the solution
    list<OurEdge*>::iterator bridgeIt=curNode->bridge_var.begin();
    for (int i = 0; i < curNode->bridge_var.size(); i++, bridgeIt++) {
      OurEdge* e = *bridgeIt;
      fixedEdges.push_back(*bridgeIt);
	if (DEBUG == 13)
	  cout << "aresta inserida em fixedEdges2: " << e->u << "-" << e->v << endl;     
    }

    //fixing edges fixed by the branch tree to be part of solution
    list<OurEdge*>::iterator oneIt=curNode->one_var.begin();
    for (int i = 0; i < curNode->one_var.size(); i++, oneIt++) {
      OurEdge* edge = *oneIt;      
      fixedEdges.push_back(edge);
      if (DEBUG == 13)
	cout << "aresta inserida em fixedEdges3: " << edge->u << "-" << edge->v << endl;
    }  
}

IloNum executeGreedyHeuristic(BranchTreeNode* curNode, IloNum& globalUB) {
    list<OurEdge*> tempSol = list<OurEdge*>();
    list<OurEdge*> fixedEdges = list<OurEdge*>();

    if (DEBUG == 13)
      cout << "dentro de executeGreedyHeuristic" << endl;
    getFixedEdges(curNode, fixedEdges);

    IloNum tempUB = greedySpanner(curNode->zero_var, fixedEdges, tempSol); //heuristic
    if (DEBUG == 13)
      cout << "depois de greedySpanner" << endl;

    if (tempUB < globalUB) {
      edgesSol.clear();
      for (list<OurEdge*>::iterator it = tempSol.begin(); it != tempSol.end(); it++) {
	OurEdge* edge = *it;
	edgesSol.push_back(edge);
      }      
    }

    if (DEBUG == 13)
      cout << "fim de executeGreedyHeuristic" << endl;
    
    return tempUB;
}

void calculateUpperBound(BranchTreeNode* curNode, IloNum& globalUB) {
  if (DEBUG == 13)
    cout << "dentro de calculateUpperBound" << endl;
  IloNum tempUB = globalUB;
  /*
  if (solveMIP(curNode->height)) {
    tempUB = callCplexMIPSolver(globalUB); //heuristic to find a feasible solution
    } else */
  if (solveGreedy()){
    tempUB = executeGreedyHeuristic(curNode, globalUB);
    //tempUB = greedySpanner(curNode->zero_var, fixedEdges, tempSol); //heuristic to find a feasible solution
  }
  if (tempUB < globalUB) {
    globalUB = tempUB;
    //masterLP.solver->setParam(IloCplex::CutUp, globalUB);
  }

  if (DEBUG == 13)
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
  
  // if (DEBUG == 13)
  //   cout << "dentro de branchOnX" << endl;
//int branchOnX(list<BranchTreeNode*>& unexploredNodes, IloNum& globalUB) {

    // if (1 == 1)
    //   printViolatedConstraints();
  
  //BranchTreeNode* curNode = unexploredNodes.front();
  BranchTreeNode* curNode = unexploredNodes.top();
  if (DEBUG == 10)
    cout << "dentro de branchOnX2" << endl;  
  if (DEBUG == 13) {
    cout << "dentro de branchOnX, label de curNode: " << curNode->label << ", height: " << curNode->height << ", lb: " << curNode->lower_bound << ", UB: " << globalUB << endl;
    printTreePath(curNode);
    //printSolution(true, true, true);
  }
  if (DEBUG == 4)
    cout << "depois de ler o front dos unexploredNodes, o tamanho da lista eh: " << unexploredNodes.size() << endl;

  //if (roundLB(curNode->lower_bound) < globalUB) {
  if (DEBUG == 13)
    cout << "antes de roundLB, inicio de branchOnX" << endl;
    if (roundLB(curNode) < globalUB) {
      
      //checking what edges are bridge based on the (temporary) removed edges
      /*
	list<OurEdge*> tempBridgeSet = list<OurEdge*>();
	findBridges(curNode->zero_var, tempBridgeSet);

	for (list<OurEdge*>::iterator tempIt=tempBridgeSet.begin(); tempIt != tempBridgeSet.end(); tempIt++) {
	  OurEdge* edge = *tempIt;
	  if (DEBUG == 9)
	    cout << "fixou a aresta " << edge->u << "-" << edge->v << " no curNode cujo lb eh: " << curNode->lower_bound << endl;
	  curNode->bridge_var.push_back(edge);
	}
      */
	//--------------------------------------
	
      //---------------------adding branch (and bridge) restrictions-----------
	/*
	IloRangeArray branch_one_constraint = IloAdd(*(masterLP.model), IloRangeArray(masterLP.env, 1, curNode->one_var.size()*1, curNode->one_var.size()*1));

	IloRangeArray single_edge_path_constraint = IloAdd(*(masterLP.model), IloRangeArray(masterLP.env, 1, curNode->one_var.size()*1, curNode->one_var.size()*1)); 
	
      if (curNode->one_var.size() > 0) {
	//variaveis so terao valor 1 (na propria inequacao ja impoem esta restricao nas variaveis)
	curNode->branch_one_constraint = &branch_one_constraint;
	// curNode->branch_one_constraint->setNames("branch_cons");

	curNode->single_edge_path_constraint = &single_edge_path_constraint;
	// curNode->single_edge_path_constraint->setNames("single_edge");
      }      

	IloRangeArray bridge_constraint = IloAdd(*(masterLP.model), IloRangeArray(masterLP.env, 1, curNode->bridge_var.size()*1, curNode->bridge_var.size()*1));      
      if (curNode->bridge_var.size() > 0) {
	//variaveis so terao valor 1 (na propria inequacao ja impoem esta restricao nas variaveis)
	curNode->bridge_constraint = &bridge_constraint;
      }
      
	IloRangeArray branch_zero_constraint = IloAdd(*(masterLP.model), IloRangeArray(masterLP.env, 1, 0, 0));      
      if (curNode->zero_var.size() > 0) {
	//variaveis so terao valor 0 (na propria inequacao ja impoem esta restricao nas variaveis)
	curNode->branch_zero_constraint = &branch_zero_constraint;
	// curNode->branch_zero_constraint->setNames("branch zero constraints");

	//it's redundant as we have the edge constraints (when xe = 0, all paths that contain that edge have to be zero
	//removeYpColumns(curNode);
      }

      for (list<OurEdge*>::iterator it=curNode->zero_var.begin(); it != curNode->zero_var.end(); it++) {
      //for (int i = 0; i < curNode->branch_zero_constraint->getSize(); i++, it++) {
	OurEdge* e = *it;
	
	if (DEBUG == 9)
	  cout << "vai adicionar a aresta " << e->u << "-" << e->v << " na restricao de eliminacao de zero var" << endl;
	
	//BRANCH_ZERO_CONSTRAINT[0].setLinearCoef(e->varX, 1.0);
	BRANCH_ZERO_CONSTRAINT[0].setLinearCoef(masterLP.varXArray[e->label], 1.0);
      }

      for (list<OurEdge*>::iterator oneIt=curNode->one_var.begin(); oneIt != curNode->one_var.end(); oneIt++) {
	OurEdge* e = *oneIt;

	if (e->single_edge_path == NULL) {
	  if ((DEBUG == 9) && (e->u == 2) && (e->v == 5))
	    cout << "vai criar um novo caminho de uma aresta entre 2 e 5" << endl;
	  list<OurEdge*> edges = list<OurEdge*>();
	  edges.push_back(e);
	  SpanPath* p = newPath(getUVLabel(e), edges);
	  //e->single_edge_path = p;
	  addYVarToMasterLP(p);
	}
	SINGLE_EDGE_PATH_CONSTRAINT[0].setLinearCoef(e->single_edge_path->varYArray[0], 1.0);
	
	if (DEBUG == 9)
	  cout << "vai adicionar a aresta " << e->u << "-" << e->v << " na restricao de eliminacao de one var" << endl;	
	//BRANCH_ONE_CONSTRAINT[0].setLinearCoef(e->varX, 1.0);
	BRANCH_ONE_CONSTRAINT[0].setLinearCoef(masterLP.varXArray[e->label], 1.0);
      }

      for (list<OurEdge*>::iterator bridgeIt=curNode->bridge_var.begin(); bridgeIt != curNode->bridge_var.end(); bridgeIt++) {
	OurEdge* e = *bridgeIt;
	BRIDGE_CONSTRAINT[0].setLinearCoef(masterLP.varXArray[e->label], 1.0);
      } 
	*/         
      /*-----------------------------------------------------*/

      //pricing
      if (DEBUG == 9) {
	printTreePath(curNode);
	printConstraints();
	cout << "antes de price, branchOnX, lb: " << curNode->lower_bound << ", altura: " << curNode->height << endl;
      }
      // if (1 == 1)
      // 	exit(0);
      
      //curNode->lower_bound = price(curNode/*, trieRoot, masks*/);

      list<OurEdge*> fixedEdges = list<OurEdge*>();
      getFixedEdges(curNode, fixedEdges);
    if (DEBUG == 13)
      cout << "depois de getFixedEdges" << endl;
      list<OurEdge*> tempSol = list<OurEdge*>();
      curNode->lower_bound = getMSTbasedOnFixedEdges(fixedEdges, curNode->zero_var, tempSol);

    if (DEBUG == 13)
      cout << "depois de price, tempo: " << getSpentTime() << endl;      
            
      if (getSpentTime() >= TIME_LIMIT)
	return TIME_EXCEEDED;
      
      // if (curNode->lower_bound < (*bestLBNode)->lower_bound)
      // 	*bestLBNode = curNode;
      if (DEBUG == 7) {
	cout << "depois de price e antes de calculateUpperBound, branchOnX" << endl;
	//printSolution(true, true, true);
      }
      
      //calculateUpperBound(curNode, globalUB);

      if (DEBUG == 13)
	cout << "antes de roundLB, depois de calculateUpperBound" << endl;

      if (roundLB(curNode) < globalUB) {	
	if (!createBranchNodes(curNode, unexploredNodes)) {
	  if (DEBUG == 12)
	    cout << "antes de setSolution" << endl;	  
	  //globalUB = setSolution();
	}
      }
      //else - time to prune

      //---------time to remove branch restrictions (just 2 restrictions)--------
      /*
      if (DEBUG == 12)
	cout << "depois de createBranchNodes, hora de remover branch restrictions" << endl;
      if (curNode->one_var.size() > 0) {
	if (DEBUG == 12)
	  cout << "vai remover as restrições de branch das variaveis 1" << endl;	
	masterLP.model->remove(BRANCH_ONE_CONSTRAINT[0]);
	masterLP.model->remove(SINGLE_EDGE_PATH_CONSTRAINT[0]);
      }
      if (curNode->bridge_var.size() > 0) {
	if (DEBUG == 12)
	  cout << "vai remover as restrições de branch das variaveis bridge" << endl;	
	masterLP.model->remove(BRIDGE_CONSTRAINT[0]);
      }      
      if (curNode->zero_var.size() > 0) {
	if (DEBUG == 12)
	  cout << "vai remover as restrições de branch das variaveis 0" << endl;	
	masterLP.model->remove(BRANCH_ZERO_CONSTRAINT[0]);
	//it's redundant as we have the edge constraints (when xe = 0, all paths that contain that edge have to be zero
      }

      callCplexLPSolver();
      */
      //-----------------------------------------------------    
    }

    //unexploredNodes.pop_front();
    //unexploredNodes.erase(unexploredNodes.begin());
    unexploredNodes.pop();
    if (DEBUG == 13)
      cout << "apagou no dos unexplored nodes" << endl;    

    if ((unexploredNodes.size() == 0)) {
      if (DEBUG == 10)
	cout << "finalizaram os nos da arvore de branch" << endl;
      return BRANCH_FINISHED;
    } else {
      if (DEBUG == 4)
	cout << "ainda tem nos na arvore de branch" << endl;      
      return BRANCH_NOT_FINISHED;
    }
}

void findUnAllowedEdges() {
  if (DEBUG == 10)
    cout << "dentro de findUnAllowedEdges" << endl;
  FOR_EACH_EDGE_e
    if (e->cost > input.strFactor * input.dist[e->u][e->v]) {
      masterLP.varXArray[e->label].setBounds(0,0);
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
  edges.reserve(input.nEdges - removedEdges.size());

  /*creating the temporary graph with non-zero edges (i.e., edges that were
not fixed to zero in the branch tree)
   */
  int j = 0;
  FOR_EACH_EDGE_e
    int flag = 0;
    for (list<OurEdge*>::iterator it=removedEdges.begin(); (it != removedEdges.end()) && !flag; it++) {
      OurEdge* remEdge = *it;
      //cout << "analisando aresta: " << remEdge->label << endl;

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
    //FOR_EACH_EDGE_e
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
	// masterLP.varXArray[e->label].setBounds(1,1);
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
  masterLP.solver->exportModel("CGmodel.lp");
  //masterLP.solver->setParam(IloCplex::Param::RootAlgorithm, 1);
}

void deleteObjects(BranchTreeNode* branchRoot, IloEnv* env, IloModel* model, IloCplex* solver) {
  if (DEBUG == 12)
    cout << "dentro de deleteObjects" << endl;  
  //generated paths
  FOR_EACH_ADDED_COLUMN
    SpanPath* path = *it;
    delete path->edges;
  if (DEBUG == 12)
    cout << "delete 0.5" << endl;    
    delete path;
  }
  if (DEBUG == 12)
    cout << "delete 1" << endl;

  delete masterLP.pi;
  delete masterLP.sigma;
  if (DEBUG == 12)
    cout << "delete 2" << endl;

  //edges
  FOR_EACH_EDGE_e
    delete e;
  }
  input.edges.clear();
  if (DEBUG == 12)
    cout << "delete 3" << endl;


  //dist array
  for (int i = 0; i < input.nNodes; i++) {
    delete input.dist[i];
  }
  delete input.dist;
  if (DEBUG == 12)
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
    if (DEBUG == 12)
      cout << "delete 5" << endl;    
  }
  if (DEBUG == 12)
    cout << "delete 6" << endl;

  //cplex objects
  (*solver).end();
  (*model).end();
  (*env).end();

}

void printViolatedConstraints() {
  // Perform conflict analysis. 
  // We allow any constraint to be part of the conflict. 

  // Build a list of constraints that can be part of the conflict. 
  // Since bounds of variables may also render the model infeasible 
  // we also add bounds to the set of constraints that are considered 
  // for a conflict. 
  cout << "Building constraint list ... " << flush; 
  IloConstraintArray cons(masterLP.env); 
  IloNumArray prefs(masterLP.env); 
  for (IloModel::Iterator it(masterLP.solver->getModel()); it.ok(); ++it)
  { 
    IloExtractable e = *it; 
    if (e.isVariable()) { 
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
      IloConstraint con = e.asConstraint();
      std::string str(con.getName());
      if (
	  (str.find("single_edge") !=std::string::npos)  ||
	(str.find("branch_cons") !=std::string::npos)  ||
	  (str.find("path_cons") !=std::string::npos) ) 
	cout << "cons: " << e.asConstraint() << endl;
      cons.add(e.asConstraint()); 
      prefs.add(1.0); 
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

// ./spanner < input_file
int main(int argc, char* argv[]) {
  //  flagPrice = 0;
  // time_t start, end;
  // start = clock();
  double total_time;  
  //double objValue = 0;
  IloNum globalUB;

    IloEnv env;
    IloModel model(env);
    IloCplex solver(model);

    
  ofstream timeOutFile;

  BranchTreeNode* branchRoot;

  edgesSol = list<OurEdge*>();

  resetTime();
  
  try {

    if (DEBUG == 13)
      cout << "antes de readINput" << endl;
    maxNumber = readInput(argv[1], argv[2]) + 1;
    globalUB = maxNumber;    

    if (DEBUG == 13)
      cout << "depois de readInput, tempo: " << getSpentTime() << endl;
    
    // IloRangeArray edge_constraints = IloAdd(model, IloRangeArray(env, input.nK * input.nEdges, 0, IloInfinity));
    // IloRangeArray path_constraints = IloAdd(model, IloRangeArray(env, input.nK, 1, IloInfinity));
    // //IloRangeArray bound_constraints = IloAdd(model, IloRangeArray(env, input.nEdges, 0, IloInfinity));
    // IloRangeArray bound_constraints;
    // IloObjective obj = IloAdd(model, IloMinimize(env)); //nao deveria ser IloMaximize e colocar o sinal de menos para estar na forma padrao?
    
    initLemonAndInputVariables();
    if (DEBUG == 13)
      cout << "depois de initLemonAndInputVariables, tempo: " << getSpentTime() << endl;    
    calcDistances();

    if (DEBUG == 13)
      cout << "depois de calcDistances, tempo: " << getSpentTime() << endl;

    //BranchTreeNode* bestLBNode = branchRoot;
    //masterLP.varXArray = IloNumVarArray(env, input.nEdges, 0, 1, ILOFLOAT);
    
    //---------local lower bound-------------
    //initMasterLP(env, &model, &edge_constraints, &path_constraints, &bound_constraints, &obj, &solver);

    if (DEBUG == 13)
      cout << "depois de initMasterLP, tempo: " << getSpentTime() << endl;
    
    //setCplexParam();
    if (DEBUG == 13)
      cout << "depois de initMasterLP" << endl;
    //---------Constroi o RMP, resolve o modelo-------------------
    //buildInitialMasterLP();

    if (DEBUG == 13)
      cout << "depois de buildInitialMasterLP, tempo: " << getSpentTime() << endl;
    
    // if (!input.unit) {
    //   findUnAllowedEdges();
    // }

    if (DEBUG == 13)
      cout << "depois de findUnAllowedEdges, tempo: " << getSpentTime() << endl;
    
    if (DEBUG == 7)
      cout << "depois de buildInitialMasterLP" << endl;
    initializeBranchTreeNode(&branchRoot);
    // IloNum localLB = getFirstSolution(env, &model, &solver, &edge_constraints, &path_constraints, &bound_constraints, &obj/*, trieRoot, masks*/);
      list<OurEdge*> fixedEdges = list<OurEdge*>();
      getFixedEdges(branchRoot, fixedEdges);
    if (DEBUG == 13)
      cout << "depois de getFixedEdges" << endl;
      list<OurEdge*> tempSol = list<OurEdge*>();
      IloNum localLB = getMSTbasedOnFixedEdges(fixedEdges, branchRoot->zero_var, tempSol);

    
    if (DEBUG == 13)
      cout << "depois de getFirstSolution, tempo: " << getSpentTime() << endl;
    
    //------------------------------------------------------------
    if (localLB != TIME_EXCEEDED) {

      if (DEBUG == 10)
	cout << "depois de getFirstSolution" << endl;
      //initializeBranchTreeNode(&branchRoot);
      branchRoot->height = 0;
      branchRoot->lower_bound = localLB;
      branchRoot->ancestor = NULL;
      //-----------------------------------

      //---------updating upper bound-----------
      globalUB = executeGreedyHeuristic(branchRoot, globalUB);      
      //masterLP.solver->setParam(IloCplex::CutUp, globalUB);

      if (DEBUG == 13)
	cout << "depois de getFirstSolution, tempo: " << getSpentTime() << endl;
    
      //calculateUpperBound(branchRoot, globalUB);
      if (DEBUG == 13) {
	cout << "depois de calculateUpperBound" << endl;
	cout << "lb: " << branchRoot->lower_bound << ", ub: " << globalUB << endl;
      }
      // if (1 == 1)
      //   return 1;
      //---------------------------------------

      //if (roundLB(branchRoot->lower_bound) < globalUB) {
      if (localLB < globalUB) {
      //if (roundLB(branchRoot) < globalUB) {
	priority_queue<BranchTreeNode*, std::vector<BranchTreeNode*>, compLB> unexploredNodes(compareLB);
	//unexploredNodes.push_back(branchRoot);

	if (DEBUG == 11)
	  cout << "antes de createBranchNodes" << endl;
	
	if (!createBranchNodes(branchRoot, unexploredNodes)) {
	  globalUB = setSolution(tempSol);
	  if (DEBUG == 13)
	    cout << "nao criou branch nodes" << endl;
	  //setSolution();
        } else {
	  if (DEBUG == 13)
	    cout << "depois de createBranchNodes" << endl;	

	  // if (1 == 1)
	  // 	return 0;

	  while (!branchOnX(unexploredNodes, globalUB/*, &bestLBNode, trieRoot, masks*/));
	
        }

      } else {
	if (DEBUG == 10)
	  cout << "nao precisou fazer branch and bound" << endl;
      }
      
    }
    // if (1 == 1)
    //   return 0;
    

    //end = clock();
    if (getSpentTime() < TIME_LIMIT) {
      cout << "solução ótima: " << globalUB << endl;
      if (checkSolution()) {
	cout << "solução está ok" << endl;
	//printSolution();
      } else
	cout << "solução não é spanner" << endl;      
    }

    //printSolution(true, false, false);
    
    // for (list<OurEdge*>::iterator it = edgesSol.begin(); it != edgesSol.end(); it++) {
    //   OurEdge* edge = *it;
    //   cout << edge->u << "-" << edge->v << endl;
    // }
    
    // if ( masterLP.solver->solve() ) {
    //   end = clock();
    //   //end2 = masterLP.solver->getTime();

    //   IloAlgorithm::Status solStatus= masterLP.solver->getStatus();
    //   cout << "solStatus: " << solStatus << endl;

    //   if ( solStatus == IloAlgorithm::Optimal ) {

    // 	cout << "Solution is optimal" << endl;
    // 	cout << "Objective value: "
    // 		  << masterLP.solver->getObjValue() << endl;
    // 	objValue = masterLP.solver->getObjValue();

    // 	//printSolution(true, true, true, true);	   
    // 	// IloNumArray sol(masterLP.env, input.nEdges);
    // 	// masterLP.solver->getValues(sol, cplexLP->x);
    // 	// IloNum tol = masterLP.solver->getParam(IloCplex::EpInt);
    // 	// separate(sol, tol, input.nNodes, input.nEdges);
	   
    // 	// if (!isSolutionConnected())
    // 	//   cout << "Grafo final nao eh conexo" << endl;

    //   } else {
    // 	cout << "Solution status is not Optimal" << endl;
    //   }
    // } else {
    //   //noSolutionFlag = 1;
    //   end = clock();
    //   cout << "No solution available" << endl;
    //   // if (1 == 0)
    //   // 	printViolatedConstraints();      
    // }

  } catch (const IloException& e) {
      cerr << "Exception caught: " << e << endl;
  }
  catch (...) {
    cerr << "Unknown exception caught!" << endl;
  }


  int val = 1;
  //double optSolution = 0;
  //armazenando estatísticas
  //total_time = (double)( end - start )/(double)CLOCKS_PER_SEC ;
  total_time = getSpentTime();
  //total_time2 = (double)( end2 - start2 );
  cout << "total time: " << total_time << endl;
  //cout << "total time2: " << total_time2 << endl;;
  timeOutFile.open (argv[3], ios::out | ios::app);
  //timeOutFile.open (argv[3], ios::out);
  //cout << argv[3] << endl;
  double qtPriceVal = (qtPrice > 0) ? (price_total_time / qtPrice) : -1;
  if (total_time >= TIME_LIMIT) {
    val = 0;
    timeOutFile << argv[4] << " " << 0 << " " << total_time << " " << qtZeroBranchNodes << " " << qtOneBranchNodes << " " << masterLP.colsAdded.size() << " " << qtPrice << " " << qtPriceVal << endl;
  } else {
    //optSolution = 1;
    //optSolution = masterLP.solver->getObjValue();
    timeOutFile << argv[4] << " " << globalUB << " " << total_time << " " << qtZeroBranchNodes << " " << qtOneBranchNodes << " " << masterLP.colsAdded.size() << " " << qtPrice << " " << qtPriceVal << endl;
  }
  timeOutFile.close();

  deleteObjects(branchRoot, &env, &model, &solver);

  
  return val;
}
