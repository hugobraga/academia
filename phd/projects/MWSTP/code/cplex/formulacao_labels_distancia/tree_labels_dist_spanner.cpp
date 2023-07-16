#include "tree_labels_dist_spanner.h"
// #include "separate.h"
// #include "common.h"

#include <string>
#include <cmath>
#include <vector>
#include <algorithm>
//#include <limits> //infinity

#include <lemon/connectivity.h>
#include <lemon/min_cost_arborescence.h>

#include <lemon/preflow.h>

#include <lemon/path.h>

#include <stdio.h>
#include <stdlib.h>

#include <lemon/adaptors.h>

#include <time.h>

#include <sstream>
#include <fstream>

namespace {
  unsigned const DEBUG = 16;
  unsigned const BOUND_FLAG = 0;

  int const UNIT_BOUND_USED = -2;
  int const UNIT_BOUND_UNUSED = -3;

  int const TIME_LIMIT_CONST = 0;
  int const NO_INTEGRALITY_GAP = -1;
  int const NO_SOLUTION_CONST = -1;
};

bool incumbentFlag = false;

//LemonObjects* lemonObjects;
InputData input;
CplexEnv cplexLP;

Digraph digraph;  
vector<Digraph::Node> digraphNodes;
LengthMap weight(digraph);
LengthMap invWeight(digraph);
EdgeMap edgeMap;
ReverseEdgeMap revEdgeMap;
ReverseNodeMap nodeLabelMap;
NodeMap labelNodeMap;

clock_t initialClock;

/*----------FNCTIONS DEFINITION----------------*/
void initLemonAndInputVaRiables();
IloNum LB(MinCostArborescence<Digraph, LengthMap> mca, Digraph::Node root);
IloNum UB(MinCostArborescence<Digraph, LengthMap> minca, Digraph::Node root);
void initCplexLP(IloEnv _env, IloModel* _model, IloCplex* _solver);
void initVariables(IloEnv _env, IloModel* _model, IloCplex* _solver);
void calcDistances();
void calcLongestDistances();
void createLPFormulation(IloEnv env);
void readInput(int argc, char **argv, IloEnv env);
void setCplexParam();
void findOneDegreeVerticesAndBoundZ();
// void findBridges(Digraph& digraph, LengthMap& weight);
void freeMemoryCloseVar();
void printSolution(bool xFlag, bool zFlag, bool uFlag, bool printValue);
void printViolatedConstraints();
double getMij(Edge* e);
int getNEdges();
int getArcIndex(int edgeLabel, bool in);
/*----------------------------------------------*/

void resetTime() {
    initialClock = clock();
}

float getTime() {
  return (float) (clock()) / CLOCKS_PER_SEC;
}

float getSpentTime() {
    return (float)(clock() - initialClock) / CLOCKS_PER_SEC;
}


 void initLemonAndInputVariables2() {
   digraphNodes.reserve(input.nNodes);
   input.nodes.reserve(input.nNodes);
   //input.edges.reserve(input.nEdges);
   input.arcs.reserve(input.nEdges);

   for (int i = 0; i < input.nNodes; i++) {
     digraphNodes[i] = digraph.addNode();
     Digraph::Node* lemonNode = &(digraphNodes[i]);

     Node* node = new Node();
     node->nodeRef = lemonNode;
     node->label = i;
     labelNodeMap[i] = node;
     nodeLabelMap[*(node->nodeRef)] = i;

     input.nodes[i] = node;
   }

   FOR_EACH_EDGE_e
     Digraph::Arc f = digraph.addArc(*(labelNodeMap[e->u]->nodeRef), *(labelNodeMap[e->v]->nodeRef));
     Digraph::Arc g = digraph.addArc(*(labelNodeMap[e->v]->nodeRef), *(labelNodeMap[e->u]->nodeRef));
     weight[f] = e->cost;
     weight[g] = e->cost;
     invWeight[f] = 1/e->cost;
     invWeight[g] = 1/e->cost;     
     edgeMap[f] = e;
     edgeMap[g] = e;

     EdgeArcs* arcs = new EdgeArcs();
     arcs->in = f;
     arcs->out = g;
     arcs->label = e->label;
     input.arcs[e->label] = arcs;

     revEdgeMap[e] = arcs;     
   }
 }

void findOneDegreeVerticesAndBoundZ() {
  int degree[input.nNodes];

  for (int i = 0; i < input.nNodes; i++)
    degree[i] = 0;
  
  FOR_EACH_EDGE_e
    degree[e->u]++;
    degree[e->v]++;
  }

  FOR_EACH_EDGE_e
    if (degree[e->u] == 1) {
      cplexLP.z[e->u][getArcIndex(e->label, true)].setBounds(1,1);;
	FOR_EACH_NODE_u
	  if (u->label != e->u) {
	    cplexLP.z[u->label][getArcIndex(e->label, true)].setBounds(0,0);
	  }
	}
    } else if (degree[e->v] == 1) {
	cplexLP.z[e->v][getArcIndex(e->label, false)].setBounds(1,1);
	FOR_EACH_NODE_u
	  if (u->label != e->v) {
	    cplexLP.z[u->label][getArcIndex(e->label, false)].setBounds(0,0);
	  }
	}
    }

  }
}

void findBridges(Digraph& digraph, LengthMap& weight, list<Edge*>& bridges, EdgeMap& edgeMap) {

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

  FOR_EACH_ARC_pair
    Digraph::Arc arc = pair->in;
    Digraph::Node s = digraph.source(pair->in);
    Digraph::Node t = digraph.target(pair->in);
    Digraph::Arc arcRef = findArc(tempDigraph, nr[s], nr[t]);
    Digraph::Arc oppArcRef = findArc(tempDigraph, nr[t], nr[s]);
    tempLength[arcRef] = weight[arc];
    tempLength[oppArcRef] = weight[pair->in];
  }

  FOR_EACH_ARC_pair
    IloNum len = weight[pair->in];
    Digraph::Node s = digraph.source(pair->in);
    Digraph::Node t = digraph.target(pair->in);
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
      Edge* edge = edgeMap[pair->in];
      bridges.push_back(edge);
      // cplexLP.x[edge->label].setBounds(1, 1);
    }

    Digraph::Arc e = tempDigraph.addArc(nr[s], nr[t]);
    Digraph::Arc f = tempDigraph.addArc(nr[t], nr[s]);
    tempLength[e] = len;
    tempLength[f] = len;
  }
}


IloNum LB(MinCostArborescence<Digraph, LengthMap> mca, Digraph::Node root) {
   mca.run(root);
   return mca.arborescenceCost();
 }

#ifndef STR_FACTOR_AS_VAR
IloNum UB(MinCostArborescence<Digraph, LengthMap> minca, Digraph::Node root) {
  minca.run(root);
  // return -minca.arborescenceCost();

  double cost = 0;
  
  FOR_EACH_ARC_pair
    Digraph::Arc arcIn = pair->in;
    Digraph::Arc arcOut = pair->out;

    if (minca.arborescence(arcIn) || minca.arborescence(arcOut)) {
      cost += weight[arcIn];
    }
  }
  return cost;
}

int isOdd(int number) {
  if (number % 2 == 0)
    return false;
  else
    return true;
}

void addSolution(Digraph& tree, ReverseNodeMap& treeLabelMap, LengthMap& treeWeight) {
  bool addedXVar[input.nEdges];
  bool addedZVar[input.nNodes][2*input.nEdges];
  IloNumVarArray startXVar(cplexLP.env);
  IloNumVarArray startZVar(cplexLP.env);
  IloNumVarArray startUVar(cplexLP.env);
  IloNumArray startXVal(cplexLP.env);
  IloNumArray startZVal(cplexLP.env);
  IloNumArray startUVal(cplexLP.env);

  ArcLookUp<Digraph> alu(digraph);

  for (Digraph::ArcIt a(tree); a != INVALID; ++a) {
    Node* srcNode = labelNodeMap[treeLabelMap[tree.source(a)]];
    Node* trgNode = labelNodeMap[treeLabelMap[tree.target(a)]];
    Digraph::Node src = *(srcNode->nodeRef);
    Digraph::Node trg = *(trgNode->nodeRef);
    Digraph::Arc arc = alu(src, trg);

    Edge* e = edgeMap[arc];
    startXVar.add(cplexLP.x[e->label]);
    startXVal.add(1);
    cplexLP.x[e->label].setBounds(1,1);
    addedXVar[e->label] = true;
  }

  Dijkstra<Digraph, LengthMap> dijkstra(tree, treeWeight);
  
  for (Digraph::NodeIt u(tree); u != INVALID; ++u) {

    int uLabel = treeLabelMap[u];
    dijkstra.run(u);
    for (Digraph::NodeIt v(tree); v != INVALID; ++v) {
      int vLabel = treeLabelMap[v];
      
      if (v == u) {
	startUVar.add(cplexLP.u[uLabel][vLabel]);
	startUVal.add(0);
	cplexLP.u[uLabel][vLabel].setBounds(0,0);
	continue;
      }
      Digraph::Node nod = v;

      double vDist = dijkstra.dist(v);
      startUVar.add(cplexLP.u[uLabel][vLabel]);
      startUVal.add(vDist);
      cplexLP.u[uLabel][vLabel].setBounds(vDist,vDist);

      while (dijkstra.predArc(nod) != INVALID) {
	  Digraph::Arc a = dijkstra.predArc(nod);

	  Node* srcNode = labelNodeMap[treeLabelMap[tree.source(a)]];
	  Node* trgNode = labelNodeMap[treeLabelMap[tree.target(a)]];
	  Digraph::Node src = *(srcNode->nodeRef);
	  Digraph::Node trg = *(trgNode->nodeRef);
	  Digraph::Arc arc = alu(src, trg); //arco do grafo original

	  Edge* edge = edgeMap[arc];
	  bool flag = false;
	  if (edge->u == nodeLabelMap[digraph.source(arc)])
	    flag = true;
	  startZVar.add(cplexLP.z[uLabel][getArcIndex(edge->label, flag)]);
	  startZVal.add(1);
	  cplexLP.z[uLabel][getArcIndex(edge->label, flag)].setBounds(1,1);
	  addedZVar[uLabel][getArcIndex(edge->label, flag)] = true;
	  
	  nod = tree.source(a);
      }
    }  
  }

  // FOR_EACH_EDGE_e
  //   if (!addedXVar[e->label]) {
  //     startXVar.add(cplexLP.x[e->label]);
  //     startXVal.add(0);
  //   }
    
  //   FOR_EACH_NODE_u
  //     if (!addedZVar[u->label][getArcIndex(e->label, true)]) {
  // 	startZVar.add(cplexLP.y[u->label][getArcIndex(e->label, true)]);
  // 	startZVal.add(0);
  // 	startZVar.add(cplexLP.y[u->label][getArcIndex(e->label, false)]);
  // 	startZVal.add(0);
  //     }
  //   }
  // }

  // cplexLP.solver->addMIPStart(startXVar, startXVal);
  // cplexLP.solver->addMIPStart(startZVar, startZVal);
  // cplexLP.solver->addMIPStart(startUVar, startUVal);
  startXVar.end();
  startXVal.end();
  startZVar.end();
  startZVal.end();
  startUVar.end();
  startUVal.end();  
}

int getMaxRadiusFromSource(Digraph::Node &c1, Digraph::Node &c2, bool consider2ndVert, int tol) {
  Bfs<ListDigraph> bfs(digraph);
  bfs.init();
  bfs.addSource(c1);
  if (consider2ndVert)
    bfs.addSource(c2);
  bfs.start();
  
  int max = -1;
  FOR_EACH_NODE_u
    int dist = bfs.dist(*(u->nodeRef));
    if (dist > max) {
      max = dist;
      if (max > tol)
	return max;
    }
  }
  return max;
}


int UB2UnitWeights2() {
  int radius = floor(input.strFactor / 2);

  FOR_EACH_NODE_u    
    Node* center = u;
    if (isOdd(input.strFactor)) {
      
      for (Digraph::OutArcIt it(digraph, *(u->nodeRef)); it != INVALID; ++it) {
	Digraph::Node v = digraph.target(it);

	if (radius >= getMaxRadiusFromSource(*(u->nodeRef), v, true, radius))
	  return 1;		
	// Bfs<ListDigraph> bfsOdd(digraph);	
	// bfsOdd.init();
	// bfsOdd.addSource(u->nodeRef);
	// bfsOdd.addSource(v);
	// bfsOdd.start();
      }
    } else {
      if (radius >= getMaxRadiusFromSource(*(u->nodeRef), *(u->nodeRef), false, radius))
	return 1;
      // Bfs<ListDigraph> bfs(digraph);
      // bfs.init();
      // bfs.addSource(u->nodeRef);
      // bfs.start();
    }
  }
  return 0;
}

int UB2UnitWeights() {
  int radius = floor(input.strFactor / 2);
  Bfs<ListDigraph> bfs(digraph);

  //ReverseNodeMap treeLabelMap;
  vector<Digraph::Node> tempNodes;
  vector<int> nodeExist;
  tempNodes.reserve(input.nNodes);
  nodeExist.reserve(input.nNodes);

  
  FOR_EACH_NODE_u

    Node* center = u;

    for (Digraph::OutArcIt it(digraph, *(u->nodeRef)); it != INVALID; ++it) {
      Digraph::Node v = digraph.target(it);

      Digraph tempDigraph;
      LengthMap tempWeight(tempDigraph);
      
      for (int i = 0; i < input.nNodes; i++) {
	nodeExist[i] = 0;
      }

      bfs.init();
      bfs.addSource(*(center->nodeRef));
      tempNodes[center->label] = tempDigraph.addNode();
      //treeLabelMap[tempNodes[center->label]] = center->label;
      nodeExist[center->label] = 1;

      if (isOdd(input.strFactor)) {
	bfs.addSource(v);
	nodeExist[nodeLabelMap[v]] = 1;
	tempNodes[nodeLabelMap[v]] = tempDigraph.addNode();
	//treeLabelMap[tempNodes[nodeLabelMap[v]]] = nodeLabelMap[v];
      }
      
      while ( !bfs.emptyQueue() ) {
	Digraph::Node nod = bfs.processNextNode();
	if (bfs.dist(nod) > radius) {
	  break;
	} else if ((bfs.dist(nod) > 0) //first node(s) to be processed is(are) the roots
		 && (!nodeExist[nodeLabelMap[nod]])) {
	  tempNodes[nodeLabelMap[nod]] = tempDigraph.addNode();
	  //treeLabelMap[tempNodes[nodeLabelMap[nod]]] = nodeLabelMap[nod];
	  nodeExist[nodeLabelMap[nod]] = 1;
	  // Digraph::Arc f =  tempDigraph.addArc(tempNodes[nodeLabelMap[nod]], tempNodes[nodeLabelMap[bfs.predNode(nod)]]);
	  // Digraph::Arc g =  tempDigraph.addArc(tempNodes[nodeLabelMap[bfs.predNode(nod)]], tempNodes[nodeLabelMap[nod]]);
	  // tempWeight[f] = 1;
	  // tempWeight[g] = 1;
	}
      }

      if (countNodes(tempDigraph) == input.nNodes) {
	//addSolution(tempDigraph, treeLabelMap, tempWeight);	
	return 1;
      }
      nodeExist.clear();
      tempNodes.clear();
      //treeLabelMap.clear();
          
      if (!isOdd(input.strFactor))
	break;
    }
  }
  return 0;
}

#else
IloNum UB(MinCostArborescence<Digraph, LengthMap> minca, Digraph::Node root) {
  Digraph tempDigraph;
  vector<Digraph::Node> tempNodes;
  LengthMap tempWeight(tempDigraph);
  LengthMap tempLength(tempDigraph);
  
  minca.run(root);
  
  tempNodes.reserve(input.nNodes);
  
  for (int i = 0; i < input.nNodes; i++) {
    tempNodes[i] = tempDigraph.addNode();
  }
  
  FOR_EACH_ARC_pair
    Digraph::Arc arcIn = pair->in;
    Digraph::Arc arcOut = pair->out;

    if (minca.arborescence(arcIn) || minca.arborescence(arcOut)) {
      Edge* e = edgeMap[arcIn];
      Digraph::Arc f =  tempDigraph.addArc(tempNodes[e->u], tempNodes[e->v]);
      Digraph::Arc g =  tempDigraph.addArc(tempNodes[e->v], tempNodes[e->u]);
      tempWeight[f] = e->cost;
      tempWeight[g] = e->cost;
    }
  }

  Dijkstra<Digraph, LengthMap> dijkstra(tempDigraph, tempWeight);
  double maxRatio = 1;
  
  for (int u = 0; u < input.nNodes; u++) {
    dijkstra.run(tempNodes[u]);
    for (int v = (u+1); v < input.nNodes; v++) {
       Digraph::Node nod = tempNodes[v];
       double distVal = dijkstra.dist(nod);
       double ratio = distVal / input.dist[u][v];
       if (ratio > maxRatio)
	 maxRatio = ratio;
    }
  }
  return ceil(maxRatio);
}
#endif

void initCplexLP(IloEnv _env, IloModel* _model, IloCplex* _solver) {
    // cplexLP = (CplexEnv*) malloc(sizeof(CplexEnv));

    cplexLP.env = _env;
    cplexLP.model = _model;
    cplexLP.solver = _solver;

    #ifdef STR_FACTOR_AS_VAR
    cplexLP.lb = 1;
    #else
    if (input.unit)
      cplexLP.lb = input.nNodes - 1;
    else {
      MinCostArborescence<Digraph, LengthMap> mca(digraph,weight);
      cplexLP.lb = LB(mca, *(input.nodes[0]->nodeRef));
    }
    #endif
    //MinCostArborescence<Digraph, LengthMap> minca(digraph,invWeight);
    //cplexLP.ub = UB(minca, *(input.nodes[0]->nodeRef));    
}

 void initVariables(IloEnv _env, IloModel* _model, IloCplex* _solver) {
   initLemonAndInputVariables2();
   //initLemonAndInputVariables();
   calcDistances();
   initCplexLP(_env, _model, _solver);
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

void getPreviousIds(char* expr, int startId, int id, int sign, int flagX, int flagZ, int flagU) {
  //if (flagX) {
    FOR_EACH_EDGE_e
      if ((id >= startId) && id < input.idVarX[e->label]) {
	sprintf(expr, "0 %s", expr);
      }
    }
    if (flagX)
      sprintf(expr, "%s %d", expr, sign * id);
//} else if (flagZ) {
      FOR_EACH_NODE_u
	FOR_EACH_ARC_pair
	if ((id >= startId) && (id < input.idVarZ[u->label][getArcIndex(pair->label, true)]) || (id < input.idVarZ[u->label][getArcIndex(pair->label, false)])) {
	    sprintf(expr, "0 %s", expr);
	  }
	}
      }
      if (flagZ)
        sprintf(expr, "%s %d", expr, sign * id);
//} else { //flagU
    FOR_EACH_NODE_u
      FOR_EACH_NODE_v
	if ((id >= startId) && id < input.idVarU[u->label][v->label]) {
	  sprintf(expr, "0 %s", expr);
	}
      }
    }
    if (flagU)
      sprintf(expr, "%s %d", expr, sign * id);
//}
}

int checkSolution(list<Edge*>& sol) {
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
     EdgeArcs* arcs = input.arcs[e->label];
    
    Digraph::Node s = digraph.source(arcs->in);
    Digraph::Node t = digraph.target(arcs->in);
    
    Digraph::Arc arc = findArc(tempDigraph, nr[s], nr[t]);
    tempDigraph.erase(arc);
    Digraph::Arc arc2 = findArc(tempDigraph, nr[t], nr[s]);
    tempDigraph.erase(arc2);
  }

//adding the arcs to the solution graph
  for (list<Edge*>::iterator it = sol.begin(); it != sol.end(); it++) {
    Edge* edge = *it;
    if (DEBUG == 6)
      cout << "edge: " << edge->u << "-" << edge->v << endl;
    EdgeArcs* arcs = input.arcs[edge->label];
    
    Digraph::Node s = digraph.source(arcs->in);
    Digraph::Node t = digraph.target(arcs->in);

    Digraph::Arc arcIn = tempDigraph.addArc(nr[s], nr[t]);
    Digraph::Arc arcOut = tempDigraph.addArc(nr[t], nr[s]);
    IloNum len = edge->cost;
    tempLength[arcIn] = len;
    tempLength[arcOut] = len;
  }

//checking if, for each edge, the spanner property is valid
  FOR_EACH_EDGE_e
    EdgeArcs* arcs = input.arcs[e->label];
    Digraph::Node s = digraph.source(arcs->in);
    Digraph::Node t = digraph.target(arcs->in);
  
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

void getSolution(list<Edge*>& solution) {
  IloNumArray sol(cplexLP.env, input.nEdges);

  sol = IloNumArray(cplexLP.env);
  cplexLP.solver->getValues(sol, cplexLP.x);
  
  FOR_EACH_EDGE_e
    if ( sol[e->label] > cplexLP.tol ) {
      solution.push_back(e);
    }
    
  }
}


void createLPFormulation(IloEnv env) {
  if (DEBUG == 14)
    cout << "dentro de createLPFormulation" << endl;
  int j, l, k, a;
  char varName[100];
  IloRangeArray cons(cplexLP.env);
  int consCounter = 0;

  #ifdef RELAX_LP
  cplexLP.x = IloNumVarArray(cplexLP.env, input.nEdges, 0, 1, ILOFLOAT);
  #else
  cplexLP.x = IloNumVarArray(cplexLP.env, input.nEdges, 0, 1, ILOINT);
  #endif
  //cplexLP.x = IloNumVarArray(cplexLP.env, input.nEdges, 0, 1, ILOFLOAT);
  cplexLP.z = IloNumVarArray2(cplexLP.env, input.nNodes);
  cplexLP.u = IloNumVarArray2(cplexLP.env, input.nNodes);
    
  #ifdef STR_FACTOR_AS_VAR
  #ifdef RELAX_LP
  cplexLP.t = IloNumVar(cplexLP.env, 1, IloInfinity, ILOFLOAT);
  #else
  cplexLP.t = IloNumVar(cplexLP.env, 1, IloInfinity, ILOINT);
  #endif  
  #endif
  
  FOR_EACH_NODE_u
    #ifdef RELAX_LP
    cplexLP.z[u->label] = IloNumVarArray(cplexLP.env, 2*input.nEdges, 0, 1, ILOFLOAT);
  #else
    cplexLP.z[u->label] = IloNumVarArray(cplexLP.env, 2*input.nEdges, 0, 1, ILOINT);
    #endif
    cplexLP.u[u->label] = IloNumVarArray(cplexLP.env, input.nNodes, 0, IloInfinity, ILOFLOAT);
  }

  FOR_EACH_NODE_u
    FOR_EACH_ARC_pair
      int source = nodeLabelMap[digraph.source(pair->in)];
      int tar = nodeLabelMap[digraph.target(pair->in)];
      Edge* edge = edgeMap[pair->in];

      sprintf(varName, "z.%d.%d_%d", u->label, source, tar);
      cplexLP.z[u->label][getArcIndex(edge->label, true)].setName(varName);
      sprintf(varName, "z.%d.%d_%d", u->label, tar, source);
      cplexLP.z[u->label][getArcIndex(edge->label, false)].setName(varName);
    }
    cplexLP.model->add(cplexLP.z[u->label]);
  }

  FOR_EACH_EDGE_e
    sprintf(varName, "x.%d_%d",  e->u, e->v);
    cplexLP.x[e->label].setName(varName);
  }

  FOR_EACH_NODE_u
    FOR_EACH_NODE_v
      sprintf(varName, "u.%d.%d", u->label, v->label);
      cplexLP.u[u->label][v->label].setName(varName);

    }
    cplexLP.model->add(cplexLP.u[u->label]);
  }

  cplexLP.model->add(cplexLP.x);

  IloExpr minExpr(cplexLP.env);
  IloExpr nEdgesExpr(cplexLP.env);

  #ifdef STR_FACTOR_AS_VAR
  minExpr = cplexLP.t;
  #endif

  FOR_EACH_EDGE_e
    #ifndef STR_FACTOR_AS_VAR
    if (!input.unit)
      minExpr += cplexLP.x[e->label] * e->cost;
    else {
      #ifdef RELAX_LP
      minExpr += cplexLP.x[e->label] * e->cost;
      #endif
    }
    #endif
    nEdgesExpr += cplexLP.x[e->label];

    IloExpr spanExpr(cplexLP.env);

    cplexLP.model->add(cplexLP.u[e->v][e->u] == cplexLP.u[e->u][e->v]);

    #ifdef STR_FACTOR_AS_VAR
    IloExpr spanExpr2(cplexLP.env);
    spanExpr2 = cplexLP.t * e->cost;
    //cons.add(cplexLP.u[e->u][e->v] <= spanExpr2);
    cplexLP.model->add(-cplexLP.u[e->u][e->v] + spanExpr2 /** dimVar*/ >= 0);
    spanExpr2.end();
    #else
    // cons.add(cplexLP.u[e->u][e->v] <= input.strFactor * e->cost);
    // cons[consCounter++].setName("uUB");
    #endif
  }

  #ifdef STR_FACTOR_AS_VAR
  cplexLP.model->add(IloMinimize(cplexLP.env, minExpr));
  #else
  if (!input.unit)
    cplexLP.model->add(IloMinimize(cplexLP.env, minExpr));
  else {
    #ifdef RELAX_LP
    cplexLP.model->add(IloMinimize(cplexLP.env, minExpr));
    #endif
  }
  #endif
  cons.add(nEdgesExpr - (input.nNodes - 1)/**cplexLP.dimVar*/ == 0);
  cons[consCounter++].setName("numEdges");

  minExpr.end();
  nEdgesExpr.end();

  FOR_EACH_NODE_u  
    int l1 = u->label;
    int pos = 1;
    IloExpr rootInDegree(cplexLP.env);	  
    FOR_EACH_ARC_pair    
      int l2 = nodeLabelMap[digraph.target(pair->in)];
      Edge* edge = edgeMap[pair->in];

      IloExpr zAndx(cplexLP.env);
      zAndx = cplexLP.z[u->label][getArcIndex(edge->label, true)] +
      cplexLP.z[u->label][getArcIndex(edge->label, false)];
cplexLP.model->add(zAndx == cplexLP.x[edge->label]);
zAndx.end();

      //pair->in é um arco de entrada em l1
      if (l1 == l2) { 
	rootInDegree += cplexLP.z[l1][getArcIndex(edge->label, true)];
      } else {
	l2 = nodeLabelMap[digraph.target(pair->out)];
	if (l1 == l2) { //pair->out corresponde a um arco que entra em l1
	  edge = edgeMap[pair->out];
	  rootInDegree += cplexLP.z[l1][getArcIndex(edge->label, false)];  
	}
      }
    }
    cons.add(rootInDegree == 0);
    cons[consCounter++].setName("rootInDegree");

    FOR_EACH_NODE_v
      int l2 = v->label;
	if (l1 != l2) { //excluindo o caso do root
	IloExpr inDegree(cplexLP.env);
	FOR_EACH_ARC_pair
	  int l3 = nodeLabelMap[digraph.target(pair->in)];
	  Edge* edge = edgeMap[pair->in];
	  if (l2 == l3) { //pair->in é um arco de entrada em l2
	    inDegree += cplexLP.z[l1][getArcIndex(edge->label, true)];
	  } else {
	    l3 = nodeLabelMap[digraph.target(pair->out)];
	    if (l2 == l3) { //pair->out é um arco de entrada em l2
	      edge = edgeMap[pair->out];
	      inDegree += cplexLP.z[l1][getArcIndex(edge->label, false)];
	    }
	  }
	}
	cons.add(inDegree - 1 /** cplexLP.dimVar*/ == 0);
	cons[consCounter++].setName("inDegree");
      }
    }
  }

  FOR_EACH_NODE_r
    cons.add(cplexLP.u[r->label][r->label] == 0);
    cons[consCounter++].setName("root2RootLabel");

    FOR_EACH_NODE_u
    if (u->label != r->label) {
      //restrição extra
      cons.add(0 <= cplexLP.u[r->label][u->label] - input.dist[r->label][u->label] /** cplexLP.dimVar*/);
      cons[consCounter++].setName("uLB");//desnecessário?

	#ifdef STR_FACTOR_AS_VAR
	cplexLP.model->add(cplexLP.u[r->label][u->label] <= cplexLP.t * input.dist[r->label][u->label]);
	// cons.add(cplexLP.u[r->label][u->label] <= cplexLP.t * input.dist[r->label][u->label]);
	#else
	cons.add(cplexLP.u[r->label][u->label] - (input.strFactor * input.dist[r->label][u->label]) /** cplexLP.dimVar*/ <= 0);
	cons[consCounter++].setName("uUB");//desnecessário?
	#endif
      }
    }

    FOR_EACH_ARC_pair
      Edge* edge = edgeMap[pair->in];
      if (input.unit == true) {
	if (edge->u == r->label) {
	  /*A seguinte inequação força que uma das duas situações ocorra:
	   (a): a aresta uv faz parte da solução;
	   (b): o caminho entre r e v será de tamanho pelo menos 2 (u^r_v >= 2)*/
	  cons.add(cplexLP.u[r->label][edge->v] + cplexLP.z[r->label][getArcIndex(edge->label, true)] - 2 /** cplexLP.dimVar*/ >= 0);
	  cons[consCounter++].setName("uAndzIneq");	
	}
      }

      //checking for the pair->in arc;
      IloExpr MTZineqIn(cplexLP.env);
      if (edge->v != r->label) {
	double bound = getMij(edge);
	MTZineqIn = cplexLP.u[r->label][edge->u] - 
			     cplexLP.u[r->label][edge->v] +
			     (bound + edge->cost) * cplexLP.z[r->label][getArcIndex(edge->label, true)] +
			     (bound - edge->cost) * cplexLP.z[r->label][getArcIndex(edge->label, false)];
        cons.add(- MTZineqIn + bound /** cplexLP.dimVar*/ >= 0);
        cons[consCounter++].setName("MTZineqIn");	
      }
      MTZineqIn.end();

      //checking for the pair->out arc;
      IloExpr MTZineqOut(cplexLP.env);
      if (edge->u != r->label) {
	double bound = getMij(edge);
	MTZineqOut = cplexLP.u[r->label][edge->v] - 
			     cplexLP.u[r->label][edge->u] +
			     (bound + edge->cost) * cplexLP.z[r->label][getArcIndex(edge->label, false)] +
	  (bound - edge->cost) * cplexLP.z[r->label][getArcIndex(edge->label, true)];
        cons.add(- MTZineqOut + bound /** cplexLP.dimVar*/ >= 0);
        cons[consCounter++].setName("MTZineqOut");	
      }
      MTZineqOut.end();

      IloExpr MTZRootIneq(cplexLP.env);
      if (edge->u == r->label) {
	double bound = getMij(edge);	
	MTZRootIneq = cplexLP.u[r->label][edge->v] + 
	  (bound - edge->cost) * cplexLP.z[r->label][getArcIndex(edge->label, true)];
        cons.add(- MTZRootIneq + bound /** cplexLP.dimVar*/ >= 0);
        cons[consCounter++].setName("MTZRootIneq");	
      } else if (edge->v == r->label) {
	double bound = getMij(edge);	
	MTZRootIneq = cplexLP.u[r->label][edge->u] + 
	  (bound - edge->cost) * cplexLP.z[r->label][getArcIndex(edge->label, false)];
        cons.add(- MTZRootIneq + bound /** cplexLP.dimVar*/ >= 0);
        cons[consCounter++].setName("MTZRootIneq");		
      }
      MTZRootIneq.end();
    }
  }

  cplexLP.model->add(cons);
}

double getMij(Edge* e) {

  if (input.unit) {
    return input.nNodes - 1;
  } else {
    #ifdef STR_FACTOR_AS_VAR
    return input.nEdges * e->cost;
    #else
    return input.strFactor * e->cost;
    #endif
  }
}

int getArcIndex(int edgeLabel, bool in) {
  if (in)
    return edgeLabel * 2;
  else
    return edgeLabel * 2 + 1;
}


int getNEdges() {
  int count = 0;
   for (int u = 0; u < input.nNodes; u++) {
     for (int v = (u+1); v < input.nNodes; v++) {
       if (input.edgeCost[u][v] != NO_EDGE_CONST) {
	 count++;
       }
     }
   }
   return count;
}

void setCplexParam() {
  // cplexLP.solver->setParam(IloCplex::PreInd, IloFalse);
  // cplexLP.solver->setParam(IloCplex::Param::Preprocessing::Presolve, IloFalse);
  
  cplexLP.tol = cplexLP.solver->getParam(IloCplex::EpInt);
  // cplexLP.solver->setParam(IloCplex::CutLo, cplexLP.lb);
  cplexLP.solver->setParam(IloCplex::TiLim, TIME_LIMIT);
  //1 - cpu time
  //2 - physical elapsed time 
  cplexLP.solver->setParam(IloCplex::ClockType, 1);  
  cplexLP.solver->setParam(IloCplex::Threads, 1);
  
  //cplexLP.solver->setOut("log.txt");
  //cplexLP.solver->setWarning("warning.txt");
  cplexLP.solver->exportModel("model.lp");
  //cplexLP.solver->setParam(IloCplex::Param::RootAlgorithm, 1);

  // cplexLP.solver->setOut(cplexLP.env.getNullStream());
}

void readInput(int argc, char **argv, IloEnv env) {

  ifstream costs(argv[1]);

  input.edgeCost = IloNumArray2(env);
  costs >> input.edgeCost;
  costs.close();
  input.strFactor = atof(argv[2]);
  input.nNodes = input.edgeCost.getSize();
  input.nEdges = getNEdges();

  input.unit = true;
  for (int i = 0; i < input.nNodes; i++) {
    for (int j = (i+1); j < input.nNodes; j++) {
      if ((input.edgeCost[i][j] > 1) && (input.edgeCost[i][j] != NO_EDGE_CONST))
	input.unit = false;
    }
  }
}

int word_count(std::string str)  // can pass an open std::ifstream() to this if required
{
	istringstream iss(str);
	int c = 0;
	for(std::string w; iss >> w; ++c);
	return c;
}

// ****** File format ********
// nNodes nEdges strFactor
// u v cost, for each uv in E
// ***************************
double readInput2(char *fileName, char *strFactor) {
  std::ifstream infile(fileName);
  if (infile.fail())
    return -1;
  std::string line;
  getline(infile, line);
  istringstream issHead(line);
  //cout << "words: " << word_count(line) << endl;
  issHead >> input.nNodes >> input.nEdges;
  input.strFactor = atof(strFactor);
  int label = 0;
  input.unit = true;
  input.edges.reserve(input.nEdges);
  double sum = 0;
  for (int i = 0; i < input.nEdges; i++) {
    getline(infile, line);
    Edge* edge = new Edge();
    edge->label = i;
    istringstream iss(line);
    if (!(iss >> edge->u >> edge->v >> edge->cost)) { break; }
    sum += edge->cost;
    if (edge->cost > 1)
      input.unit = false;
    input.edges[edge->label] = edge;
  }

  input.dim = input.nEdges + (input.nNodes * (2 * input.nEdges)) + (input.nNodes * input.nNodes);

  return sum;
}

void freeMemoryCloseVar() {
  cplexLP.env.end();
  // free (cplexLP);
  
  //deleting Node()
  for (map<int,Node*>::iterator it=labelNodeMap.begin(); it!=labelNodeMap.end(); ++it) {
    delete it->second;
  }

  for (int i = 0; i < input.nEdges; i++) {
    delete input.arcs[i];	//deleting EdgeArcs()
    delete input.edges[i];	//deleting Edge()
  }
  
  // deleting dist matrix
  for (int i = 0; i < input.nNodes; i++) {
    delete input.dist[i];
    
    // delete input.idVarU[i];
    // delete input.idVarZ[i];
  }
  delete input.dist;

  // delete input.idVarX;
  // delete input.idVarZ;
  // delete input.idVarU;  
}

bool isSolutionConnected() {
  Digraph tempDigraph;
  vector<Digraph::Node> tempNodes;
  LengthMap tempWeight(tempDigraph);
  LengthMap tempLength(tempDigraph);

  IloNumArray sol(cplexLP.env, input.nEdges);
  sol = IloNumArray(cplexLP.env);
  cplexLP.solver->getValues(sol, cplexLP.x);  
  
  tempNodes.reserve(input.nNodes);
  
  for (int i = 0; i < input.nNodes; i++) {
    tempNodes[i] = tempDigraph.addNode();
  }

  FOR_EACH_EDGE_e
    if ( sol[e->label] > 1e-03 ) {
      Digraph::Arc f =  tempDigraph.addArc(tempNodes[e->u], tempNodes[e->v]);
      Digraph::Arc g =  tempDigraph.addArc(tempNodes[e->v], tempNodes[e->u]);
      tempWeight[f] = e->cost;
      tempWeight[g] = e->cost;      
    }    
  }

  if (stronglyConnected(tempDigraph)) {
    Dijkstra<Digraph, LengthMap> dijkstra(tempDigraph, tempWeight);

    cout << "Arvore Solucao:" << endl;
    for (int u = 0; u < input.nNodes; u++) {
      dijkstra.run(tempNodes[u]);
      for (int v = (u+1); v < input.nNodes; v++) {
	 Digraph::Node nod = tempNodes[v];
	 IloNum distVal = dijkstra.dist(nod);
	 cout << "globaldist[" << u << "][" << v << "]: " << input.dist[u][v] << endl;
	 cout << "dist[" << u << "][" << v << "]: " << distVal << endl;
      }
    }
    
    return true;
  } else
    return false;  
}

void printSolution(bool xFlag, bool zFlag, bool uFlag, bool printValue) {
  IloNumArray sol(cplexLP.env, input.nEdges);
  IloNumArray2 zSol(cplexLP.env, input.nNodes);
  IloNumArray2 uSol(cplexLP.env, input.nNodes);
  
  sol = IloNumArray(cplexLP.env);
  cplexLP.solver->getValues(sol, cplexLP.x);

  for (int i = 0; i < input.nNodes; i++) {
    zSol[i] = IloNumArray(cplexLP.env);
    cplexLP.solver->getValues(zSol[i], cplexLP.z[i]);

    uSol[i] = IloNumArray(cplexLP.env);
    cplexLP.solver->getValues(uSol[i], cplexLP.u[i]);    
  }

  if (xFlag) {
    for (int i = 0; i < input.nEdges; i++) {
      if ( sol[i] > 1e-03 ) {
	cout << cplexLP.x[i].getName();
	if (printValue)
	  cout << ": " << sol[i] << endl;
	else
	  cout << endl;	
      }
    }    
  }

  if (zFlag) {
    for (int i = 0; i < input.nNodes; i++) {
      for (int j = 0; j < 2*input.nEdges; j++) {
	if ( zSol[i][j] > 1e-03 ) {
	  cout << cplexLP.z[i][j].getName();
	  if (printValue)
	    cout << ": " << zSol[i][j] << endl;
	  else
	    cout << endl;	  
	}
      }
    }
  }

  if (uFlag) {
    for (int i = 0; i < input.nNodes; i++) {
      for (int j = 0; j < input.nNodes; j++) {
	if ( uSol[i][j] > 1e-03 ) {
	  cout << cplexLP.u[i][j].getName();
	  if (printValue)
	    cout << ": " << uSol[i][j] << endl;
	  else
	    cout << endl;		  
	}
      }
    }
  }
}

void printVariableNames() {

  FOR_EACH_EDGE_e 
    cout << "\"" << cplexLP.x[e->label].getName() << "\", ";
  }

  FOR_EACH_NODE_u
    FOR_EACH_ARC_pair
     int source = nodeLabelMap[digraph.source(pair->in)];
     int tar = nodeLabelMap[digraph.target(pair->in)];
     Edge* edge = edgeMap[pair->in];
     cout << "\"" << cplexLP.z[u->label][getArcIndex(edge->label, true)].getName() << "\", ";
     cout << "\"" << cplexLP.z[u->label][getArcIndex(edge->label, false)].getName() << "\", ";
    }
  }

  FOR_EACH_NODE_u
    FOR_EACH_NODE_v
cout << "\"" << cplexLP.u[u->label][v->label].getName() << "\", ";
    }
  }

  cout << endl;

  int id = 1;
  FOR_EACH_EDGE_e 
    cout << "\"x" << id++ << "\", ";
  }

  FOR_EACH_NODE_u
    FOR_EACH_ARC_pair
     cout << "\"x" << id++ << "\", ";
     cout << "\"x" << id++ << "\", ";
    }
  }

  FOR_EACH_NODE_u
    FOR_EACH_NODE_v
      cout << "\"x" << id++ << "\", ";
    }
  }

}

void printConstraints() {
  int nRows = cplexLP.solver->getNrows();
  cout << "Building constraint list ... " << flush << endl; 
  for (IloModel::Iterator it(cplexLP.solver->getModel()); it.ok(); ++it)
  { 
    IloExtractable e = *it;

    if ((e.isConstraint())) {
      //cout << "cons: " << e.asConstraint() << endl;
      IloConstraint con = e.asConstraint();
      if (con.getName() != NULL) {
	std::string str(con.getName());
	if (
	    (str.find("spanExpr2") !=std::string::npos)  ||
	    (str.find("LBExpr") !=std::string::npos)  ||
	    (str.find("numEdges") !=std::string::npos)  ||
	    (str.find("rootInDegree") !=std::string::npos)  ||
	    (str.find("inDegree") !=std::string::npos) ||
	    (str.find("root2RootLabel") !=std::string::npos) ||
	    (str.find("uLB") !=std::string::npos) ||
	    (str.find("uUB") !=std::string::npos) ||
	    (str.find("uAndzIneq") !=std::string::npos) ||
	    (str.find("MTZineqIn") !=std::string::npos) ||
	    (str.find("MTZineqOut") !=std::string::npos) ||
	    (str.find("MTZRootIneq") !=std::string::npos) ||
	    (str.find("MTZRootIneq") !=std::string::npos))
	  cout << "cons: " << e.asConstraint() << endl;	
      }
    } 
  }

  cout << "chegou ao fim de printConstraints" << endl;
}


void printViolatedConstraints() {
  // Perform conflict analysis. 
  // We allow any constraint to be part of the conflict. 

  // Build a list of constraints that can be part of the conflict. 
  // Since bounds of variables may also render the model infeasible 
  // we also add bounds to the set of constraints that are considered 
  // for a conflict. 
  cout << "Building constraint list ... " << flush; 
  IloConstraintArray cons(cplexLP.env); 
  IloNumArray prefs(cplexLP.env); 
  for (IloModel::Iterator it(cplexLP.solver->getModel()); it.ok(); ++it)
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
      cout << "cons: " << e.asConstraint() << endl;
      cons.add(e.asConstraint()); 
      prefs.add(1.0); 
    } 
  } 
  cout << cons.getSize() << " elements." << endl; 
  // Refine the conflict. std::cout << "Refine the conflict ..." << std::endl; 
  if ( !cplexLP.solver->refineConflict(cons, prefs) ) { 
    cout << "No conflict found!" << endl; 
  } else { 
    // Print out minimal conflict. 
    cout << "Conflict found. Minimal conflict is:" << endl; 
    int count = 0; 
    for (IloInt i = 0; i < cons.getSize(); ++i) { 
      if (cplexLP.solver->getConflict(cons[i]) == IloCplex::ConflictMember) { 
	cout << " " << cons[i] << endl; 
	++count; 
      } 
    } 
    cout << count << " constraints in minimal conflict." << endl; 
  }   
}


ILOINCUMBENTCALLBACK0(IncumbentCallback/*, bool& incumbentFlag*/) {

  // incumbentFlag = true;
  cout << "dentro de incumbent" << endl;
  cout << "fim de incumbent" << endl;
}


// Spend at least timeLimit sec. on optimization, but once
// this limit is reached, quit as soon as the solution is acceptable

ILOMIPINFOCALLBACK0(checkHasIncumbentCallback)
{
  if (DEBUG == 16)
    cout << "entrou no callback" << endl;
  // double timeUsed = cplex.getCplexTime() - timeStart;
  if (hasIncumbent())
    incumbentFlag = true;


  // try{
    IloNum objval = getIncumbentObjValue();
    // if (objval == IloInfinity)
    //   cout << "objval eh infinito" << endl;
    // else
      cout << "objval: " << objval << endl;
  // } catch (const IloException& e) {
  //   cout << "deu excessão" << endl;
  }
   
    IloNum bound  = getBestObjValue();
    
    cout << "bound: " << bound << endl;
    cout << "relative Gap: " << cplexLP.solver->getMIPRelativeGap() << endl;
   
}


int
main(int argc, char **argv)
{
  time_t start, end;
  double total_time;
  start = clock();
  // IloNum start2, end2;
  // double total_time2;
  
  ofstream timeOutFile;
  int notOptimalFlag = 0;
  int boundUsed = 0;
  
  IloEnv env;
  IloModel mod(env, "MWSP");
  IloCplex solver(mod);
  double objValue = 0;
  //start2=solver.getTime();

  // double relGap = -1;
  
  bool foundSol = false;

  ofstream modelStatsOutFile;
  char statsStr[100] = "";
  
  resetTime();
  
   try {
     //readInput(argc, argv, env);
     if (readInput2(argv[1], argv[2]) == -1) {
       cout << "falhou para abrir arquivo" << endl;
       return -1;
     }
     initVariables(env, &mod, &solver);     
     if (input.unit && BOUND_FLAG) {
       //boundUsed = UB2UnitWeights();
       boundUsed = UB2UnitWeights2();
     }
     if (boundUsed) {
       end = clock();
     } else {
       createLPFormulation(env);
       setCplexParam();
       //printConstraints();
       // printVariableNames();
       #ifndef STR_FACTOR_AS_VAR
       list<Edge*> bridgeSet = list<Edge*>();
       findBridges(digraph, weight, bridgeSet, edgeMap);

       for (list<Edge*>::iterator it=bridgeSet.begin(); it != bridgeSet.end(); it++) {
	 Edge* edge = *it;
	 cplexLP.x[edge->label].setBounds(1, 1);
       }
       
       #endif
       findOneDegreeVerticesAndBoundZ();
       //cout << "valor do bound2: " << UB2UnitWeights() << endl;
       if (DEBUG == 15)
	 cout << "vai chamar solve()" << endl;

      // IloCplex::Callback sec3 = cplexLP.solver->use(
      // 						    IncumbentCallback(env/*, foundSol*/));

       if (DEBUG == 15)
	 cout << "antes de adicionar callback" << endl;
       IloCplex::Callback sec = cplexLP.solver->use(checkHasIncumbentCallback(cplexLP.env));

       if (DEBUG == 16) {
	 cout << "nColumns: " << cplexLP.solver->getNcols() - 1 << endl;
	 cout << "nRows: " << cplexLP.solver->getNrows() << endl;
	 cout << "non zero coefficients: " << cplexLP.solver->getNNZs() << endl;
       }
       #ifdef GET_MODEL_STATS
       modelStatsOutFile.open (argv[3], ios::out | ios::app);
       modelStatsOutFile << argv[4] << " " << (cplexLP.solver->getNcols() - 1) << " " << cplexLP.solver->getNrows() << " " << cplexLP.solver->getNNZs() << endl;
       exit(0);
       #endif
       
       #ifndef RELAX_LP
       sprintf(statsStr, " %d %d %d",  (cplexLP.solver->getNcols() - 1), cplexLP.solver->getNrows(), cplexLP.solver->getNNZs());
       #endif
       
       
       if ( cplexLP.solver->solve() ) {
	 end = clock();
	 //end2 = cplexLP.solver->getTime();

	 IloAlgorithm::Status solStatus= cplexLP.solver->getStatus();
	 // cout << "solStatus: " << solStatus << endl;

	 if ( solStatus == IloAlgorithm::Optimal ) {

	   cout << "Solution is optimal" << endl;
	   env.out() << "Objective value: "
			   << cplexLP.solver->getObjValue() << endl;
	   objValue = cplexLP.solver->getObjValue();

	   list<Edge*> edgesSol;
	   edgesSol = list<Edge*>();

	   // getSolution(edgesSol);
	   // if (checkSolution(edgesSol))
	   //   cout << "solução é spanner" << endl;
	   // else
	   //   cout << "solução não é spanner" << endl;

	   // printSolution(true, false, false, true);	   
	   // IloNumArray sol(cplexLP.env, input.nEdges);
	   // cplexLP.solver->getValues(sol, cplexLP.x);
	   // IloNum tol = cplexLP.solver->getParam(IloCplex::EpInt);
	   // separate(sol, tol, input.nNodes, input.nEdges);
	   
	   // if (!isSolutionConnected())
	   //   cout << "Grafo final nao eh conexo" << endl;

	 } /*else if ( solStatus == IloAlgorithm::Feasible ) {
	   incumbentFlag = true;
	   // relGap = 100.0 * cplexLP.solver->getMIPRelativeGap();	   
	   }*/else {
	   cout << "Solution status is not Optimal nor Feasible" << endl;
	 }
       } else {//time_limit reached or no feasible solution found
	 // cout << "vou abortar" << endl;
	 total_time = getSpentTime();
	 notOptimalFlag = 1;
	 end = clock();
	 cout << "Not solved" << endl;	 

	 if (1 == 0)
	   printViolatedConstraints();      
       }

       sec.end();
       // sec3.end();
     }

     // Close the environments
     // freeMemoryCloseVar();
     
   } catch (const IloException& e) {
      cerr << "Exception caught: " << e << endl;
   }
   catch (...) {
      cerr << "Unknown exception caught!" << endl;
   } 

   if (DEBUG == 15)
     cout << "antes de freeMemoryCloseVar" << endl;
   // Close the environments
   freeMemoryCloseVar();   
   if (DEBUG == 15)
     cout << "depois de freeMemoryCloseVar" << endl;
   
   //env.end();

   double optSolution = 0;
   //armazenando estatísticas
   // total_time = (double)( end - start )/(double)CLOCKS_PER_SEC ;
   total_time = getSpentTime();
   //total_time2 = (double)( end2 - start2 );
   cout << "total time: " << total_time << endl;
   //cout << "total time2: " << total_time2 << endl;;
     if (DEBUG == 15)
       cout << "antes de abrir arquivo()" << endl;;
   timeOutFile.open (argv[3], ios::out | ios::app);
     if (DEBUG == 15)
       cout << "depois de abrir arquivo()" << endl;
   //timeOutFile.open (argv[3], ios::out);
   //cout << argv[3] << endl;

   if (total_time >= TIME_LIMIT) {
     if (DEBUG == 15)
       cout << "antes de getMIPRelativeGap()" << endl;;
     double relGap;
     if (incumbentFlag) {
       relGap = 100.0 * cplexLP.solver->getMIPRelativeGap();
     } else {
       relGap = NO_INTEGRALITY_GAP;
     }

     // try {
       
     //   relGap = 100.0 * cplexLP.solver->getMIPRelativeGap();
     // } catch (const IloException& e) {
     //   relGap = NO_INTEGRALITY_GAP;
     // }
     // if (incumbentFlag)
     //   relGap = 100.0 * cplexLP.solver->getMIPRelativeGap();
     // else
     //   relGap = NO_INTEGRALITY_GAP;
     if (DEBUG == 15)
       cout << "depois de getMIPRelativeGap()" << endl;
     timeOutFile << argv[4] << " " << TIME_LIMIT_CONST << " " << relGap << statsStr << endl;
   } else if (notOptimalFlag) {     
     timeOutFile << argv[4] << " " << NO_SOLUTION_CONST << " " << total_time << statsStr << endl;
   } else {
     // if (incumbentFlag)
     //   cout << "achou incumbent" << endl;
     // else
     //   cout << "nao achou incumbent" << endl;
     
     //optSolution = 1;
     //optSolution = cplexLP.solver->getObjValue();
     if (input.unit)
       timeOutFile << argv[4] << " " << (boundUsed ? UNIT_BOUND_USED : UNIT_BOUND_UNUSED) << " " << total_time << statsStr << endl;
     else
       timeOutFile << argv[4] << " " << objValue << " " << total_time << statsStr << endl;
   }
   timeOutFile.close();

   if (input.unit)
     return (boundUsed ? UNIT_BOUND_USED : UNIT_BOUND_UNUSED);
   else {
     if (objValue > 0)
       return 1;
     else
       return 0;
   }
} // END main
