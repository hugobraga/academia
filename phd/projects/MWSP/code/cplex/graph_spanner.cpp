#include "graph_spanner.h"

#include <ilcplex/ilocplex.h>
#include <string>
#include <cmath>
#include <vector>
#include <bitset>

#include <lemon/list_graph.h>
#include <lemon/connectivity.h>
// #include <lemon/concepts/maps.h>
// #include <lemon/maps.h>
#include <lemon/smart_graph.h>
#include <lemon/preflow.h>
#include <lemon/edmonds_karp.h>
#include <lemon/min_cost_arborescence.h>

//minimum cut undirected graph
// #include <lemon/nagamochi_ibaraki.h>
// #include <lemon/lgf_reader.h>
#include <lemon/dijkstra.h>
#include <lemon/path.h>

#include <stdio.h>
#include <stdlib.h>

#include <algorithm> //sort

#include <lemon/adaptors.h>

#include <time.h>

ILOSTLBEGIN

using namespace std;
using namespace lemon;
//using namespace lemon::concepts;

//#define FLOAT_VAR 1

namespace {
  //unsigned const NUM_NODES = 32;
  unsigned const SET_CONS_NAMES = 1;
  unsigned const SET_VAR_NAMES = 1;  
  unsigned const NO_EDGE_CONST = 999999;
  unsigned const DEBUG = 21;
};

//----------------------
double maxNumber;			//sum of edge costs plus 1
CplexEnv cplexLP;
InputData input;
clock_t initialClock;
Digraph digraph;			//represents the input graph
LengthMap weight(digraph);		//weight associated with the input graph

vector<Digraph::Node> digraphNodes;	//vector of nodes of the input graph
EdgeMap edgeMap;			//map from the arc (both directions) to the edge that represents the arc

int globalId = 0;			//para teste
//----------------------

map<int,int> globalCutMap;
int branchCount = 0;
IloCplex::MIPCallbackI::NodeId nodeId;

// Declarations for functions in this program

void initializeEdge(OurEdge* edge, int u, int v, IloNum lab, IloNum cost);
IloNum readInput(char *fileName, char *strFactor);
void calcDistances();
void initMasterLP(IloEnv _env, IloModel* _model, IloCplex* _solver);
void initLemonAndInputVariables();
void findBridges(list<OurEdge*>& removedEdges, list<OurEdge*>& bridges);
void setCplexParam();
IloNum LB(MinCostArborescence<Digraph, LengthMap> mca, Digraph::Node root);
void findOneDegreeVerticesAndBoundY();
IloNum greedySpanner(list<OurEdge*>& fixedEdges, list<OurEdge*>& solution);
IloInt separate(IloExpr* expr, IloInt s, IloInt t, IloInt edge, IloNumArray2 ySol);
void printSolution(IloNumArray& xSol, IloNumArray2& sol);
void printLPSolution(bool xFlag, bool yFlag, bool value);
void createILPFormulation();
void deleteObjects(IloEnv* env, IloModel* model, IloCplex* solver);
void printConstraints();

void resetTime() {
    initialClock = clock();
}

float getTime() {
  return (float) (clock()) / CLOCKS_PER_SEC;
}

float getSpentTime() {
    return (float)(clock() - initialClock) / CLOCKS_PER_SEC;
}

//----------compare functions-------
//ordena as arestas pelo custo
bool compareEdgesCost (OurEdge* e, OurEdge* f) { return (e->cost < f->cost); }
//-----------------------------

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
	if (DEBUG == 28)
	  cout << "leu aresta: " << u << "-" << v << endl;
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


void initMasterLP(IloEnv _env, IloModel* _model, IloCplex* _solver) {
    cplexLP.env = _env;
    cplexLP.model = _model;
    cplexLP.solver = _solver;
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
  cplexLP.solver->setParam(IloCplex::PreInd, IloFalse);
  cplexLP.solver->setParam(IloCplex::Param::Preprocessing::Presolve, IloFalse);
  //IloNum tol = cplexLP.solver->getParam(IloCplex::EpInt);
  cplexLP.tol = cplexLP.solver->getParam(IloCplex::EpInt);
  //cplexLP.solver->setParam(IloCplex::CutLo, cplexLP.lb);
  cplexLP.solver->setParam(IloCplex::TiLim, TIME_LIMIT);
  //1 - cpu time
  //2 - physical elapsed time 
  cplexLP.solver->setParam(IloCplex::ClockType, 1);
  cplexLP.solver->setParam(IloCplex::Threads, 1);
  //cplexLP.solver->setOut("log.txt");
  //cplexLP.solver->setWarning("warning.txt");
  cplexLP.solver->exportModel("model.lp");
  //cplexLP.solver->setParam(IloCplex::Param::RootAlgorithm, 1);

  // cplexLP.solver->setParam(IloCplex::MIPDisplay, 0);
  // cplexLP.solver->setParam(IloCplex::MIPInterval, 1500);
  
  cplexLP.solver->setOut(cplexLP.env.getNullStream());
}


IloNum LB(MinCostArborescence<Digraph, LengthMap> mca, Digraph::Node root) {
  //mca = new MinCostArborescence(d,length);
   mca.run(root);
   return mca.arborescenceCost();
 }


void findOneDegreeVerticesAndBoundY() {
  int degree[input.nNodes];

  for (int i = 0; i < input.nNodes; i++)
    degree[i] = 0;
  
  FOR_EACH_EDGE_e
    degree[e->u]++;
    degree[e->v]++;
  }

  FOR_EACH_EDGE_e
    if ((degree[e->u] == 1) || (degree[e->v] == 1)) {
      FOR_EACH_EDGE_f
	if (e->label != f->label)
	  cplexLP.varYArray[e->label][f->label].setBounds(0,0);
        else
	  cplexLP.varYArray[e->label][f->label].setBounds(1,1);
      }
    } 
  }
}

/*
Removed edges must not be part of the set of available edges.
FixedEdges will be forced to be part of solution.
*/
IloNum greedySpanner(list<OurEdge*>& fixedEdges, list<OurEdge*>& solution) {
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
    // e->flag = 0;
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
    
//----------------

  vector<OurEdge*> edges;
  edges.reserve(input.nEdges);

  int j = 0;

  FOR_EACH_EDGE_e
    edges[j] = input.edges[e->label];
    j++;
  }

  std::sort(edges.begin(), edges.end(), compareEdgesCost);

  // FOR_EACH_EDGE_e 
  // for (int i = 0; i < fracSize; i++) {
  for (int i = 0; i < input.nEdges; i++) {
    OurEdge* e = edges[i];
    
    if (DEBUG == 12)
      cout << "aresta que ficou: " << e-> u << "-" << e->v << endl;
    Digraph::Node s = digraph.source(e->uv);
    Digraph::Node t = digraph.target(e->uv);
    
    Dijkstra<Digraph, LengthMap> dijkstra(tempDigraph, tempLength);
    bool reached = dijkstra.run(nr[s], nr[t]);
    if ((!reached) || ((input.strFactor * e->cost) < dijkstra.dist(nr[t]))) {
      // if (e->flag != -1) {
	Digraph::Arc arcIn = tempDigraph.addArc(nr[s], nr[t]);
	Digraph::Arc arcOut = tempDigraph.addArc(nr[t], nr[s]);
	IloNum len = e->cost;
	tempLength[arcIn] = len;
	tempLength[arcOut] = len;

	solution.push_back(e);
	ub += len;	
      // } else {//ao remover esta aresta, nao é possível encontrar solução
      // 	return NO_GREEDY_SOLUTION;
      // }
    }    
  }

  return ub;

}


IloInt separate(IloExpr* expr, IloInt s, IloInt t, IloInt edge, IloNumArray2 ySol) {
  Digraph::ArcMap<IloNum> cap(digraph);
  // IloInt i, j;
  // IloInt numEdges = getNumberEdges(n);
  std::map<int,int>::iterator cutIt;

  FOR_EACH_EDGE_e
    //se o valor eh proximo de 0 (pode ser negativo ou positivo)
    if (abs(ySol[edge][e->label]) <= cplexLP.tol) {
      //if (abs(sol[edge][ind]) <= tol) {
      cap[e->uv] = 0;
      cap[e->vu] = 0;
    } else {
      cap[e->uv] = ySol[edge][e->label];
      cap[e->vu] = ySol[edge][e->label];
    }
  
  }


//-----------------------------------
  /*
    Preflow provides an implementation of Goldberg-Tarjan's preflow 
push-relabel algorithm producing a flow of maximum value in a digraph
   */
  Preflow<Digraph, Digraph::ArcMap<double> > pre(digraph,cap,digraphNodes[s], digraphNodes[t]);
  pre.init();
  pre.run();
  if (pre.flowValue() >= 1.0 - cplexLP.tol) {
    return 0;
  } else {
    Digraph:: NodeMap<bool> cut(digraph);
    pre.minCutMap(cut);
    
    for(Digraph::ArcIt e(digraph); e!=INVALID; ++e) {
      /*Observe que como estou considerando um digrafo e cada aresta do grafo 
original foi transformada em dois arcos, apenas um dos arcos será considerado 
como "aresta" de corte, pois os arcos estao em sentidos contrarios (visto que 
se se o source de um arco estiver dentro de um dos lados do corte, o source do 
outro arco nao estara dentro deste mesmo lado do corte).
       */
      if ((cut[digraph.source(e)] && !cut[digraph.target(e)])) {
	
	/*pedaço do código para adicionar menos inequações*/
	cutIt = globalCutMap.find(digraph.id(e));
	if (cutIt == globalCutMap.end()) {
	  globalCutMap[digraph.id(e)] = edge;
	} else {
	  return 0;
	}
	/*----------------------------*/

	OurEdge* origEdge = edgeMap[e];
	*expr += cplexLP.varYArray[edge][origEdge->label];
      }
    }
    return 1;
  }

//-----------------------------------

}

void printSolution(IloNumArray& xSol, IloNumArray2& sol) {
  FOR_EACH_EDGE_e
    if (xSol[e->label] > cplexLP.tol) {
      cout << "x[" << e->u << "-" << e->v << "]: " << xSol[e->label] << endl;
    }

    FOR_EACH_EDGE_f
      cout << "y[" << e->u << "-" << e->v << "][" << f->u << "-" << f->v << "]: " << sol[e->label][f->label] << endl;
    }
  }
}

void printLPSolution(bool xFlag, bool yFlag, bool value) {
  cout << "dentro de printLPSolution" << endl;
  
  if (xFlag) {
    IloNumArray sol(cplexLP.env, input.nEdges);

    sol = IloNumArray(cplexLP.env);
    cplexLP.solver->getValues(sol, cplexLP.varXArray);
    
    
    FOR_EACH_EDGE_e      
      double val;
      // val = cplexLP.solver->getValue(cplexLP.varXArray[e->label]);
      val = sol[e->label];
      
      if ( val > cplexLP.tol ) {
	cout << "x[" << e->u << "-" << e->v << "]";
	if (value)
	  cout << ":: " << val << "(" << e->cost << ")" << endl;
	else
	  cout << endl;
      }
    }
  }

  if (yFlag) {
    IloNumArray2 ySol(cplexLP.env, input.nEdges);
    FOR_EACH_EDGE_e
      ySol[e->label] = IloNumArray(cplexLP.env);
      cplexLP.solver->getValues(ySol[e->label], cplexLP.varYArray[e->label]);

      FOR_EACH_EDGE_f
	double val;
	val = ySol[e->label][f->label];

	if ( val > cplexLP.tol ) {
	  cout << "y[" << e->u << "-" << e->v << "][" << f->u << "-" << f->v << "]";
	  if (value)
	    cout << ":: " << val << endl;
	  else
	    cout << endl;
	}	
      }
    }
  }
}

ILOMIPINFOCALLBACK0(InfoCallback/*, IloCplex, cplex, Edges, y, Edge, x, IloNum, tol, IloInt, n*/) {
  IloEnv env = getEnv();
  env.out() << "dentro de InfoCallback" << endl;
  // printConstraints();
   // IloNumArray2 sol(cplexLP.env, input.nEdges);
   // IloNumArray xSol(cplexLP.env, input.nEdges);
   // for (int i = 0; i < input.nEdges; i++) {
   //    sol[i] = IloNumArray(cplexLP.env);
   //    getValues(sol[i], cplexLP.varYArray[i]);
   // }
   // getValues(xSol, cplexLP.varXArray);
   // if (DEBUG == 8) {
   //   printSolution(xSol, sol);
   // }  
}

ILOINCUMBENTCALLBACK0(IncumbentCallback/*, Edge, x, Edges, y, IloInt, numEdges, IloNumArray2, inputData, vector<Digraph::Arc>&, arcsVec, IloNum, tol, IloNum, global_ub*/) {
  IloNumArray2 sol(cplexLP.env, input.nEdges);
  IloNumArray xSol(cplexLP.env, input.nEdges);

   for (int i = 0; i < input.nEdges; i++) {
      sol[i] = IloNumArray(cplexLP.env);
      getValues(sol[i], cplexLP.varYArray[i]);
   }

  cout << "dentro de incumbent" << endl;
  getValues(xSol, cplexLP.varXArray);
  printSolution(xSol, sol);
    //printSol(xSol, numEdges, x);
  cout << "fim de incumbent" << endl;
}

/*
  Calcula um LB para cada nó de árvore de branch and bound e verifica se este 
LB local é maior do que o UB global. Se for, interrompe a árvore neste nó. 
  O LB é calculado somando as arestas internas dos componentes cujas arestas 
são induzidas pelas variáveis cujos valores são != 0, somando com uma árvore 
geradora mínima que conecta estes componentes.
*/
// ILOBRANCHCALLBACK6(BranchCallback, Edge, x, IloInt, numEdges, IloNumArray2, inputData, vector<Digraph::Arc>&, arcsVec, IloNum, tol, IloNum, global_ub) {
//   IloInt n = inputData.getSize();
//   IloEnv env = getEnv();
//   IloNumArray xSol(env, numEdges);
//   IloInt i, j;
//   Digraph local_spanner;
//   LengthMap local_length(local_spanner);
//   Digraph::NodeMap<int> scomp(local_spanner);
//   IloNum ub = 0;
//   Digraph::ArcMap<Digraph::Arc> local_ecr(local_spanner);
//   cout << "dentro de branch callback" << endl;

//   getValues(xSol, x);
//   printSol(xSol, numEdges, x);
//   if (DEBUG == 2) {
//     cout << "branch callback2" << endl; 
//     IloCplex::MIPCallbackI::NodeId id = getNodeId();
//     cout << "remaining nodes: " << getNremainingNodes() << endl;
//   }

//   digraphCopy(d, local_spanner).
//     nodeRef(original_nr).arcRef(original_er).
//     run();

//   /* 
//     local_spanner representa um grafo induzido pelas arestas cuja variável 
//     possui valor diferente de 0.
//    */
//   IloInt count = -1;
//   for (i = 0; i < n; i++) {
//     for (j = (i+1); j < n; j++) {
//       if (inputData[i][j] != NO_EDGE_CONST) {
// 	count++;
// 	if (DEBUG == 1)
// 	  env.out() << ", i: " << i << ", j: " << j << endl;
// 	IloInt ind = count*2;//getEdgeIndex(n, i+1, j+1);
// 	  Digraph::Arc e = arcsVec[ind];
// 	  Digraph::Arc f = arcsVec[ind+1];
// 	  if (DEBUG == 1) {
// 	    cout << "id em arcsVec de e: " << ind << endl;
// 	    cout << "id em arcsVec de f: " << ind+1 << endl;
// 	  }
// 	  if (abs(xSol[ind]) <= tol) {
// 	  //Para cada aresta do grafo, existem duas posições em arcsVec.	
// 	    local_spanner.erase(original_er[e]);
// 	    local_spanner.erase(original_er[f]);
// 	  } else {
// 	    local_length[original_er[e]] = length[e];
// 	    local_length[original_er[f]] = length[f];
// 	  }
//       }
//     }
//   }

// int comNum = stronglyConnectedComponents(local_spanner,scomp);
//  if (DEBUG == 4)
//    fprintf(ofp, "num. de componentes: %d\n", comNum);
// /*
//   tempDig representa um grafo onde cada componente de local_spanner é contraído 
//   para um vértice.
// */
//   Digraph tempDig;
//   Digraph::Node tempNodeVec[comNum];
//   LengthMap tempLength(tempDig);
//   IloNum lb = 0;

//   if (comNum > 1) {
//     for (i = 0; i < comNum; i++) {
//       tempNodeVec[i] = tempDig.addNode();
//     }
//     /*
//       Como todos os arcos de local_spanner sao percorridos e em local_spanner, 
// para cada aresta existe dois arcos, os arcos em ambos os sentidos serao 
// adicionados a tempDig.
//      */
//     IloNum localWeight = 0;
//     for (Digraph::ArcIt a(local_spanner); a != INVALID; ++a) { 
//       //Digraph::Arc arc = *a;
//       Digraph::Node s = local_spanner.source(a);
//       Digraph::Node t = local_spanner.target(a);
//       if (scomp[s] != scomp[t]) {
// 	/*
// 	  Cada componente de local_spanner é identificado com um id em scomp 
// 	  que começa em 0.
// 	*/
// 	Digraph::Arc newArc = tempDig.addArc(tempNodeVec[scomp[s]], tempNodeVec[scomp[t]]);
// 	tempLength[newArc] = local_length[a];
//       } else {//a aresta é interna a um dos componentes
// 	localWeight += local_length[a];
//       }
//     }

//     /*
//       Observe que para cada aresta interna, existem dois arcos. Dividimos o 
// peso por 2 para só levar em consideracao o peso de um dos arcos.
//      */
//     localWeight = localWeight/2;

//     MinCostArborescence<Digraph, LengthMap> temp_mca(tempDig,tempLength);
//     lb = localWeight + LB(temp_mca, tempNodeVec[0]);
//   }

//   if (DEBUG == 2) {
//     cout << "lower bound calculado: " << lb << endl;
//     cout << "num. de branches: " << getNbranches() << endl;
//   }
//   if (lb > global_ub) {
//     if (DEBUG == 2)
//       cout << "executou prune" << endl;
//     prune();
//   } else return;
// }

ILOSIMPLEXCALLBACK0(MyCallback) {
  cout << "Iteration " << getNiterations() << ": ";
  if ( isFeasible() ) {
     cout << "Objective = " << getObjValue() << endl;
  } else {
    cout << "Infeasibility measure = " << getInfeasibility() << endl;
  }
}

ILOBRANCHCALLBACK0(BranchCallback2/*, Edge, x, Edges, y, IloInt, numEdges, IloNumArray2, inputData, vector<Digraph::Arc>&, arcsVec, IloNum, tol, IloNum, global_ub*/) {
  if (DEBUG == 8) {
    cout << "dentro de BRANCHCALLBACK" << endl;
    BranchType type = getBranchType();
    switch(type) {
    case BranchOnVariable: 
      cout << "tipo de branch: " << "BranchOnVariable" << endl;
      break;
    case BranchOnSOS1: 
      cout << "tipo de branch: " << "BranchOnSOS1" << endl;
      break;
    case BranchOnSOS2: 
      cout << "tipo de branch: " << "BranchOnSOS2" << endl;
      break;
    case UserBranch:
      cout << "tipo de branch: " << "UserBranch" << endl;
      break;
    }
  }
  branchCount++;
  nodeId = getNodeId();
  if (DEBUG == 6) {
    cout << "numero de branches: " << getNbranches() << endl;
    for (int i = 0; i < getNbranches(); i++) {
      IloNumVarArray vars = IloNumVarArray(getEnv());;
      IloNumArray bounds = IloNumArray(getEnv());
      IloCplex::BranchDirectionArray dirs = IloCplex::BranchDirectionArray(getEnv());
      IloNum var = getBranch(vars, bounds, dirs, i);
      cout << "valor do retorno: " << var << endl;
      cout << "tam de vars: " << vars.getSize() << ", bounds: " << bounds.getSize() << ", dirs: " << dirs.getSize() << endl;
      if (var > 0) {
	char branchType[10];
	switch (dirs[0]) {
	case IloCplex::BranchUp:
	  cout << "tamanho de vars: " << vars.getSize() << endl;
	  cout << "vars: " << vars[0].getName() << ", tipo: U" << endl;
	  break;
	case IloCplex::BranchDown:
	  cout << "tamanho de vars: " << vars.getSize() << endl;
	  cout << "vars: " << vars[0].getName() << ", tipo: D" << endl;
	}
      }
    }
    cout << "capturou o NodeId: " << nodeId._id << endl;
  }
  if (DEBUG == 5)
    cout << "fim de BRANCHCALLBACK" << endl;
}

ILOHEURISTICCALLBACK0(HeuristicCallback/*, Edges, y, Edge, x*/) {
  IntegerFeasibilityArray feasX;
  IloNumArray             obj;
  IloNumArray             solX;
  // IloInt numEdges   = y.getSize();
  // IloNumArray2 sol (getEnv(), numEdges);
  IloNumArray2 sol (cplexLP.env, input.nEdges);

  feasX = IntegerFeasibilityArray(cplexLP.env);
  obj  = IloNumArray(cplexLP.env);
  solX    = IloNumArray(cplexLP.env);
  FOR_EACH_EDGE_e
    sol[e->label] = IloNumArray(cplexLP.env);
    getValues(sol[e->label], cplexLP.varYArray[e->label]);
  }

  try {
      getFeasibilities(feasX, cplexLP.varXArray);
      getValues       (solX   , cplexLP.varXArray);
      // IloInt cols   = solX.getSize();

      if (DEBUG == 15) {
	FOR_EACH_EDGE_e
	  cout << "teste, variavel " << cplexLP.varXArray[e->label].getName() << ": " << solX[e->label] << endl;
	}
	
	IloInt invCount = 0;
	IloNum objval = getObjValue();
	cout << "arestas na solução: " << endl;


	FOR_EACH_EDGE_e
	  if (solX[e->label] > 0)
	    cout << cplexLP.varXArray[e->label].getName() << endl;

	  FOR_EACH_EDGE_f
	    if (sol[e->label][f->label] > 0)
	      cout << cplexLP.varYArray[e->label][f->label].getName() << endl;
	  }
	}

	FOR_EACH_EDGE_f
	  if ( feasX[f->label] == Infeasible ) {
	    cout << "variavel " << cplexLP.varXArray[f->label].getName() << ": " << solX[f->label] << " está inviável" << endl;
	    invCount++;
	  } else if (feasX[f->label] == ImpliedInfeasible) {
	    cout << "variavel " << cplexLP.varXArray[f->label].getName() << ": " << solX[f->label] << " está inviável (por implicação)" << endl;
	    invCount++;	    
	  }

	  IntegerFeasibilityArray feas;
	  feas = IntegerFeasibilityArray(cplexLP.env);
	  getFeasibilities(feas, cplexLP.varYArray[f->label]);

	  FOR_EACH_EDGE_e
	    if ( feas[e->label] == Infeasible ) {
	      cout << "variavel " << cplexLP.varYArray[e->label][f->label].getName() << ": " << sol[e->label][f->label] << " está inviável" << endl;
	      invCount++;	      
	    } else if (feas[e->label] == ImpliedInfeasible) {
	      cout << "variavel " << cplexLP.varYArray[e->label][f->label].getName() << ": " << sol[e->label][f->label] << " está inviável (por implicação)" << endl;
	      invCount++;
	    }
	  }

	  feas.end();
	}

	cout << "Quantidade de variaveis inviavel: " << invCount << endl;
      }

      // for (IloInt i = 0; i < cols; i++) {
      // 	  IntegerFeasibilityArray feas;
      // 	  feas = IntegerFeasibilityArray(getEnv());
      // 	  getFeasibilities(feas, y[i]);
      // 	for (IloInt j = 0; j < cols; j++) {
      // 	  if (feas[j] == Infeasible)  {
	    
      // 	  }
      // 	}
      // }
  }
   catch (...) {
      feasX.end();
      cout << "flag 1" << endl;
      obj.end();
      cout << "flag 2" << endl;
      for (int j = 0; j < solX.getSize(); j++)
	sol[j].end();
      cout << "flag 3" << endl;
      solX.end();
      cout << "flag 4" << endl;
      throw;
   }
   feasX.end();
   obj.end();
   for (int j = 0; j < solX.getSize(); j++)
     sol[j].end();
   solX.end();
   if (DEBUG == 15) {
     cout << "fim de HEURISTICCALLBACK" << endl;
   }
}

ILONODECALLBACK0(NodeCallback) {
  if (DEBUG == 8) {
    cout << "dentro de NODECALLBACK" << endl;
  }
  IloInt remainingNodes = getNremainingNodes();
  if (DEBUG == 5) {
    for (IloInt i = 0; i < 1; i++) {
      IloNumVar var = getBranchVar(i);
      IloInt depth = getDepth(i);
      cout << "no " << getNodeId(i)._id << ", depth: " << depth << ", nome: " << var.getName() << endl;
    }
  }
  
  if (DEBUG == 8) {
    cout << "fim de NODECALLBACK" << endl;
  }
  
}
#ifdef FLOAT_VAR
ILOUSERCUTCALLBACK0(AddSetUserCutCallback/*, Edges, y, Edge, x, IloNum, tol, IloInt, n, IloNumArray2, inputData, vector<Digraph::Arc>&, arcsVec, vector<Digraph::Node>&, nodeVec*/) {
   // IloEnv env = getEnv();
   IloNumArray2 ySol(cplexLP.env, input.nEdges);
   // IloNumArray xSol(cplexLP.env, input.nEdges);

   int flag = 0;
   globalCutMap.clear();
   FOR_EACH_EDGE_e
      ySol[e->label] = IloNumArray(cplexLP.env);
      getValues(ySol[e->label], cplexLP.varYArray[e->label]);     
   }

   // getValues(xSol, cplexLP.varXArray);

   FOR_EACH_EDGE_e
     IloExpr expr(cplexLP.env);
     IloInt ret  = separate(&expr, s, t, e->label, ySol);

     if (ret != 0) {
       add(expr >= 1).end();       
       flag = 1;
     }
     expr.end();

   }

   if (!flag) {
     cout << "a sol está ok." << endl;
     return;
   }
   

   // FOR_EACH_EDGE_e
   //   ySol[e->label].end();
   // }

   // ySol.end();

   if (!flag) {
     cout << "a sol está ok." << endl;
     return;
   }  
}
#endif

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
	// cout << "cons: " << str << endl;
	// cout << "cons: " << e.asConstraint() << endl;
	if (
	    (str.find("cut_constraints") !=std::string::npos)  ||
	    (str.find("span_constraint") !=std::string::npos)  ||
	    (str.find("relate_vars") !=std::string::npos)) {
	  cout << "cons2: " << e.asConstraint() << endl;
	}
      }
    } 
  }

  cout << "chegou ao fim de printConstraints" << endl;
}


// void printConstraints() {
//   IloModel::Iterator iter(*(cplexLP.model));
//   while (iter.ok()) {
//     IloConstraint constraint = (*iter).asConstraint();
//     if (constraint.getImpl()) {
//       cplexLP.env.out() << "restrição: " << constraint.getName() << endl;
//     }
//     ++iter;
//   }
// }


ILOLAZYCONSTRAINTCALLBACK0(AddSetCutCallback/*, Edges, y, Edge, x, IloNum, tol, IloInt, n, IloNumArray2, inputData, vector<Digraph::Arc>&, arcsVec, vector<Digraph::Node>&, nodeVec*/) {
  int consCounter = 0;
   IloEnv env = getEnv();
   IloNumArray2 ySol(cplexLP.env, input.nEdges);
   // IloNumArray xSol(cplexLP.env, input.nEdges);

   int flag = 0;
   globalCutMap.clear();
   FOR_EACH_EDGE_e
      ySol[e->label] = IloNumArray(cplexLP.env);
      getValues(ySol[e->label], cplexLP.varYArray[e->label]);     
   }

   // getValues(xSol, cplexLP.varXArray);

   FOR_EACH_EDGE_e
     IloExpr expr(cplexLP.env);
     IloInt ret  = separate(&expr, e->u, e->v, e->label, ySol);

     if (ret != 0) {
       // IloRange con(cplexLP.env, expr >= 1);
       IloRange con = (expr >= 1);
       con.setName("cut_constraints");
       add(con).end();       
       // cplexLP.env.out() << con.asConstraint() << endl;       
       // // cons.add(expr >= 1);
       // // cons[consCounter].setName("cut_constraints");
       // add(expr >= 1).end();       
       flag = 1;
     }
     expr.end();

   }

   FOR_EACH_EDGE_e
     ySol[e->label].end();
   }
   ySol.end();

   return;
}

// void addTempConstraints(IloModel mod, Edges y) {
//   IloEnv env = mod.getEnv();
//   IloExpr expr(env);
//   expr += y[10][10];
//   expr += y[10][94];
//   expr += y[10][113];
//   mod.add(expr >= 1);
//   expr.end();

//   IloExpr expr1(env);
//   expr1 += y[12][12];
//   expr1 += y[12][89];
//   expr1 += y[12][118];
//   mod.add(expr1 >= 1);
//   expr1.end();

//   IloExpr expr2(env);
//   expr2 += y[12][12];
//   expr2 += y[12][89];
//   expr2 += y[12][118];
//   mod.add(expr2 >= 1);
//   expr2.end();

//   IloExpr expr3(env);
//   expr3 += y[14][118];
//   expr3 += y[14][113];
//   expr3 += y[14][98];
//   expr3 += y[14][74];
//   expr3 += y[14][28];
//   expr3 += y[14][14];
//   mod.add(expr3 >= 1);
//   expr3.end();

//   IloExpr expr4(env);
//   expr4 += y[29][42];
//   expr4 += y[29][29];
//   mod.add(expr4 >= 1);
//   expr4.end();

//   IloExpr expr5(env);
//   expr5 += y[30][61];
//   expr5 += y[30][59];
//   expr5 += y[30][42];
//   expr5 += y[30][30];
//   mod.add(expr5 >= 1);
//   expr5.end();

//   IloExpr expr6(env);
//   expr6 += y[59][59];
//   mod.add(expr6 >= 1);
//   expr6.end();

//   IloExpr expr7(env);
//   expr7 += y[61][115];
//   expr7 += y[61][71];
//   expr7 += y[61][61];
//   mod.add(expr7 >= 1);
//   expr7.end();

//   IloExpr expr8(env);
//   expr8 += y[77][85];
//   expr8 += y[77][77];
//   mod.add(expr8 >= 1);
//   expr8.end();

//   IloExpr expr9(env);
//   expr9 += y[115][115];
//   //expr9 = y[115][115];
//   mod.add(expr9 >= 1);
//   expr9.end();

//   IloExpr expr10(env);
//   expr10 += y[10][113];
//   expr10 += y[10][98];
//   expr10 += y[10][10];
//   mod.add(expr10 >= 1);
//   expr10.end();

//   IloExpr expr11(env);
//   expr11 += y[12][113];
//   expr11 += y[12][98];
//   expr11 += y[12][89];
//   expr11 += y[12][74];
//   expr11 += y[12][28];
//   expr11 += y[12][14];
//   expr11 += y[12][12];
//   mod.add(expr11 >= 1);
//   expr11.end();

//   IloExpr expr12(env);
//   expr12 += y[28][118];
//   mod.add(expr12 >= 1);
//   expr12.end();

//   IloExpr expr13(env);
//   expr13 += y[29][61];
//   expr13 += y[29][59];
//   expr13 += y[29][30];
//   expr13 += y[29][29];
//   mod.add(expr13 >= 1);
//   expr13.end();

//   IloExpr expr14(env);
//   expr14 += y[61][74];
//   expr14 += y[61][115];
//   expr14 += y[61][61];
//   mod.add(expr14 >= 1);
//   expr14.end();

//   IloExpr expr15(env);
//   expr15 += y[85][85];
//   expr15 += y[85][77];
//   mod.add(expr15 >= 1);
//   expr15.end();

//   IloExpr expr16(env);
//   expr16 += y[89][118];
//   mod.add(expr16 >= 1);
//   expr16.end();

//   IloExpr expr17(env);
//   expr17 += y[10][118];
//   expr17 += y[10][74];
//   expr17 += y[10][28];
//   expr17 += y[10][14];
//   expr17 += y[10][10];
//   mod.add(expr17 >= 1);
//   expr17.end();

//   IloExpr expr18(env);
//   expr18 += y[12][113];
//   expr18 += y[12][94];
//   expr18 += y[12][89];
//   mod.add(expr18 >= 1);
//   expr18.end();

//   IloExpr expr19(env);
//   expr19 += y[29][115];
//   expr19 += y[29][59];
//   expr19 += y[29][71];
//   expr19 += y[29][30];
//   expr19 += y[29][29];
//   mod.add(expr19 >= 1);
//   expr19.end();

//   IloExpr expr20(env);
//   expr20 += y[42][61];
//   mod.add(expr20 >= 1);
//   expr20.end();

//   IloExpr expr21(env);
//   expr21 += y[77][89];
//   expr21 += y[77][77];
//   mod.add(expr21 >= 1);
//   expr21.end();

//   IloExpr expr22(env);
//   expr22 += y[89][118];
//   mod.add(expr22 >= 1);
//   expr22.end();

//   IloExpr expr23(env);
//   expr23 += y[94][113];
//   expr23 += y[94][94];
//   mod.add(expr23 >= 1);
//   expr23.end();

// }

// int getNumberEdges(int numNodes)
// {
//   return (numNodes * (numNodes - 1))/2;
// }

// void printMatrix(IloEnv env, IloNumArray2 cost, IloInt n) {
//   IloInt i, j;
//   for (i = 0; i < n; i++) {
//     for (j = 0; j < n; j++) {
//       cout << "[" << i << "][" << j << "]: " << cost[i][j] << ", ";
//     }
//     cout << endl;
//   }
//   env.out() << endl;
// }

int
main(int argc, char **argv)
{

    IloEnv env;
    IloModel model(env);
    IloCplex solver(model);

    double optValue = -1;
    int noSolutionFlag = 0;
  
  resetTime();
   try {

     //-----------------------
     IloNum temp = readInput(argv[1], argv[2]);
     if (temp == -1)
       return -1;
     maxNumber = temp + 1;

    initLemonAndInputVariables();
    calcDistances();
    
    if (DEBUG == 22)
      cout << "depois de calcDistances, tempo: " << getSpentTime() << endl;

    //---------local lower bound-------------
    initMasterLP(env, &model, &solver);    

    MinCostArborescence<Digraph, LengthMap> mca(digraph,weight);
    IloNum lb = LB(mca, digraphNodes[0]);

    // IloNum ub = UB(nodeVec, n, dist, /*d, length,*/ arcVec);

    list<OurEdge*> tempSol = list<OurEdge*>();
    list<OurEdge*> fixedEdges = list<OurEdge*>();
    
    IloNum ub = greedySpanner(fixedEdges, tempSol);

    createILPFormulation();

    list<OurEdge*> emptySet = list<OurEdge*>();
    list<OurEdge*> bridgeSet = list<OurEdge*>();
    findBridges(emptySet, bridgeSet);

    for (list<OurEdge*>::iterator it=bridgeSet.begin(); it != bridgeSet.end(); it++) {
      OurEdge* edge = *it;
      edge->bridge = true;
      if (DEBUG == 27)
	cout << "a aresta " << edge->u << "-" << edge->v << " eh bridge" << endl;
      cplexLP.varXArray[edge->label].setBounds(1,1);
    }

    findOneDegreeVerticesAndBoundY();

    setCplexParam();


      #ifdef FLOAT_VAR
      IloCplex::Callback sec2 = cplexLP.solver->use(
      					 AddSetUserCutCallback(env, y, x, tol,edgeCost.getSize(), edgeCost, arcsVec, nodeVec));      
      #endif
      IloCplex::Callback sec = cplexLP.solver->use(
      					 AddSetCutCallback(env/*, y, x, tol,edgeCost.getSize(), edgeCost, arcsVec, nodeVec*/));
      //cplex.use(MyCallback(env));
      cplexLP.solver->use(HeuristicCallback(env/*, y, x*/));
      // cplex.use(NodeCallback(env));
      // IloCplex::Callback sec2 = cplex.use(
      //  					  BranchCallback2(env, x, y, numEdges, edgeCost, arcsVec, tol, ub));
      // IloCplex::Callback sec3 = cplex.use(
      // 					  IncumbentCallback(env, x, y, numEdges, edgeCost, arcsVec, tol, ub));
      // IloCplex::Callback sec4 = cplexLP.solver->use(
      // 						    InfoCallback(env/*, cplex, y, x, tol, n*/)
      // 					  );
    

      if ( cplexLP.solver->solve() ) {
	// end = clock();
	//end2 = cplexLP.solver->getTime();

	IloAlgorithm::Status solStatus= cplexLP.solver->getStatus();
	cout << "solStatus: " << solStatus << endl;

	if ( solStatus == IloAlgorithm::Optimal ) {

	  cout << "Solution is optimal" << endl;
	  env.out() << "Objective value: "
		    << cplexLP.solver->getObjValue() << endl;
	  optValue = cplexLP.solver->getObjValue();

	  printLPSolution(true, true, true);

	  // printSolution(true, true, true, true);	   
	  // IloNumArray sol(cplexLP.env, input.nEdges);
	  // cplexLP.solver->getValues(sol, cplexLP.x);
	  // IloNum tol = cplexLP.solver->getParam(IloCplex::EpInt);
	  // separate(sol, tol, input.nNodes, input.nEdges);
	   
	  // if (!isSolutionConnected())
	  //   cout << "Grafo final nao eh conexo" << endl;

	} else {
	  noSolutionFlag = 1;
	  // end = clock();
	  cout << "Solution status is not Optimal" << endl;
	}
      }
       
     //----------------------
      sec.end();
      // sec2.end();
      // sec3.end();
      // sec4.end();

   }
   catch (const IloException& e) {
      cerr << "Exception caught: " << e << endl;
   }
   catch (...) {
      cerr << "Unknown exception caught!" << endl;
   }

   // Close the environments

   // env.end();

   //fclose(ofp);

  double total_time;
  ofstream timeOutFile;
  int val = 1;
  //armazenando estatísticas
  total_time = getSpentTime();
  cout << "total time: " << total_time << endl;
  timeOutFile.open (argv[3], ios::out | ios::app);
  
  if (total_time >= TIME_LIMIT) {
    val = 0;
    timeOutFile << argv[4] << " " << 0 << endl;
  } else if (noSolutionFlag) {
    timeOutFile << argv[4] << " " << -1 << " " << total_time << endl;
  } else {
    timeOutFile << argv[4] << " " << optValue << " " << total_time << endl;
  }
  timeOutFile.close();

  deleteObjects(&env, &model, &solver);

  
  return val;

} // END main

void deleteObjects(IloEnv* env, IloModel* model, IloCplex* solver) {
  if (DEBUG == 13)
    cout << "dentro de deleteObjects" << endl;  
  //generated paths

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

  //cplex objects
  (*solver).end();
  (*model).end();
  (*env).end();

}

 void createILPFormulation() {
   char varName[100];
   IloExpr minExpr(cplexLP.env);

   IloConstraintArray cons(cplexLP.env);
   int consCounter = 0;
   
   cplexLP.varXArray = IloNumVarArray(cplexLP.env, input.nEdges, 0, 1, ILOINT);
   cplexLP.varYArray = IloNumVarArray2(cplexLP.env, input.nEdges);

   // IloExpr minExpr(masterLP.env);
   FOR_EACH_EDGE_e
     minExpr += cplexLP.varXArray[e->label] * e->cost;
     // cplexLP.obj->setLinearCoef(cplexLP.varXArray[e->label], e->cost);
     if (SET_VAR_NAMES) {
       sprintf(varName, "x.%d_%d", e->u, e->v);    
       cplexLP.varXArray[e->label].setName(varName);
     }

     #ifdef FLOAT_VAR
     cplexLP.varYArray[e->label] = IloNumVarArray(cplexLP.env, input.nEdges, 0, 1, ILOFLOAT);
     #else
     cplexLP.varYArray[e->label] = IloNumVarArray(cplexLP.env, input.nEdges, 0, 1, ILOINT);
     #endif
   
     FOR_EACH_EDGE_f
       if (SET_VAR_NAMES) {
	 sprintf(varName, "y.%d_%d-%d_%d", e->u, e->v, f->u, f->v);
	 cplexLP.varYArray[e->label][f->label].setName(varName);	 
       }
     }

     cplexLP.model->add(cplexLP.varYArray[e->label]);
   }

   cplexLP.model->add(cplexLP.varXArray);

   //Função objetivo
   cplexLP.model->add(IloMinimize(cplexLP.env, minExpr));
   minExpr.end();

   FOR_EACH_EDGE_e

     IloExpr sumEdgeExpr(cplexLP.env);

     FOR_EACH_EDGE_f
     if (DEBUG == 20)
       cout << "relacionando as variaveis: e (" << e->label << "), f (" << f->label << ")" << endl;
       //restrição que relaciona as variaveis
	IloExpr relateVars(cplexLP.env);
	relateVars = cplexLP.varYArray[e->label][f->label] - cplexLP.varXArray[f->label];
	IloRange con = (relateVars <= 0);
       con.setName("relate_vars");
       // cons.add(cplexLP.varYArray[e->label][f->label] <= cplexLP.varXArray[f->label]);
       // cons[consCounter++].setName("relate_vars");
       // cplexLP.model->add(cplexLP.varYArray[e->label][f->label] <= cplexLP.varXArray[f->label]);
       cplexLP.model->add(con);
	relateVars.end();

       sumEdgeExpr += cplexLP.varYArray[e->label][f->label] * f->cost;
     }

   //restrição de árvore
   // i = 0;
   // IloExpr expr(env);
   // for (tuple_list::const_iterator k = nodeIndex.begin(); i < numEdges; i++, ++k) {
   //   if ((x[i].getUB() == 0) && (x[i].getLB() == 0))
   //     continue;
   //   expr += x[i];
   // }
   // mod.add(expr <= numNodes - 1);
   // expr.end();


     //restrição de spanner
     // cplexLP.model->add(sumEdgeExpr <= input.strFactor * input.dist[e->u][e->v]);
     IloExpr spanExpr (cplexLP.env);
     spanExpr = sumEdgeExpr - input.strFactor * input.dist[e->u][e->v];
     IloRange con = (spanExpr <= 0);
     // IloRange con(cplexLP.env, sumEdgeExpr <= input.strFactor * input.dist[e->u][e->v]);
     con.setName("span_constraint");
     cplexLP.model->add(con);
     // cons.add(sumEdgeExpr <= input.strFactor * input.dist[e->u][e->v]);
     // cons[consCounter++].setName("span_constraint");
     sumEdgeExpr.end();
   }

   cplexLP.model->add(cons);
 }
