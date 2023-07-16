

#include <ilcplex/ilocplex.h>
#include <string>
#include <cmath>
#include <vector>
#include <bitset>
#include <tuple> //pair

#include <lemon/list_graph.h>
#include <lemon/connectivity.h>
#include <lemon/concepts/maps.h>
#include <lemon/maps.h>
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

ILOSTLBEGIN

using namespace std;
using namespace lemon;
//using namespace lemon::concepts;

typedef ListDigraph Digraph;
//typedef concepts::WriteMap<Digraph::Node, bool> CutMap;

typedef vector< pair<int,int> > tuple_list;
typedef vector< Digraph::Arc > ArcsIndex;
typedef Digraph::ArcMap<IloNum> LengthMap;

  typedef Digraph::NodeMap<bool> NodeFilter;
  typedef Digraph::ArcMap<bool> ArcFilter;
  typedef SubDigraph<Digraph, NodeFilter, ArcFilter> Adaptor;
//typedef Digraph::ArcMap<double> CostMap;

namespace {
  unsigned const NUM_NODES = 32;
  unsigned const NO_EDGE_CONST = 999999;
  unsigned const DEBUG = 15;
};

typedef IloArray<IloNumVarArray> Edges;
//typedef IloArray<IloIntVarArray> Edges;
//typedef IloIntVarArray Edge;
typedef IloNumVarArray Edge;

typedef struct Edge_lem {
  IloInt v1;
  IloInt v2;
  //IloNum length;
  Digraph::Arc oppArc;
} Edge_lem;

typedef struct EdgeNodes {
  Digraph::Node v1;
  IloInt v1_id;
  Digraph::Node v2;
  IloInt v2_id;
} EdgeNodes;

Digraph d;
LengthMap length(d);
LengthMap oppLength(d);

map<int,Edge_lem> globalEdgeMap;

Digraph::NodeMap<Digraph::Node> original_nr(d);
Digraph::ArcMap<Digraph::Arc> original_er(d);

map<int,int> globalCutMap;

IloNum strFactor;

char fileName[] = "log.txt";
FILE *ofp;

//temp
//int globalFlag = 0;

//branch tree
IloCplex::MIPCallbackI::NodeId nodeId;
int branchCount = 0;

// Declarations for functions in this program

void createMasterILP(IloModel mod, Edge& x, Edges y, IloNumArray2 edgeCost, IloNumArray2 dist, IloNum strFactor);
int getNumberEdges(int numberNodes);
IloInt getEdgeIndex(IloInt n, IloInt v1, IloInt v2);
IloInt getVertexIndex (IloInt n, IloInt *v2, IloInt edge);
IloInt separate(IloEnv env, IloExpr* expr, IloInt s, IloInt t, IloInt n, Edges y, IloInt edge, IloNumArray2 sol, IloNum tol, IloNumArray2 inputData, vector<Digraph::Arc>& arcsVec, vector<Digraph::Node>& nodeVec);
void printSol(IloEnv env, IloNumArray xSol, IloNumArray2 sol, int numEdges, int n);
void initialize(vector<Digraph::Node>& nodeVec, IloNumArray2 inputData, vector<Digraph::Arc>& arcVec, ArcsIndex& arcsVec, IloNumArray2 dist);
//IloNum UB(vector<Digraph::Node>& nodeVec, IloNum n, IloNumArray2 dist, vector<Digraph::Arc>& arcVec);
IloNum UB(MinCostArborescence<Digraph, LengthMap> minca, Digraph::Node root);
void printSol(IloNumArray2 sol, IloInt n, IloNumArray2 inputData);
void printSolEdges(IloNumArray2 sol, IloInt numEdges, IloNumArray2 inputData);


/*
  Cria o digrafo que representa o grafo de entrada.
*/
void initialize(vector<Digraph::Node>& nodeVec, IloNumArray2 inputData, vector<Digraph::Arc>& arcVec, ArcsIndex& arcsVec, IloNumArray2 dist) {
  IloNum n = inputData.getSize();
  IloInt i, j;
  if (DEBUG == 1)
    cout << "inicio de initialize" << endl;

  for (i = 0; i < n; i++) {
    nodeVec.push_back(d.addNode());
    if (DEBUG == 4)
      cout << "id do no criado: " << d.id(nodeVec[i]) << endl;
  }


  for (i = 0; i < n; i++) {
    for (j = (i+1); j < n; j++) {
      if (inputData[i][j] != NO_EDGE_CONST) {
	Digraph::Arc e = d.addArc(nodeVec[i],nodeVec[j]);
	Digraph::Arc f = d.addArc(nodeVec[j],nodeVec[i]);
	arcVec.push_back(e);
	if (DEBUG == 1) 
	  cout << "id em arcsVec de e, após ser criado: " << arcsVec.size() << endl;
	arcsVec.push_back(e);
	if (DEBUG == 1) 
	  cout << "id em arcsVec de e, após ser criado: " << arcsVec.size() << endl;
	arcsVec.push_back(f);
	// cout << "id da primeira aresta: " << d.id(arcsVec[0]) << endl;
	// cout << "id da segunda aresta: " << d.id(arcsVec[1]) << endl;
	length[e] = inputData[i][j];
	length[f] = inputData[i][j];
	oppLength[e] = -inputData[i][j];
	oppLength[f] = -inputData[i][j];

	
	if (DEBUG == 8)
	  cout << "arco: " << i << " -> " << j << ": " << length[e] << endl;
	Edge_lem cutEdge, cutEdge2;
	cutEdge.v1 = i;
	cutEdge.v2 = j;
	cutEdge.oppArc = f;
	cutEdge2.v1 = j;
	cutEdge2.v2 = i;
	cutEdge2.oppArc = e;
	globalEdgeMap[d.id(e)] = cutEdge;
	globalEdgeMap[d.id(f)] = cutEdge2;
      }
    }
  }

  //Calculating the length of the All-Pair Shortest Paths (sub)graph.
  Dijkstra<Digraph, LengthMap> dijkstra(d,length);
  for (i = 0; i < n; i++) {
    for (j = (i+1); j < n; j++) {
      dijkstra.run(nodeVec[i], nodeVec[j]);
      IloNum distVal = dijkstra.dist(nodeVec[j]);
      dist[i][j] = distVal;
      if (DEBUG == 8)
      	cout << "armazenou dist[" << i << "][" << j << "]: " << dist[i][j] << endl;
      dist[j][i] = distVal;
    }
  }

  // if (DEBUG == 1) {
  //   if (stronglyConnected(d))
  //     cout << "o digrafo eh fortemente conexo" << endl;
  //   cout << "fim de initialize" << endl;  
  // }
}

bool comparator(Digraph::Arc e, Digraph::Arc f) {
  if (length[e] < length[f]) return true;
  else return false;
}

IloNum LB(MinCostArborescence<Digraph, LengthMap> mca, Digraph::Node root) {
  //mca = new MinCostArborescence(d,length);
   mca.run(root);
   return mca.arborescenceCost();
 }

IloNum UB(MinCostArborescence<Digraph, LengthMap> minca, Digraph::Node root) {
  //MinCostArborescence<Digraph, LengthMap> mca(d,oppLength);

  minca.run(root);
  return -minca.arborescenceCost();
}

/*Os pares (i,j) que não definem arestas são removidos*/
void boundNotUsedVariables(IloNumArray2 inputData, Edge &x, Edges &y/*, IloNumVarArray &startVar, IloNumArray &startVal*/) {
  if (DEBUG == 8)
    cout << "dentro de boundNotUsedVariables" << endl;
  int n = inputData.getSize();
  for (int i = 0; i < n; i++) {
    for (int j = (i+1); j < n; j++) {
      if (DEBUG == 8)
	cout << "inputData[" << i << "][" << j << "]: " << inputData[i][j] << endl;
      //o par (i,j) não define uma aresta
      if (inputData[i][j] == NO_EDGE_CONST) {
	IloInt ind = getEdgeIndex(n, i, j);
	if (DEBUG == 8)
	  cout << "x[" << i << "-" << j << "]: bound 0" << endl;
	x[ind].setBounds(0, 0);
	for (int k = 0; k < n; k++) {
	  for (int l = (k+1); l < n; l++) {
	    //cout << "antes de getEdgeIndex" << endl;
	    IloInt ind2 = getEdgeIndex(n, k, l);
	    y[ind][ind2].setBounds(0, 0);
	    if (DEBUG == 8)
	      cout << "y[" << i << "-" << j << "][" << k << "-" << l << "]: bound 0" << endl;
	  }
	}
      } else {
	for (int k = 0; k < n; k++) {
	  for (int l = (k+1); l < n; l++) {
	    //o par (k,l) não define uma aresta
	    if (inputData[k][l] == NO_EDGE_CONST) {
	      //cout << "antes de getEdgeIndex2" << endl;
	      IloInt ind = getEdgeIndex(n, i, j);
	      IloInt ind2 = getEdgeIndex(n, k, l);
	      y[ind][ind2].setBounds(0, 0);
	      if (DEBUG == 8)
	      	cout << "y[" << i << "-" << j << "][" << k << "-" << l << "]: bound 0" << endl;
	    }
	  }
	}	
      }
    }
  }
  if (DEBUG == 6)
    cout << "fim de boundNotUsedVariables" << endl;
}

void findOneDegreeVerticesAndBoundY(IloNumArray2 inputData, Edges &y, IloNumVarArray &startVar, IloNumArray &startVal) {
  int n = inputData.getSize();
  int degree[n];
  IloInt numEdges = getNumberEdges(n);

  if (DEBUG == 8)
    cout <<  "dentro de findOneDegreeVerticesAndBoundY" << endl;

  for (int i = 0; i < n; i++)
    degree[i] = 0;
  for (int i = 0; i < n; i++) {
    for (int j = (i+1); j < n; j++) {
      if (inputData[i][j] != NO_EDGE_CONST) {
	if (DEBUG == 6)
	  cout << "vai incrementar o grau de: " << i << ", " << j << endl;
	degree[i]++;
	if (DEBUG == 6)
	  cout << "grau de i: " << degree[i] << endl;
	degree[j]++;
	if (DEBUG == 6)
	  cout << "grau de j: " << degree[j] << endl;
      }
    }
  }

  for (int ind2 = 0; ind2 < numEdges; ind2++) {
    IloInt i, j;
    i = getVertexIndex(n, &j, ind2);
    if (inputData[i][j] != NO_EDGE_CONST) {
      if ((degree[i] == 1) || degree[j] == 1) {
	if (DEBUG == 6)
	  cout << "grau de " << i <<  ": " << degree[i] << ", grau de " << j << ": " << degree[j] << endl;
	for (int ind = 0; ind < numEdges; ind++) {
	  IloInt k,l;
	  k = getVertexIndex(n, &l, ind);
	  if (inputData[k][l] != NO_EDGE_CONST) {
	    /*O caminho entre os vertices da aresta ind nao pode passar pela aresta 
ind2 pois um dos extremos de ind2 tem grau 1*/
	    /*Se ind = ind2, então eles representam a mesma aresta, e neste caso é óbvio que o caminho entre os vértices k,l passam pela aresta definida por ind2*/
	    if (ind != ind2) {
	      y[ind][ind2].setBounds(0,0);
	      if (DEBUG == 8)
		cout << "y[" << k << "-" << l << "][" << i << "-" << j << "]: bound 0" << endl;
	    }
	  }
	}
      }
    }
  }

  if (DEBUG == 6)
    cout <<  "fim de findOneDegreeVerticesAndBoundY" << endl;  
}

void findBridges(Edge& x, Edges& y, vector<Digraph::Arc>& arcsVec, int edgesSize, int n, IloNumVarArray &startVar, IloNumArray &startVal) {

  Digraph tempDigraph;
  LengthMap tempLength(tempDigraph);
  Digraph::NodeMap<Digraph::Node> nr(d);
  Digraph::NodeMap<Digraph::Node> ncr(tempDigraph);
  Digraph::NodeMap<Digraph::Arc> ar(d);
  Digraph::NodeMap<Digraph::Arc> acr(tempDigraph);

  // Digraph bridgeDigraph;
  // LengthMap bridgeLength(bridgeDigraph);
  // Digraph::NodeMap<Digraph::Node> bridge_nr(d);
  // Digraph::NodeMap<Digraph::Node> bridge_ncr(bridgeDigraph);

  if (DEBUG == 8)
    cout << "dentro de findBridges" << endl;

  // digraphCopy(d, bridgeDigraph).
  //   nodeRef(bridge_nr).
  //   nodeCrossRef(bridge_ncr).
  //   run();
  // for (Digraph::ArcIt arc(bridgeDigraph); arc != INVALID; ++arc) {
  //   bridgeDigraph.erase(arc);
  // }

  digraphCopy(d, tempDigraph).
    nodeRef(nr).
    nodeCrossRef(ncr).
    run();

  for (int i = 0; i < edgesSize; i++) {
    Digraph::Arc arc = arcsVec[i*2];
    Digraph::Node s = d.source(arc);
    Digraph::Node t = d.target(arc);
    Digraph::Arc arcRef = findArc(tempDigraph, nr[s], nr[t]);
    Digraph::Arc oppArcRef = findArc(tempDigraph, nr[t], nr[s]);
    tempLength[arcRef] = length[arc];
    tempLength[oppArcRef] = length[arc];
  }

  for (int i = 0; i < edgesSize; i++) {
    Digraph::Arc originalArc = arcsVec[i*2];
    IloNum len = length[originalArc];
    Digraph::Node s = d.source(originalArc);
    Digraph::Node t = d.target(originalArc);
    Digraph::Arc arc = findArc(tempDigraph, nr[s], nr[t]);

    if (DEBUG == 6) {
      cout << "nr[s]: " << tempDigraph.id(nr[s]) << ", nr[t]: " << tempDigraph.id(nr[t]) << ", s: " << d.id(s) << ", t: " << d.id(t) << ", strFactor: " << strFactor << ", len: " << len << ", strFactor * len: " << strFactor*len << endl;
    }

    if (arc == INVALID) {
      cout << "arco invalido" << endl;
      continue;
    }

    tempDigraph.erase(arc);
    Digraph::Arc arc2 = findArc(tempDigraph, nr[t], nr[s]);
    tempDigraph.erase(arc2);

    Dijkstra<Digraph, LengthMap> dijkstra(tempDigraph, tempLength);
    bool reached = dijkstra.run(nr[s], nr[t]);
    if ((!reached) || ((strFactor * len) < dijkstra.dist(nr[t]))) {
      Edge_lem edge = globalEdgeMap.find(d.id(originalArc))->second;
      IloInt ind;
	if (edge.v1 < edge.v2)
	  ind = getEdgeIndex(n, edge.v1, edge.v2);
	else
	  ind = getEdgeIndex(n, edge.v2, edge.v1);
      
	x[ind].setBounds(1, 1);
	if (DEBUG == 8)
	  cout << "x[" << edge.v1 << "-" << edge.v2 << "]: bound 1" << endl;
      /*o único caminho (spanner) entre os vertices que definem a aresta ind 
passa pela aresta ind, pois a aresta é uma ponte*/
      y[ind][ind].setBounds(1,1);
      if (DEBUG == 8)
	cout << "y[" << edge.v1 << "-" << edge.v2 << "][" << edge.v1 << "-" << edge.v2 << "]: bound 1" << endl;

	// Digraph::Arc e = bridgeDigraph.addArc(bridge_nr[s], bridge_nr[t]);
	// Digraph::Arc f = bridgeDigraph.addArc(bridge_nr[t], bridge_nr[s]);
	// bridgeLength[e] = len;
	// bridgeLength[f] = len;

	
      if (DEBUG == 6)
	cout << "achou uma ponte: " << edge.v1 << "-" << edge.v2 << ", e o ind: " << ind << endl; 
    }
    Digraph::Arc e = tempDigraph.addArc(nr[s], nr[t]);
    Digraph::Arc f = tempDigraph.addArc(nr[t], nr[s]);
    tempLength[e] = len;
    tempLength[f] = len;
  }

  // for (Digraph::ArcIt arc(d); arc != INVALID; ++arc) {
  //   if (DEBUG == 6)
  //     cout << "checando aresta " << d.id(arc) << endl;
  //   Dijkstra<Digraph, LengthMap> dijkstra(bridgeDigraph, bridgeLength);
  //   Digraph::Node s = d.source(arc);
  //   Digraph::Node t = d.target(arc);
  //   bool reached = dijkstra.run(bridge_nr[s], bridge_nr[t]);
  //   IloNum len = length[arc];

  //   if (!reached) {
  //   } else if ((strFactor * len) < dijkstra.dist(bridge_nr[t])) {
  //   }
  // }
}

// IloNum UB(vector<Digraph::Node>& nodeVec, IloNum n, IloNumArray2 dist, vector<Digraph::Arc>& arcVec) {
//   IloNum ub = 0;  
//   IloInt i, j;
//   Digraph::ArcMap<bool> flag(d, false);
//   Digraph greedy_spanner;
//   LengthMap greedy_length(greedy_spanner);


//   if (DEBUG == 11)
//     cout << "dentro de UB" << endl;

//   //ordenar as arestas do grafo de entrada em ordem não-decrescente.
//   sort(arcVec.begin(), arcVec.end(), comparator);

//   //construção da greedy spanner
//   digraphCopy(d, greedy_spanner).
//     nodeRef(original_nr).
//     run();

//   if (DEBUG == 4)
//     cout << "antes de dar o erase: " << endl;

//  for (Digraph::ArcIt a(greedy_spanner); a != INVALID; ++a) {
//     if (DEBUG == 1)
//       cout << "apagou aresta de greedy_spanner" << endl;
//     greedy_spanner.erase(a);
//   }

//  if (DEBUG == 4)
//    cout << "depois de dar o erase: " << endl;

//   for (vector<Digraph::Arc>:: iterator it = arcVec.begin(); it != arcVec.end(); it++) {
//     Digraph::Arc arc = *it;
//     if (DEBUG == 1) {
//       cout << "length da aresta: " << length[arc] << endl;
//     }
//     Dijkstra<Digraph, LengthMap> dijkstra(greedy_spanner,greedy_length);
//     if (DEBUG == 7)
//       cout << "id dos nos:, source: " << d.id(d.source(arc)) << ", target: " << d.id(d.target(arc)) << endl;
//     bool reached = dijkstra.run(original_nr[d.source(arc)], original_nr[d.target(arc)]);
//     if ((!reached) || ((strFactor * length[arc]) < dijkstra.dist(original_nr[d.target(arc)]))) {
// 	if (DEBUG == 11)
// 	  cout << "adicionou aresta: " << d.id(d.source(arc)) << "-" << d.id(d.target(arc)) << ", custo: " << length[arc] << endl;
// 	Digraph::Arc e = greedy_spanner.addArc(original_nr[d.source(arc)], original_nr[d.target(arc)]);
// 	if (DEBUG == 1)
// 	  cout << "id do arco criado: " << greedy_spanner.id(e) << endl;
// 	Digraph::Arc f = greedy_spanner.addArc(original_nr[d.target(arc)], original_nr[d.source(arc)]);
// 	greedy_length[e] = length[arc];
// 	greedy_length[f] = length[arc];
// 	ub+= length[arc];
//     } else {
//       if (DEBUG == 7)
// 	cout << "existe caminho entre extremos da aresta" << endl;
//     }
//   }

//   return ub;
// }

IloInt separate(IloEnv env, IloExpr* expr, IloInt s, IloInt t, IloInt n, Edges y, IloInt edge, IloNumArray2 sol, IloNum tol, IloNumArray2 inputData, vector<Digraph::Arc>& arcsVec, vector<Digraph::Node>& nodeVec) {
  map<int,Edge_lem> mymap;
  Digraph::ArcMap<IloNum> cap(d);
  IloInt i, j;
  IloInt numEdges = getNumberEdges(n);
  std::map<int,int>::iterator cutIt;

  if ((DEBUG == 8)) {
    cout << "dentro de separate" << endl;
  }

  if (DEBUG == 11) {
     i = getVertexIndex(n, &j, edge);
     cout << "node s: " << d.id(nodeVec[s]) << ", node t: " << d.id(nodeVec[t]) << ", edge: " << i << "-" << j << endl;
  }

  if (DEBUG == 4) {
    cout << "dentro de separate, edge: " << edge << ", numEdges: " << numEdges << endl;

    for(Digraph::ArcIt e(d); e!=INVALID; ++e) {
      cout << "id do aresta: " << d.id(e) << endl;
    }
  }

  IloInt count = -1;
  IloInt countVirtualEdges = -1;
  for (i = 0; i < n; i++) {
    for (j = (i+1); j < n; j++) {
      countVirtualEdges++;
      if (inputData[i][j] != NO_EDGE_CONST) {
	count++;
	if (DEBUG == 8)
	  env.out() << ", i: " << i << ", j: " << j << endl;
	IloInt ind = count;//getEdgeIndex(n, i+1, j+1);
	if (DEBUG == 8) {
	  cout << "valor de edge: " << edge << ", countVirtualEdges: " << countVirtualEdges << endl;
	  env.out() << "adicionou a aresta ao grafo, cap: " << sol[edge][countVirtualEdges] << endl;
	}
	Digraph::Arc e = arcsVec[ind*2];
	if (DEBUG == 4) {
	  cout << "ind*2: " << ind*2 << endl;
	}
	fprintf(ofp, "ind: %d\n", ind);
	Digraph::Arc f = arcsVec[ind*2+1];
	if (DEBUG == 4) {
	  cout << "id da aresta e: " << d.id(e) << endl;
	}
	if (DEBUG == 4) {
	  cout << "id da aresta f: " << d.id(f) << endl;
	  fprintf(ofp, "id da aresta f: %d\n", d.id(f));
	  cout << "ind*2+1: " << ind*2+1 << endl;
	  fprintf(ofp, "ind+1: %d\n", ind+1);
	  cout << ", sol[edge][ind], edge: " << edge << ", ind: " << ind << ": " << sol[edge][countVirtualEdges] << endl;
	}
	//se o valor eh proximo de 0 (pode ser negativo ou positivo)
	if (abs(sol[edge][countVirtualEdges]) <= tol) {
	//if (abs(sol[edge][ind]) <= tol) {
	  cap[e] = 0;
	  if (DEBUG == 6)
	    cout << "valor da capacidade de e eh 0: " << 0 << endl;
	  cap[f] = 0;
	} else {
	  if (DEBUG == 8) {
	    cout << "valor da capacidade de e: " << sol[edge][countVirtualEdges] << endl;
	  }
	  cap[e] = sol[edge][countVirtualEdges];
	  cap[f] = sol[edge][countVirtualEdges];
	}
	Edge_lem cutEdge, cutEdge2;
	cutEdge.v1 = i;
	cutEdge.v2 = j;
	cutEdge2.v1 = j;
	cutEdge2.v2 = i;
	mymap[d.id(e)] = cutEdge;
	mymap[d.id(f)] = cutEdge2;
      }
    }
  }
  
  if (DEBUG == 8) {
    cout << "antes de preflow" << endl;
    cout << "node s: " << d.id(nodeVec[s]) << ", node t: " << d.id(nodeVec[t]) << endl;
    fprintf(ofp, "node s: %d, node t: %d\n", d.id(nodeVec[s]), d.id(nodeVec[t]));
  }

  /*
    Preflow provides an implementation of Goldberg-Tarjan's preflow 
push-relabel algorithm producing a flow of maximum value in a digraph
   */
  Preflow<Digraph, Digraph::ArcMap<IloNum>> pre(d,cap,nodeVec[s],nodeVec[t]);
  pre.init();
  pre.run();
  if (pre.flowValue() >= 1.0 - tol) {
    if ((DEBUG == 8))
      env.out() << "o fluxo maximo eh = 1" << endl;
    return 0;
  } else {
    if ((DEBUG == 8)) {
      env.out() << "o fluxo maximo eh < 1" << endl;
      fprintf(ofp, "o fluxo maximo eh < 1\n");
    }
    Digraph:: NodeMap<bool> cut(d);
    pre.minCutMap(cut);
    if ((DEBUG == 8)) {
      env.out() << "resultados de CutMap:" << endl;
      //fprintf(ofp, "resultados de CutMap\n");
    }
    if (DEBUG == 12)
      cout << "arestas do corte que compoem a inequacao: " << endl;
    for(Digraph::ArcIt e(d); e!=INVALID; ++e) {
      if (DEBUG == 8) {
	cout << "id - source: " << d.id(d.source(e)) << ", target: " << d.id(d.target(e)) << endl;
      }
      /*Observe que como estou considerando um digrafo e cada aresta do grafo 
original foi transformada em dois arcos, apenas um dos arcos será considerado 
como "aresta" de corte, pois os arcos estao em sentidos contrarios (visto que 
se se o source de um arco estiver dentro de um dos lados do corte, o source do 
outro arco nao estara dentro deste mesmo lado do corte).
       */
      if ((cut[d.source(e)] && !cut[d.target(e)])) {
	if (DEBUG == 8) {
	  cout << "eh uma aresta de corte: id - source: " << d.id(d.source(e)) << ", target: " << d.id(d.target(e)) << endl;
	}
	/*pedaço do código para adicionar menos inequações*/
	cutIt = globalCutMap.find(d.id(e));
	if (cutIt == globalCutMap.end()) {
	  globalCutMap[d.id(e)] = edge;
	} else {
	  if ((DEBUG == 8))
	    cout << "não adicionou uma inequação" << endl;
	  return 0;
	}
	/*----------------------------*/

	Edge_lem cutEdge = mymap.find(d.id(e))->second;
	// if ((DEBUG == 12)) {
	//   cout << "(" << cutEdge.v1 << "," << cutEdge.v2 << ") + " << endl;
	// }
	IloInt ind;
	if (cutEdge.v1 < cutEdge.v2)
	  ind = getEdgeIndex(n, cutEdge.v1, cutEdge.v2);
	else
	  ind = getEdgeIndex(n, cutEdge.v2, cutEdge.v1);
	if ((DEBUG == 12)) {
	  cout << "restricao adicionada, edge: " << edge << ", ind: " << ind << ", (" << cutEdge.v1 << "," << cutEdge.v2 << ") + " << endl;
	}
	*expr += y[edge][ind];
      }
    }
    if ((DEBUG == 6))
      cout << endl;
    return 1;
  }
}

IloInt getVertexIndex (IloInt n, IloInt *v2, IloInt edge) {
  IloInt row = n - 1;
  IloInt column = edge;
  for (IloInt i = 0; i < (n-1); i++) {
    if ((column - row) >= 0) {
      column -= row;
      row--;
    } else {
      *v2 = column + (i + 1);
      break;
    }
  }
  return ((n-1)-row);
}

void printSol(IloEnv env, IloNumArray xSol, IloNumArray2 sol, int numEdges, int n) {
  cout << "dentro de printSol" << endl;
  for (IloInt i = 0; i < numEdges; i++)
    if (xSol[i] > 0) {
      IloInt k, l;
      k = getVertexIndex (n, &l, i);
      env.out() << "xSol[" << k << "-" << l << "]: " << xSol[i] << endl;
    }

  //for (IloInt i = 0; i < numEdges; i++) {
  for (IloInt i = 0; i < numEdges; i++) {
	IloInt k, l;
	k = getVertexIndex (n, &l, i);
    for (IloInt j = 0; j < numEdges; j++) {
      if (sol[i][j] > 0) {
	IloInt p, q;
	p = getVertexIndex (n, &q, j);
	cout << "sol[" << k << "-" << l << "][" << p << "-" << q << "]: " << sol[i][j] << endl;
      }
    }
  }
}

void printSol(IloNumArray xSol, IloInt numEdges, Edge x) {
  for (IloInt i = 0; i < numEdges; i++) {
    cout << "xSol[" << i << "]: " << xSol[i] << endl;
    //fprintf(ofp, "xSol[%s]: %.4f\n", x[i].getName(), xSol[i]);
  }
}

void printSol(IloNumArray2 sol, IloInt n, IloNumArray2 inputData) {
  for (IloInt i = 0; i < 1; i++) {
    for (IloInt j = (i+1); j < n; j++) {
      if (inputData[i][j] != NO_EDGE_CONST) {
	cout << "sol[" << i << "][" << j << "]: " << sol[i][j] << endl;
      }
    }
  }

  // for (IloInt i = 0; i < 1; i++) {
  //   for (IloInt j = 0; j < numEdges; j++) {
  //     cout << "sol[" << i << "][" << j << "]: " << sol[i][j] << endl;
  //   }
  // }
}

void printSolEdges(IloNumArray2 sol, IloInt numEdges, IloNumArray2 inputData) {
  for (IloInt i = 0; i < 1; i++) {
    for (IloInt j = 0; j < numEdges; j++) {
      cout << "i: " << i << ", j: " << j << endl;
      //if (inputData[i][j] != NO_EDGE_CONST) {
	cout << "sol[" << i << "][" << j << "]: " << sol[i][j] << endl;
	//}
    }
  }

  // for (IloInt i = 0; i < 1; i++) {
  //   for (IloInt j = 0; j < numEdges; j++) {
  //     cout << "sol[" << i << "][" << j << "]: " << sol[i][j] << endl;
  //   }
  // }
}


ILOMIPINFOCALLBACK5(InfoCallback, IloCplex, cplex, Edges, y, Edge, x, IloNum, tol, IloInt, n) {
   IloInt i, j;
   IloInt numEdges   = y.getSize();
   IloEnv env = getEnv();
   IloNumArray2 sol(env, numEdges);
   IloNumArray xSol(env, numEdges);
   env.out() << "dentro de info callback" << endl;
   for (i = 0; i < numEdges; i++) {
      sol[i] = IloNumArray(env);
      cplex.getValues(sol[i], y[i]);
   }
   cplex.getValues(xSol, x);
   env.out() << "depois de recuperar o valor de x" << endl;
   if (DEBUG == 8) {
     for (i = 0; i < numEdges; i++)
       env.out() << "[" << i << "]: " << xSol[i] << endl;
     printSol(env, xSol, sol, numEdges, n);
   }  
}

ILOINCUMBENTCALLBACK7(IncumbentCallback, Edge, x, Edges, y, IloInt, numEdges, IloNumArray2, inputData, vector<Digraph::Arc>&, arcsVec, IloNum, tol, IloNum, global_ub) {
  IloInt n = inputData.getSize();
  IloEnv env = getEnv();
  IloNumArray2 sol(env, numEdges);
  IloNumArray xSol(env, numEdges);

   for (int i = 0; i < numEdges; i++) {
      sol[i] = IloNumArray(env);
      getValues(sol[i], y[i]);
   }

  cout << "dentro de incumbent" << endl;
  getValues(xSol, x);
  printSol(env, xSol, sol, numEdges, n);
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

ILOBRANCHCALLBACK7(BranchCallback2, Edge, x, Edges, y, IloInt, numEdges, IloNumArray2, inputData, vector<Digraph::Arc>&, arcsVec, IloNum, tol, IloNum, global_ub) {
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

ILOHEURISTICCALLBACK2(HeuristicCallback, Edges, y, Edge, x) {
  IntegerFeasibilityArray feasX;
  IloNumArray             obj;
  IloNumArray             solX;
  IloInt numEdges   = y.getSize();
  IloNumArray2 sol (getEnv(), numEdges);

      feasX = IntegerFeasibilityArray(getEnv());
      obj  = IloNumArray(getEnv());
      solX    = IloNumArray(getEnv());
	for (IloInt i = 0; i < numEdges; i++) {
	  sol[i] = IloNumArray(getEnv());
	  getValues(sol[i], y[i]);
	}

  try {
    if (DEBUG == 12) {
      cout << "dentro heuristic callback" << endl;
      cout << "objective value: " << getObjValue() << endl;
    }
      getFeasibilities(feasX, x);
      getValues       (solX   , x);
      IloInt cols   = solX.getSize();

      if (DEBUG == 12) {
	for (IloInt j = 0; j < cols; j++) {
	  cout << "teste, variavel " << x[j].getName() << ": " << solX[j] << endl;
	}
	
	IloInt invCount = 0;
	IloNum objval = getObjValue();
	cout << "arestas na solução: " << endl;
	for (IloInt i = 0; i < cols; i++) {
	  //if (solX[i] == 1)
	  if (solX[i] > 0)
	    cout << x[i].getName() << endl;
	  for (IloInt j = 0; j < cols; j++) {
	    //if (sol[i][j] == 1)
	    if (sol[i][j] > 0)
	      cout << y[i][j].getName() << endl;
	  }
	}
	for (IloInt j = 0; j < cols; j++) {
	  if ( feasX[j] == Infeasible ) {
	    cout << "variavel " << x[j].getName() << ": " << solX[j] << " está inviável" << endl;
	    invCount++;
	  } else if (feasX[j] == ImpliedInfeasible) {
	    cout << "variavel " << x[j].getName() << ": " << solX[j] << " está inviável (por implicação)" << endl;
	    invCount++;
	  } 
	  IntegerFeasibilityArray feas;
	  feas = IntegerFeasibilityArray(getEnv());
	  getFeasibilities(feas, y[j]);
	  for (IloInt i = 0; i < cols; i++) {
	    if ( feas[i] == Infeasible ) {
	      cout << "variavel " << y[j][i].getName() << ": " << sol[j][i] << " está inviável" << endl;
	      invCount++;
	    } else if (feas[i] == ImpliedInfeasible) {
	      cout << "variavel " << y[j][i].getName() << ": " << sol[j][i] << " está inviável (por implicação)" << endl;
	      invCount++;
	    }
	  }
	  feas.end();

	}
	cout << "Quantidade de variaveis inviavel: " << invCount << endl;
      }

      for (IloInt i = 0; i < cols; i++) {
	  IntegerFeasibilityArray feas;
	  feas = IntegerFeasibilityArray(getEnv());
	  getFeasibilities(feas, y[i]);
	for (IloInt j = 0; j < cols; j++) {
	  if (feas[j] == Infeasible)  {
	    
	  }
	}
      }
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
   if (DEBUG == 6) {
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

ILOLAZYCONSTRAINTCALLBACK7(AddSetCutCallback, Edges, y, Edge, x, IloNum, tol, IloInt, n, IloNumArray2, inputData, vector<Digraph::Arc>&, arcsVec, vector<Digraph::Node>&, nodeVec) {
   IloInt i, j;
   IloInt numEdges   = y.getSize();
   IloEnv env = getEnv();
   IloNumArray2 sol(env, numEdges);
   IloNumArray xSol(env, numEdges);

   // if (globalFlag == 3)
   //   return;
   // globalFlag++;

   if (DEBUG == 11) {
     cout << "dentro de lazy constraint" << endl;
   }
   if (DEBUG == 8) {
     fprintf(ofp, "dentro de lazy constraint\n");
     fprintf(ofp, "UB: %.2f\n", getBestObjValue());

     cout << "inicio de lazy constraint: " << endl;
     // for (Digraph::ArcIt a(d); a != INVALID; ++a) {
     //   cout << "id da aresta: " << d.id(a) << endl;
     // }
   }
   for (i = 0; i < numEdges; i++) {
      sol[i] = IloNumArray(env);
      getValues(sol[i], y[i]);
   }
   getValues(xSol, x);
   if (DEBUG == 8)
     env.out() << "depois de recuperar o valor de x, numEdges: " << numEdges << endl;
   if (DEBUG == 11) {
     printSol(env, xSol, sol, numEdges, n);
   }

   int flag = 0;
   globalCutMap.clear();
   for (i = 0; i < numEdges; i++) {
     if ((x[i].getUB() == 0) && (x[i].getLB() == 0))
       continue;
     IloInt s, t;
     s = getVertexIndex(n, &t, i);
     if (DEBUG == 12)
       cout << "para edge: " << i << ", valor de s: " << d.id(nodeVec[s]) << ", valor de t: " << d.id(nodeVec[t]) << endl;     
     IloExpr expr(env);
     if (DEBUG == 6) {
       cout << "antes de separate" << endl;
       cout << "node s: " << d.id(nodeVec[s]) << ", node t: " << d.id(nodeVec[t]) << endl;
       printSol(env, xSol, sol, numEdges, n);
     }
     
     IloInt ret  = separate(env, &expr, s, t, n, y, i, sol, tol, inputData, arcsVec, nodeVec);
     if (ret != 0) {
       if (DEBUG == 12)
	 cout << "adicionou a restrição ao modelo" << endl;
       add(expr >= 1).end();
       expr.end();
       flag = 1;
     }
   }

     if (DEBUG == 6)
       cout << "numero de linhas do modelo: " << getNrows() << endl;

   for (i = 0; i < numEdges; i++) {
     sol[i].end();
   }
   sol.end();

   if (!flag) {
     cout << "a sol está ok." << endl;
     return;
   }
}

void addTempConstraints(IloModel mod, Edges y) {
  IloEnv env = mod.getEnv();
  IloExpr expr(env);
  expr += y[10][10];
  expr += y[10][94];
  expr += y[10][113];
  mod.add(expr >= 1);
  expr.end();

  IloExpr expr1(env);
  expr1 += y[12][12];
  expr1 += y[12][89];
  expr1 += y[12][118];
  mod.add(expr1 >= 1);
  expr1.end();

  IloExpr expr2(env);
  expr2 += y[12][12];
  expr2 += y[12][89];
  expr2 += y[12][118];
  mod.add(expr2 >= 1);
  expr2.end();

  IloExpr expr3(env);
  expr3 += y[14][118];
  expr3 += y[14][113];
  expr3 += y[14][98];
  expr3 += y[14][74];
  expr3 += y[14][28];
  expr3 += y[14][14];
  mod.add(expr3 >= 1);
  expr3.end();

  IloExpr expr4(env);
  expr4 += y[29][42];
  expr4 += y[29][29];
  mod.add(expr4 >= 1);
  expr4.end();

  IloExpr expr5(env);
  expr5 += y[30][61];
  expr5 += y[30][59];
  expr5 += y[30][42];
  expr5 += y[30][30];
  mod.add(expr5 >= 1);
  expr5.end();

  IloExpr expr6(env);
  expr6 += y[59][59];
  mod.add(expr6 >= 1);
  expr6.end();

  IloExpr expr7(env);
  expr7 += y[61][115];
  expr7 += y[61][71];
  expr7 += y[61][61];
  mod.add(expr7 >= 1);
  expr7.end();

  IloExpr expr8(env);
  expr8 += y[77][85];
  expr8 += y[77][77];
  mod.add(expr8 >= 1);
  expr8.end();

  IloExpr expr9(env);
  expr9 += y[115][115];
  //expr9 = y[115][115];
  mod.add(expr9 >= 1);
  expr9.end();

  IloExpr expr10(env);
  expr10 += y[10][113];
  expr10 += y[10][98];
  expr10 += y[10][10];
  mod.add(expr10 >= 1);
  expr10.end();

  IloExpr expr11(env);
  expr11 += y[12][113];
  expr11 += y[12][98];
  expr11 += y[12][89];
  expr11 += y[12][74];
  expr11 += y[12][28];
  expr11 += y[12][14];
  expr11 += y[12][12];
  mod.add(expr11 >= 1);
  expr11.end();

  IloExpr expr12(env);
  expr12 += y[28][118];
  mod.add(expr12 >= 1);
  expr12.end();

  IloExpr expr13(env);
  expr13 += y[29][61];
  expr13 += y[29][59];
  expr13 += y[29][30];
  expr13 += y[29][29];
  mod.add(expr13 >= 1);
  expr13.end();

  IloExpr expr14(env);
  expr14 += y[61][74];
  expr14 += y[61][115];
  expr14 += y[61][61];
  mod.add(expr14 >= 1);
  expr14.end();

  IloExpr expr15(env);
  expr15 += y[85][85];
  expr15 += y[85][77];
  mod.add(expr15 >= 1);
  expr15.end();

  IloExpr expr16(env);
  expr16 += y[89][118];
  mod.add(expr16 >= 1);
  expr16.end();

  IloExpr expr17(env);
  expr17 += y[10][118];
  expr17 += y[10][74];
  expr17 += y[10][28];
  expr17 += y[10][14];
  expr17 += y[10][10];
  mod.add(expr17 >= 1);
  expr17.end();

  IloExpr expr18(env);
  expr18 += y[12][113];
  expr18 += y[12][94];
  expr18 += y[12][89];
  mod.add(expr18 >= 1);
  expr18.end();

  IloExpr expr19(env);
  expr19 += y[29][115];
  expr19 += y[29][59];
  expr19 += y[29][71];
  expr19 += y[29][30];
  expr19 += y[29][29];
  mod.add(expr19 >= 1);
  expr19.end();

  IloExpr expr20(env);
  expr20 += y[42][61];
  mod.add(expr20 >= 1);
  expr20.end();

  IloExpr expr21(env);
  expr21 += y[77][89];
  expr21 += y[77][77];
  mod.add(expr21 >= 1);
  expr21.end();

  IloExpr expr22(env);
  expr22 += y[89][118];
  mod.add(expr22 >= 1);
  expr22.end();

  IloExpr expr23(env);
  expr23 += y[94][113];
  expr23 += y[94][94];
  mod.add(expr23 >= 1);
  expr23.end();

}

int getNumberEdges(int numNodes)
{
  return (numNodes * (numNodes - 1))/2;
}

void printMatrix(IloEnv env, IloNumArray2 cost, IloInt n) {
  IloInt i, j;
  for (i = 0; i < n; i++) {
    for (j = 0; j < n; j++) {
      cout << "[" << i << "][" << j << "]: " << cost[i][j] << ", ";
    }
    cout << endl;
  }
  env.out() << endl;
}

int
main(int argc, char **argv)
{  
  //globalFlag = 0;
   IloEnv env;
   // /*arcsVec - armazena os vetores do grafo de entrada. Este vetor possui dois 
   //   arcos para cada aresta do grafo de entrada.
   //  */
   // /*arcVec - armazena os vetores do grafo de entrada. Este vetor só possui um 
   //   arco para cada aresta do grafo de entrada.
   //  */
   vector<Digraph::Arc> arcVec;
   vector<Digraph::Arc> arcsVec;
   
   vector<Digraph::Node> nodeVec;

   IloNumVarArray startVar(env);
   IloNumArray startVal(env);

   FILE * fp;
   fp = fopen ("file.txt", "w");
   fprintf(fp, "%s %s %s %d", "We", "are", "in", 2012);
   fclose(fp);
   //return 0;

   ofp = fopen("log.txt", "w");
   fprintf(ofp, "%s %s %s %d", "We", "are", "in", 2012);
   if (ofp == NULL)
     cout << "deu erro na abertura do arquivo" << endl;

   try {
       // Check the command line arguments

      if (argc != 3) {
	cerr << "Faltou nome do arquivo";
         throw (-1);
      }

      // Read edge_costs from data file
      IloNumArray2 edgeCost(env);
      ifstream data(argv[1]);
      if ( !data ) throw(-1);
      strFactor = atof(argv[2]);
      cout << "strFactor: " << strFactor << endl;
      data >> edgeCost;
      //printMatrix(env, edgeCost, edgeCost.getSize());
      data.close();

      IloInt n = edgeCost.getSize();
      IloNumArray2 dist(env, n);
      for (IloInt i = 0; i < n; i++) {
	dist[i] = IloNumArray(env, n);
      }

      initialize(nodeVec, edgeCost, /*d, length,*/ arcVec, arcsVec, dist);
      if (DEBUG == 4) {
	cout << "depois de initialize" << endl;
	for (IloInt i = 0; i < n; i++)
	  cout << "id do no: " << d.id(nodeVec[i]) << endl;
       }

      MinCostArborescence<Digraph, LengthMap> mca(d,length);
      IloNum lb = LB(mca, nodeVec[0]);
      
      //IloNum ub = UB(nodeVec, n, dist, /*d, length,*/ arcVec);
      MinCostArborescence<Digraph, LengthMap> minca(d,oppLength);
      IloNum ub = UB(minca, nodeVec[0]);
      if (DEBUG == 15) {
	cout << "lower bound da sol: " << lb << endl;
	cout << "upper bound da sol: " << ub << endl;
      }
      //printMatrix(env, dist, dist.getSize());

      // create master ILP
      IloModel mod(env, "MWSP");
      IloInt numEdges = getNumberEdges(edgeCost.getSize());
      Edges y(env, numEdges);
      //Edge x;
      //Edge x(env, numEdges, 0, 1, ILOFLOAT);
      Edge x(env, numEdges, 0, 1, ILOINT);

      /*
	Esta variavel dumb serve para forçar que pelo menos uma variavel 
do PL seja inteira. Isto é necesserário para que o lazy constrainst callback 
funcione quando o PL só possua variáveis contínuas.
      */
      //dumb variable
      IloNumVar tempVar(env, 0, 1, ILOINT);
      tempVar.setBounds(1, 1);
      mod.add(tempVar);

  cout << "antes de createMasterILP: " << endl;
 // for (Digraph::ArcIt a(d); a != INVALID; ++a) {
 //      cout << "id da aresta: " << d.id(a) << endl;
 //  }

      createMasterILP(mod, x, y, edgeCost, dist, strFactor);
      //addTempConstraints(mod, y);

  cout << "depois de createMasterILP: " << endl;
 // for (Digraph::ArcIt a(d); a != INVALID; ++a) {
 //      cout << "id da aresta: " << d.id(a) << endl;
 //  } 
 //o PL deveria funcionar mesmo sem esta chamada de função. Ela só ajuda a otimizar.
  findBridges(x, y, arcsVec, arcVec.size(), n, startVar, startVal);

 findOneDegreeVerticesAndBoundY(edgeCost, y, startVar, startVal);

  boundNotUsedVariables(edgeCost, x, y/*, startVar, startVal*/);

  //testBoundVariables(edgeCost.getSize(), x, y, startVar, startVal);

      // // Set up the cut callback to be used for separating Benders' cuts
      IloCplex cplex(mod);
      //cplex.exportModel("model.lp");
      //cplex.setParam(IloCplex::PreInd, IloFalse); 
      //cplex.setParam(IloCplex::Param::Preprocessing::Presolve, IloFalse);

      cplex.setParam(IloCplex::PreInd, IloFalse);
      cplex.setParam(IloCplex::Param::Preprocessing::Presolve, IloFalse);

      //cplex.setParam(IloCplex.BooleanParam.PreInd, false);
      //cplex.setParam(IloCplex::Param::Preprocessing::Reduce, 0);

      // cplex.setParam(IloCplex::Reduce, 0);
      // cplex.setParam(IloCplex::RelaxPreInd,0);
      // cplex.setParam(IloCplex::PreslvNd,-1);

      // cplex.setParam(IloCplex::Param::MIP::Display, 5);

      //cplex.setParam(IloCplex::Param::Simplex::Display, 2);

      //cplex.setParam(IloCplex::MIPInterval, 1);

      //cplex.setParam(IloCplex::Threads, 1); 

      // // Turn on traditional search for use with control callbacks
      //cplex.setParam(IloCplex::MIPSearch, IloCplex::Traditional);
      //cplex.setParam(IloCplex::RootAlg, IloCplex::Primal);

      // Solve the model and write out the solution

      /*tol corresponde ao valor de tolerância admitida para uma solução 
	inteira. Se o valor da variável inteira estiver dentro desta 
	tolerância, a variável é considerada inteira e não haverá branch.
       */
      IloNum tol = cplex.getParam(IloCplex::EpInt);
      cplex.setParam(IloCplex::CutUp, ub);
      cplex.setParam(IloCplex::CutLo, lb);
      IloCplex::Callback sec = cplex.use(
      					 AddSetCutCallback(env, y, x, tol,edgeCost.getSize(), edgeCost, arcsVec, nodeVec));
      //cplex.use(MyCallback(env));
      cplex.use(HeuristicCallback(env, y, x));
      // cplex.use(NodeCallback(env));
      // IloCplex::Callback sec2 = cplex.use(
      //  					  BranchCallback2(env, x, y, numEdges, edgeCost, arcsVec, tol, ub));
      // IloCplex::Callback sec3 = cplex.use(
      // 					  IncumbentCallback(env, x, y, numEdges, edgeCost, arcsVec, tol, ub));
      // IloCplex::Callback sec4 = cplex.use(
      // 					  InfoCallback(env, cplex, y, x, tol, n)
      // 					  );

      /*Instrui o cplex para não aplicar para não aplicar pré-processamento 
	antes de resolver o LP.
	Acho que é necessário desabilitar quando trabalha com lazy constraints.
       */
      // cplex.setParam(IloCplex::PreInd, IloFalse);

      // cplex.addMIPStart(startVar, startVal);
      // startVal.end();
      // startVar.end();

      cplex.exportModel("model.lp");

      if ( cplex.solve() ) {

         IloAlgorithm::Status solStatus= cplex.getStatus();
         env.out() << endl << "Solution status: " << solStatus << endl;

         env.out() << "Objective value: "
                         << cplex.getObjValue() << endl;

         if ( solStatus == IloAlgorithm::Optimal ) {

            // Write out the optimal spanner

            IloInt i, j;
            IloNumArray sol(env, numEdges);
	    IloNumArray2 ySol(env, numEdges);

	    sol = IloNumArray(env);
	    cplex.getValues(sol, x);
	    env.out() << "Optimal spanner:" << endl;
            for (i = 0; i < numEdges; i++) {
	      ySol[i] = IloNumArray(env);
	      cplex.getValues(ySol[i], y[i]);
	      if ( sol[i] > 1e-03 ) {
		env.out() << x[i].getName() << endl;
	      }
            }
	    printSol(env, sol, ySol, numEdges, n);
         }
         else {
            cout << "Solution status is not Optimal" << endl;
         }
      }
      else {
	env.out() << "UB: " << cplex.getBestObjValue() << endl;

         cout << "No solution available" << endl;

      // 	 /*-----------------------*/
      // 	 // No feasible solution was found. Perform conflict analysis. 
      // 	 // We allow any constraint to be part of the conflict. 
      // 	 cout << "Model is infeasible. Running conflict analysis." << endl;
      // // Build a list of constraints that can be part of the conflict. 
      // // Since bounds of variables may also render the model infeasible 
      // // we also add bounds to the set of constraints that are considered 
      // // for a conflict. 
      // 	 cout << "Building constraint list ... " << flush; 
      // 	 IloConstraintArray cons(env); 
      // 	 IloNumArray prefs(env); 
      // 	 for (IloModel::Iterator it(cplex.getModel()); it.ok(); ++it) { 
      // 	   IloExtractable e = *it; 
      // 	   if (e.isVariable()) { 
      // 	     IloNumVar v = e.asVariable(); 
      // 	     if (v.getLB() > -IloInfinity) { 
      // 	       cons.add(IloBound(v, IloBound::Lower)); 
      // 	       prefs.add(1.0); 
      // 	     } 
      // 	     if (v.getUB() < IloInfinity) { 
      // 	       cons.add(IloBound(v, IloBound::Upper)); 
      // 	       prefs.add(1.0); 
      // 	     } 
      // 	   } else if (e.isConstraint()) { 
      // 	     env.out() << "cons: " << e.asConstraint() << endl;
      // 	     cons.add(e.asConstraint()); 
      // 	     prefs.add(1.0); 
      // 	   } 
      // 	 } 
      // 	 env.out() << cons.getSize() << " elements." << endl; 
      // 	 // Refine the conflict. std::cout << "Refine the conflict ..." << std::endl; 
      // 	 if ( !cplex.refineConflict(cons, prefs) ) { 
      // 	   env.out() << "No conflict found!" << endl; 
      // 	 } else { 
      // 	   // Print out minimal conflict. 
      // 	   env.out() << "Conflict found. Minimal conflict is:" << endl; 
      // 	   int count = 0; 
      // 	   for (IloInt i = 0; i < cons.getSize(); ++i) { 
      // 	     if (cplex.getConflict(cons[i]) == IloCplex::ConflictMember) { 
      // 	       env.out() << " " << cons[i] << endl; 
      // 	       ++count; 
      // 	     } 
      // 	   } 
      // 	   env.out() << count << " constraints in minimal conflict." << endl; 
      // 	 } 
      // 	 /*-----------------------*/


	 /*-------------------------------------*/
 // if ( ( cplex.getStatus() == IloAlgorithm::Infeasible ) ||
 //           ( cplex.getStatus() == IloAlgorithm::InfeasibleOrUnbounded  ) ) {
 //         cout << endl << "No solution - starting Conflict refinement" << endl;

 //         IloConstraintArray infeas(env);
 //         IloNumArray preferences(env);

 //         infeas.add(rng); 
 //         infeas.add(sos1); infeas.add(sos2);
 //         if ( lazy.getSize() || cuts.getSize() ) {
 //           cout << "Lazy Constraints and User Cuts ignored" << endl;
 //         }
 //         for (IloInt i = 0; i<var.getSize(); i++) {
 //            if ( var[i].getType() != IloNumVar::Bool ) {
 //              infeas.add(IloBound(var[i], IloBound::Lower));
 //              infeas.add(IloBound(var[i], IloBound::Upper));
 //            }
 //         }

 //         for (IloInt i = 0; i<infeas.getSize(); i++) {
 //           preferences.add(1.0);  // user may wish to assign unique preferences
 //         }

 //         if ( cplex.refineConflict(infeas, preferences) ) {
 //            IloCplex::ConflictStatusArray conflict = cplex.getConflict(infeas);
 //            env.getImpl()->useDetailedDisplay(IloTrue);
 //            cout << "Conflict :" << endl;
 //            for (IloInt i = 0; i<infeas.getSize(); i++) {
 //              if ( conflict[i] == IloCplex::ConflictMember)
 //                   cout << "Proved  : " << infeas[i] << endl;
 //              if ( conflict[i] == IloCplex::ConflictPossibleMember)
 //                   cout << "Possible: " << infeas[i] << endl;
 //            }
 //         }
 //         else
 //            cout << "Conflict could not be refined" << endl;
 //         cout << endl;
 //      }
	 /*-------------------------------------*/

	 //cplex.writeConflict("conflict.dat");
      }

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

   env.end();

   cout << "fechou o arquivo" << endl;
   fclose(ofp);

   return 0;

} // END main

//v1 e v2 não podem ser iguais
IloInt getEdgeIndex(IloInt numNodes, IloInt v1, IloInt v2) {
  //return ((v1 - 1) * (2 * numNodes - v1)/2) + (v2 - v1 - 1);
  return (v1 * ((numNodes-1) + (numNodes-v1)))/2 + (v2-v1-1);
}

void printConstraints(IloModel mod, IloEnv env) {
  IloModel::Iterator iter(mod);
  while (iter.ok()) {
    IloConstraint constraint = (*iter).asConstraint();
    if (constraint.getImpl()) {
      env.out() << "restrição: " << constraint.getName() << endl;
    }
    ++iter;
  }
}

void
createMasterILP(IloModel mod, Edge& x, Edges y, IloNumArray2 edgeCost, IloNumArray2 dist, IloNum strFactor)
{
  IloInt i, j, k, l;
   IloEnv env = mod.getEnv();
   const IloInt numNodes = edgeCost.getSize();
   IloInt numEdges = y.getSize();

   if (DEBUG == 8)
     env.out() << "inicio de createMasterILP, numNodes: " << numNodes << ", edges: " << numEdges << endl;

   char varName[100];
   IloInt count = 0;
   IloInt e1;
   tuple_list nodeIndex;

   //x = IloNumVarArray(env, numEdges, 0, 1, ILOINT);
   //x = IloNumVarArray(env, numEdges, 0, 1);
   //x = IloNumVarArray(env, numEdges, 0, 1, ILOFLOAT);

   for (i = 0; i < numNodes; ++i) {
     for (j = (i + 1); j < numNodes; ++j) {
       sprintf(varName, "x.%d_%d", (int) i, (int) j);
       x[count].setName(varName);
       if (edgeCost[i][j] == NO_EDGE_CONST) {
	 if (DEBUG == 8)
	   cout << "x[" << i << "-" << j << "]: bound 0" << endl;
	 x[count].setBounds(0, 0);
       }
       nodeIndex.push_back( make_pair<int, int>(i,j));
       if (DEBUG == 1)
	 env.out() << "count: " << count << endl;
       y[count] = IloNumVarArray(env, numEdges, 0, 1, ILOINT);
       //y[count] = IloNumVarArray(env, numEdges, 0, 1);
       //y[count] = IloNumVarArray(env, numEdges, 0, 1, ILOFLOAT);

       IloInt count2 = 0;
       for (IloInt k = 0; k < numNodes; ++k) {
	 for (l = (k+1); l < numNodes; ++l) {
	   sprintf(varName, "y.%d_%d-%d_%d", (int) i, (int) j, (int) k, (int) l);
	   y[count][count2].setName(varName);
	   count2++;
	 }
       }
       mod.add(y[count]);
       if (DEBUG == 6)
       env.out() << "adicionou um vetor y, i: " << i << ", j: " << j << endl;
       count++;
     }
     //y[count][count].setBounds(0, 0);
   }
   mod.add(x);
   if (DEBUG == 6)
   env.out() << "adicionou vetor x" << endl;
  
   //objective function
   IloExpr obj(env);
   i = 0;
   for (tuple_list::const_iterator k = nodeIndex.begin(); k != nodeIndex.end(); ++k, i++) {
     //obj += edgeCost[k->get<0>()][k->get<1>()]*x[i];
     if (DEBUG == 1) {
       env.out() << "adicionando a variavel x[" << i << "]" << endl;
     }
     if ((x[i].getUB() == 0) && (x[i].getLB() == 0))
       continue;
     obj += edgeCost[k->first][k->second]*x[i];
   }
   mod.add(IloMinimize(env, obj));
   if (DEBUG == 6)
   env.out() << "adicionou função objetivo" << endl;
   obj.end();

   //restrição que relaciona as variáveis
   for (i = 0; i < numEdges; ++i) {
     if ((x[i].getUB() == 0) && (x[i].getLB() == 0))
       continue;
     for (j = 0; j < numEdges; j++) {
       if ((x[j].getUB() == 0) && (x[j].getLB() == 0))
	 continue;
       IloExpr expr(env);
       expr = y[i][j];
       mod.add(expr <= x[j]);
       expr.end();
     }
   }

   //restrição de árvore
   i = 0;
   IloExpr expr(env);
   for (tuple_list::const_iterator k = nodeIndex.begin(); i < numEdges; i++, ++k) {
     if ((x[i].getUB() == 0) && (x[i].getLB() == 0))
       continue;
     expr += x[i];
   }
   mod.add(expr <= numNodes - 1);
   expr.end();

   //boundYToX
   int n = edgeCost.getSize();
   for (i = 0; i < n; i++) {
     for (j = (i+1); j < n; j++) {
       IloExpr expr(env);
       //o par (i,j) não define uma aresta
       if (edgeCost[i][j] != NO_EDGE_CONST) {
	 IloInt ind = getEdgeIndex(n, i, j);
	 expr = y[ind][ind];
	 mod.add(expr == x[ind]);
	 expr.end();
       } 
     }
   }

   //boundYToX-2
  for (i = 0; i < n; i++) {
    for (j = (i+1); j < n; j++) {
      //o par (i,j) não define uma aresta
      if (edgeCost[i][j] != NO_EDGE_CONST) {
	IloInt ind = getEdgeIndex(n, i, j);
	for (k = 0; k < n; k++) {
	  for (l = (k+1); l < n; l++) {
	    if (edgeCost[k][l] == NO_EDGE_CONST) {
	      IloExpr expr(env);
	      IloInt ind2 = getEdgeIndex(n, k, l);
	      expr = y[ind][ind2];
	      mod.add(expr <= 1 - x[ind]);
	      expr.end();
	    }
	  }
	}
      }
    }
  }
   


   //restrição de spanner
   i = 0;
   for (tuple_list::const_iterator k = nodeIndex.begin(); i < numEdges; i++, ++k) {
     if ((x[i].getUB() == 0) && (x[i].getLB() == 0))
       continue;
     j = 0;
     IloExpr expr(env);
     for (tuple_list::const_iterator l = nodeIndex.begin(); j < numEdges; j++, ++l) {
       if ((x[j].getUB() == 0) && (x[j].getLB() == 0))
	 continue;
       expr += y[i][j]*edgeCost[l->first][l->second];
     }
     mod.add(expr <= strFactor * dist[k->first][k->second]);
     expr.end();
   }

}// END createMasterILP
