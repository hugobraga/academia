

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

#include <time.h>

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
  //unsigned const NUM_NODES = 32;
  unsigned const NO_EDGE_CONST = 999999;
  unsigned const DEBUG = 11;

  unsigned const TIME_LIMIT = 3600;

  unsigned const NUMBER_ARB = 3;
};

typedef IloArray<IloNumVarArray> Edges;
//typedef IloArray<IloIntVarArray> Edges;
//typedef IloIntVarArray Edge;
typedef IloNumVarArray Edge;

typedef IloArray<IloNumVarArray> IloNumVarArray2;
typedef IloArray<Edges> IloNumVarArray3;
typedef IloArray<IloNumArray2> IloNumArray3;

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

Digraph globalDigraph;
LengthMap length(globalDigraph);
LengthMap oppLength(globalDigraph);

map<int,Edge_lem> globalEdgeMap;

Digraph::NodeMap<Digraph::Node> original_nr(globalDigraph);
Digraph::ArcMap<Digraph::Arc> original_er(globalDigraph);

map<int,int> globalCutMap;

IloNum strFactor;
int numberOfEdges;

char fileName[] = "log.txt";
FILE *ofp;

//temp
//int globalFlag = 0;

//branch tree
IloCplex::MIPCallbackI::NodeId nodeId;
int branchCount = 0;

//statistics
int numOfEdges = 0;

// Declarations for functions in this program

void createMasterILP(IloModel mod, Edge& x, IloNumVarArray3 y, IloNumVarArray3 z, IloNumArray2 edgeCost, IloNumArray2 dist, IloNum strFactor);
int getNumberEdges(int numberNodes);
IloInt getEdgeIndex(IloInt n, IloInt v1, IloInt v2);
IloInt getVertexIndex (IloInt n, IloInt *v2, IloInt edge);
IloInt separate(IloEnv env, IloExpr* expr, IloInt s, IloInt t, IloInt n, Edges y, IloInt edge, IloNumArray2 sol, IloNum tol, IloNumArray2 inputData, vector<Digraph::Arc>& arcsVec, vector<Digraph::Node>& nodeVec);
void printSol(IloEnv env, IloNumArray xSol, IloNumArray3 sol, int numEdges, int n);
void initialize(vector<Digraph::Node>& nodeVec, IloNumArray2 inputData, vector<Digraph::Arc>& arcVec, ArcsIndex& arcsVec, IloNumArray2 dist);
IloNum UB(vector<Digraph::Node>& nodeVec, IloNum n, IloNumArray2 dist, vector<Digraph::Arc>& arcVec);
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
    nodeVec.push_back(globalDigraph.addNode());
    if (DEBUG == 4)
      cout << "id do no criado: " << globalDigraph.id(nodeVec[i]) << endl;
  }


  for (i = 0; i < n; i++) {
    for (j = (i+1); j < n; j++) {
      if (inputData[i][j] != NO_EDGE_CONST) {
	numOfEdges++;
	Digraph::Arc e = globalDigraph.addArc(nodeVec[i],nodeVec[j]);
	Digraph::Arc f = globalDigraph.addArc(nodeVec[j],nodeVec[i]);
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
	globalEdgeMap[globalDigraph.id(e)] = cutEdge;
	globalEdgeMap[globalDigraph.id(f)] = cutEdge2;
      }
    }
  }

  //Calculating the length of the All-Pair Shortest Paths (sub)graph.
  Dijkstra<Digraph, LengthMap> dijkstra(globalDigraph,length);
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
void boundNotUsedVariables(IloNumArray2 inputData, Edge &x, IloNumVarArray3 &y/*, IloNumVarArray &startVar, IloNumArray &startVal*/) {
  if (DEBUG == 11)
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
      } else {
	IloInt ind = getEdgeIndex(n, i, j);
	for (int k = 0; k < n; k++) {
	  for (int l = (k+1); l < n; l++) {
	    if (inputData[i][j] == NO_EDGE_CONST) {
	      IloInt ind2 = getEdgeIndex(n, k, l);
	      for (int a = 0; a < NUMBER_ARB; a++) {
		y[a][ind][ind2].setBounds(0, 0);
	      }
	    }
	  }
	}
      }
    }
  }
  if (DEBUG == 11)
    cout << "fim de boundNotUsedVariables" << endl;
}

void findOneDegreeVerticesAndBoundY(IloNumArray2 inputData, IloNumVarArray3 &y, IloNumVarArray &startVar, IloNumArray &startVal) {
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
	      for (int a = 0; a < NUMBER_ARB; a++) {
		y[a][ind][ind2].setBounds(0,0);
		if (DEBUG == 8)
		  cout << "y[" << a << "][" << k << "-" << l << "][" << i << "-" << j << "]: bound 0" << endl;
		
	      }
	    }
	  }
	}
      }
    }
  }

  if (DEBUG == 11)
    cout <<  "fim de findOneDegreeVerticesAndBoundY" << endl;  
}

void findBridges(Edge& x, IloNumVarArray3& y, vector<Digraph::Arc>& arcsVec, int edgesSize, int n, IloNumVarArray &startVar, IloNumArray &startVal) {

  Digraph tempDigraph;
  LengthMap tempLength(tempDigraph);
  Digraph::NodeMap<Digraph::Node> nr(globalDigraph);
  Digraph::NodeMap<Digraph::Node> ncr(tempDigraph);
  Digraph::NodeMap<Digraph::Arc> ar(globalDigraph);
  Digraph::NodeMap<Digraph::Arc> acr(tempDigraph);

  // Digraph bridgeDigraph;
  // LengthMap bridgeLength(bridgeDigraph);
  // Digraph::NodeMap<Digraph::Node> bridge_nr(d);
  // Digraph::NodeMap<Digraph::Node> bridge_ncr(bridgeDigraph);

  if (DEBUG == 8)
    cout << "dentro de findBridges" << endl;

  digraphCopy(globalDigraph, tempDigraph).
    nodeRef(nr).
    nodeCrossRef(ncr).
    run();

  for (int i = 0; i < edgesSize; i++) {
    Digraph::Arc arc = arcsVec[i*2];
    Digraph::Node s = globalDigraph.source(arc);
    Digraph::Node t = globalDigraph.target(arc);
    Digraph::Arc arcRef = findArc(tempDigraph, nr[s], nr[t]);
    Digraph::Arc oppArcRef = findArc(tempDigraph, nr[t], nr[s]);
    tempLength[arcRef] = length[arc];
    tempLength[oppArcRef] = length[arc];
  }

  for (int i = 0; i < edgesSize; i++) {
    Digraph::Arc originalArc = arcsVec[i*2];
    IloNum len = length[originalArc];
    Digraph::Node s = globalDigraph.source(originalArc);
    Digraph::Node t = globalDigraph.target(originalArc);
    Digraph::Arc arc = findArc(tempDigraph, nr[s], nr[t]);

    if (DEBUG == 6) {
      cout << "nr[s]: " << tempDigraph.id(nr[s]) << ", nr[t]: " << tempDigraph.id(nr[t]) << ", s: " << globalDigraph.id(s) << ", t: " << globalDigraph.id(t) << ", strFactor: " << strFactor << ", len: " << len << ", strFactor * len: " << strFactor*len << endl;
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
      Edge_lem edge = globalEdgeMap.find(globalDigraph.id(originalArc))->second;
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
	for (int a = 0; a < NUMBER_ARB; a++) {
	  y[a][ind][ind].setBounds(1,1);
	}
      if (DEBUG == 8)
	cout << "y[" << edge.v1 << "-" << edge.v2 << "][" << edge.v1 << "-" << edge.v2 << "]: bound 1" << endl;
	
      if (DEBUG == 6)
	cout << "achou uma ponte: " << edge.v1 << "-" << edge.v2 << ", e o ind: " << ind << endl; 
    }
    Digraph::Arc e = tempDigraph.addArc(nr[s], nr[t]);
    Digraph::Arc f = tempDigraph.addArc(nr[t], nr[s]);
    tempLength[e] = len;
    tempLength[f] = len;
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

void printSol(IloEnv env, IloNumArray xSol, IloNumArray3 sol, int numEdges, int n) {
  cout << "dentro de printSol" << endl;
  for (IloInt i = 0; i < numEdges; i++)
    if (xSol[i] > 0) {
      IloInt k, l;
      k = getVertexIndex (n, &l, i);
      env.out() << "xSol[" << k << "-" << l << "]: " << xSol[i] << endl;
    }

  //for (IloInt i = 0; i < numEdges; i++) {
  for (int a = 0; a < NUMBER_ARB; a++) {
    for (IloInt i = 0; i < numEdges; i++) {
      IloInt k, l;
      k = getVertexIndex (n, &l, i);
      for (IloInt j = 0; j < numEdges; j++) {
	if (sol[a][i][j] > 0) {
	  IloInt p, q;
	  p = getVertexIndex (n, &q, j);
	  cout << "sol[" << a << "][" << k << "-" << l << "][" << p << "-" << q << "]: " << sol[a][i][j] << endl;
	}
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

int getNumberOfEdges(IloNumArray2 costs, int n) {
  int number = 0;
  for (int i = 0; i < n; i++) {
    for (int j = i+1; j < n; j++) {
      if (costs[i][j] != NO_EDGE_CONST)
	number++;
    }
  }
  return number;
}

int
main(int argc, char **argv)
{
  time_t start, end;
  /* returns elapsed time in sec */
  // clock_t start, end;
  /* for elapsed CPU time */
  double total_time;
  start = clock();
  
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

   ofstream timeOutFile;
   ofstream edgesOutFile;
   int noSolutionFlag = 0;

   FILE * fp;
   fp = fopen ("file.txt", "w");
   fprintf(fp, "%s %s %s %d", "We", "are", "in", 2012);
   fclose(fp);
   //return 0;

   try {
       // Check the command line arguments

      if (argc != 5) {
	cerr << "Faltou nome do arquivo";
         throw (-1);
      }

      // Read edge_costs from data file
      IloNumArray2 edgeCost(env);
      ifstream data(argv[1]);
      
      if ( !data ) throw(-1);
      strFactor = atof(argv[2]);
      //cout << "strFactor: " << strFactor << endl;
      data >> edgeCost;
      //printMatrix(env, edgeCost, edgeCost.getSize());
      data.close();

      IloInt n = edgeCost.getSize();
      numberOfEdges = getNumberOfEdges(edgeCost, n);
      cout << "n: " << n << endl;
      IloNumArray2 dist(env, n);
      for (IloInt i = 0; i < n; i++) {
	dist[i] = IloNumArray(env, n);
      }

      initialize(nodeVec, edgeCost, /*d, length,*/ arcVec, arcsVec, dist);
      if (DEBUG == 4) {
	cout << "depois de initialize" << endl;
	for (IloInt i = 0; i < n; i++)
	  cout << "id do no: " << globalDigraph.id(nodeVec[i]) << endl;
       }

      MinCostArborescence<Digraph, LengthMap> mca(globalDigraph,length);
      IloNum lb = LB(mca, nodeVec[0]);
      
      //IloNum ub = UB(nodeVec, n, dist, /*d, length,*/ arcVec);
      MinCostArborescence<Digraph, LengthMap> minca(globalDigraph,oppLength);
      IloNum ub = UB(minca, nodeVec[0]);

      if (DEBUG == 15) {
	cout << "lower bound da sol: " << lb << endl;
	cout << "upper bound da sol: " << ub << endl;
      }
      //printMatrix(env, dist, dist.getSize());

      // create master ILP
      IloModel mod(env, "MWSP");
      IloInt numEdges = getNumberEdges(edgeCost.getSize());
      IloNumVarArray3 y(env, NUMBER_ARB);
      //Edge x;
      //Edge x(env, numEdges, 0, 1, ILOFLOAT);
      Edge x(env, numEdges, 0, 1, ILOINT);
      IloNumVarArray3 z(env, NUMBER_ARB);

      /*
	Esta variavel dumb serve para forçar que pelo menos uma variavel 
do PL seja inteira. Isto é necesserário para que o lazy constrainst callback 
funcione quando o PL só possua variáveis contínuas.
      */
      //dumb variable
      IloNumVar tempVar(env, 0, 1, ILOINT);
      tempVar.setBounds(1, 1);
      mod.add(tempVar);

      //cout << "antes de createMasterILP: " << endl;
 // for (Digraph::ArcIt a(d); a != INVALID; ++a) {
 //      cout << "id da aresta: " << globalDigraph.id(a) << endl;
 //  }

      createMasterILP(mod, x, y, z, edgeCost, dist, strFactor);
      //addTempConstraints(mod, y);

      //cout << "depois de createMasterILP: " << endl;
 // for (Digraph::ArcIt a(d); a != INVALID; ++a) {
 //      cout << "id da aresta: " << globalDigraph.id(a) << endl;
 //  } 
 //o PL deveria funcionar mesmo sem esta chamada de função. Ela só ajuda a otimizar.
  findBridges(x, y, arcsVec, arcVec.size(), n, startVar, startVal);
  if (DEBUG == 11)
    cout << "depois de findBridges" << endl;
  
 findOneDegreeVerticesAndBoundY(edgeCost, y, startVar, startVal);
  if (DEBUG == 11)
    cout << "depois de findOneVertices" << endl; 

  boundNotUsedVariables(edgeCost, x, y/*, startVar, startVal*/);

  if (DEBUG == 11)
    cout << "depois de boundNotUsedVariables" << endl;
      // // Set up the cut callback to be used for separating Benders' cuts
      IloCplex cplex(mod);
      cplex.exportModel("model.lp");
      //cplex.setParam(IloCplex::PreInd, IloFalse); 
      //cplex.setParam(IloCplex::Param::Preprocessing::Presolve, IloFalse);

      // cplex.setParam(IloCplex::PreInd, IloFalse);
      // cplex.setParam(IloCplex::Param::Preprocessing::Presolve, IloFalse);

      cplex.setParam(IloCplex::TiLim, TIME_LIMIT);

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
      // IloCplex::Callback sec = cplex.use(
      // 					 AddSetCutCallback(env, y, x, tol,edgeCost.getSize(), edgeCost, arcsVec, nodeVec));
      //cplex.use(MyCallback(env));
      //cplex.use(HeuristicCallback(env, y, x));
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
      //return 0;

      if ( cplex.solve() ) {
	end = clock();

         IloAlgorithm::Status solStatus= cplex.getStatus();
         env.out() << endl << "Solution status: " << solStatus << endl;

         env.out() << "Objective value: "
                         << cplex.getObjValue() << endl;

         if ( solStatus == IloAlgorithm::Optimal ) {

            // Write out the optimal spanner

	   if (DEBUG == 11) {
            IloInt i, j;
            IloNumArray sol(env, numEdges);
	    IloNumArray3 ySol(env, NUMBER_ARB);

	    sol = IloNumArray(env);
	    cplex.getValues(sol, x);
	    env.out() << "Optimal spanner:" << endl;
	    for (int a = 0; a < NUMBER_ARB; a++) {
	      for (i = 0; i < numEdges; i++) {
		ySol[a][i] = IloNumArray(env);
		cplex.getValues(ySol[a][i], y[a][i]);
		if ( sol[i] > 1e-03 ) {
		  env.out() << x[i].getName() << endl;
		}
	      }
	      
	    }
	    printSol(env, sol, ySol, numEdges, n);
	   }
         }
         else {
            cout << "Solution status is not Optimal" << endl;
         }
      }
      else {
	noSolutionFlag = 1;
	end = clock();	
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

      //sec.end();
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

   //fclose(ofp);

   //armazenando estatísticas
   total_time = (double)( end - start )/(double)CLOCKS_PER_SEC ;
   cout << "total time: " << total_time << endl;;
   timeOutFile.open (argv[3], ios::out | ios::app);
   //cout << argv[3] << endl;
   if (total_time >= TIME_LIMIT) {
     timeOutFile << 0 << endl;
   } else if (noSolutionFlag) {
     timeOutFile << -1 << endl;
   } else
     timeOutFile << total_time << endl;
   timeOutFile.close();

   edgesOutFile.open (argv[4], ios::out | ios::app);
   edgesOutFile << numOfEdges << endl;
   edgesOutFile.close();

   
   return 0;

} // END main

//v1 e v2 não podem ser iguais
IloInt getEdgeIndex(IloInt numNodes, IloInt v1, IloInt v2) {
  //return ((v1 - 1) * (2 * numNodes - v1)/2) + (v2 - v1 - 1);
  return (v1 * ((numNodes-1) + (numNodes-v1)))/2 + (v2-v1-1);
}

IloInt getArcIndex(IloInt numNodes, IloInt v1, IloInt v2) {
  if (v1 < v2) {
    return (v1 * ((numNodes-1) + (numNodes-v1)))/2 + (v2-v1-1);
  } else {
    IloInt val = (v1 * ((numNodes-1) + (numNodes-v1)))/2 + (v2-v1-1);
    return val + (v2 * ((numNodes-1) + (numNodes-v2)))/2 + (v1-v2-1);
  }
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
createMasterILP(IloModel mod, Edge& x, IloNumVarArray3 y, IloNumVarArray3 z, IloNumArray2 edgeCost, IloNumArray2 dist, IloNum strFactor)
{
  int i, j, l, k, a;
   IloEnv env = mod.getEnv();
   const IloInt numNodes = edgeCost.getSize();
   IloInt numEdges = getNumberEdges(numNodes);
   cout << "numero de nos: " << numNodes << endl;

   IloNumVarArray3 teste(env, 3);
   for (a = 0; a < 3; a++) {
     //teste[a].add(IloArray(env, numEdges));
     teste[a] = Edges(env, numEdges);
     for (int b = 0; b < numEdges; b++) {
       //teste[a][b].add(IloNumVarArray(env, numEdges, 0, 1, ILOINT));
       teste[a][b] = IloNumVarArray(env, numEdges, 0, 1, ILOINT);
     }
   }

   if (DEBUG == 11)
     env.out() << "inicio de createMasterILP, numNodes: " << numNodes << ", edges: " << numEdges << endl;

   char varName[100];
   IloInt count = 0;
   IloInt e1;
   tuple_list nodeIndex;

   //x = IloNumVarArray(env, numEdges, 0, 1, ILOINT);
   //x = IloNumVarArray(env, numEdges, 0, 1);
   //x = IloNumVarArray(env, numEdges, 0, 1, ILOFLOAT);

   for (a = 0; a < NUMBER_ARB; a++) {
     z[a] = IloNumVarArray2(env, numNodes);
     y[a] = IloNumVarArray2(env, numEdges);
     
     for (k = 0; k < numNodes; k++) {
       z[a][k] = IloNumVarArray(env, 2*numEdges, 0, 1, ILOINT);
       for (l = (k+1); l < numNodes; l++) {
	 if (edgeCost[k][l] != NO_EDGE_CONST) {
	   if (DEBUG == 10)
	     cout << "no inicio, valor de, k: " << k << ", l: " << l << endl;
	   
	   //if (edgeCost[l][k] != NO_EDGE_CONST) {
	   IloInt ind = getEdgeIndex(numNodes, k, l);
	   if (DEBUG == 11)
	     cout << "valor de, a: " << a << ", k: " << k << ", l: " << l << ", ind: " << ind << endl;
	   y[a][ind] = IloNumVarArray(env, numEdges, 0, 1, ILOINT);
	   if (DEBUG == 10) {
	     cout << "ind de y: " << ind << ", edges: " << numEdges << endl;
	   }
	   if (DEBUG == 10)
	     cout << "no fim, valor de, k: " << k << ", l: " << l << endl;
	   
	 }
	 //l = l + 1;
	 //}       
       }
     }
     
   }
   
   if (DEBUG == 11)
     cout << "parte 1" << endl;
   
   for (i = 0; i < numNodes; ++i) {
     for (j = (i + 1); j < numNodes; ++j) {
       //cout << edgeCost[i][j] << endl;
       if (edgeCost[i][j] != NO_EDGE_CONST) {
	 
	 //é necessário para limitar para 0 aqui pois abaixo dependeremos desta limitação
	 //x[count].setBounds(0, 0);
	 IloInt ind = getEdgeIndex(numNodes, i, j);
	 if (DEBUG == 10)
	   cout << "ind: " << ind << endl;
	 sprintf(varName, "x.%d_%d", (int) i, (int) j);
	 //x[count].setName(varName);
	 x[ind].setName(varName);

	 //nodeIndex.push_back( make_pair<int, int>(i,j));
	 //y[count] = IloNumVarArray(env, numEdges, 0, 1, ILOINT);
	 //y[ind] = IloNumVarArray(env, numEdges, 0, 1, ILOINT);
	 //y[count] = IloNumVarArray(env, numEdges, 0, 1);
	 //y[count] = IloNumVarArray(env, numEdges, 0, 1, ILOFLOAT);


	 if (DEBUG == 10)
	   cout << "parte 1.1" << endl;

	 for (a = 0; a < NUMBER_ARB; a++) {
	   if (DEBUG == 10)
	     cout << "a: " << a << endl;
	   IloInt count2 = 0;
	   for (k = 0; k < numNodes; ++k) {
	     if (DEBUG == 10)
	       cout << "k: " << k << endl;
	     if (DEBUG == 10)
	       cout << "parte 1.11" << endl;
	     sprintf(varName, "z.%d-%d-%d_%d", (int) a, (int) k, (int) i, (int) j);
	     z[a][k][ind].setName(varName);
	     if (DEBUG == 10)
	       cout << "parte 1.12" << endl;
	     IloInt ind2 = getArcIndex(numNodes, j, i);
	     if (DEBUG == 10)
	       cout << "parte 1.13" << endl;
	     sprintf(varName, "z.%d-%d-%d_%d", (int) a, (int) k, (int) j, (int) i);
	     z[a][k][ind2].setName(varName);	   

	     if (DEBUG == 10)
	       cout << "parte 1.2" << endl;

	   
	     for (l = (k+1); l < numNodes; ++l) {
	       if (edgeCost[k][l] != NO_EDGE_CONST) {
		 if (DEBUG == 10)
		   cout << "l: " << l << endl;
		 //if (edgeCost[k][l] != NO_EDGE_CONST) {
		 ind2 = getEdgeIndex(numNodes, k, l);
		 if (DEBUG == 10)
		   cout << "ind2: " << ind2 << endl;
		 sprintf(varName, "y.%d-%d_%d-%d_%d", (int) a, (int) i, (int) j, (int) k, (int) l);
		 //y[count][count2].setName(varName);
		 if (DEBUG == 10)
		   cout << "antes do erro" << endl;
		 y[a][ind2][ind].setName(varName);
		 if (DEBUG == 10)
		   cout << "depois do erro" << endl;	       
		 count2++;
		 //}
		 if (DEBUG == 10)
		   cout << "parte 1.3" << endl;
		 
	       }

	     }
	   }	   
	   
	 }
	 

	 //mod.add(y[count]);	 	 
       }
       count++;
     }
     //y[count][count].setBounds(0, 0);
   }
   mod.add(x);

   if (DEBUG == 11)
     cout << "parte 2" << endl;

   for (a = 0; a < NUMBER_ARB; a++) {
     for (k = 0; k < numNodes; k++) {
       mod.add(z[a][k]);
       for (l = (k+1); l < numNodes; l++) {
	 if (edgeCost[k][l] != NO_EDGE_CONST) {
	   IloInt ind = getEdgeIndex(numNodes, k, l);
	   mod.add(y[a][ind]);	   
	 }
       }
     }
     // mod.add(z[a]);
     // mod.add(y[a]);
   }

   if (DEBUG == 11)
     cout << "parte 3" << endl;
   

   //cada vertice da arb tem somente um arco de entrada
   for (a = 0; a < NUMBER_ARB; a++) {
     for (int v = 0; v < numNodes; v++) {
       for (j = 0; (j < numNodes) && (j != v); j++) {
	 IloExpr expr8(env);	 
	 for (i = 0; i < numNodes; i++) {
	   //if (i < j) {
	   if (edgeCost[i][j] != NO_EDGE_CONST) {
	     IloInt ind = getArcIndex(numNodes, i, j);
	     expr8 += z[a][v][ind];
	   }
	   
	   //}
	 }
	 mod.add(expr8 == 1);
	 expr8.end();
       }
     }     
   }
   
   IloExpr expr1(env);//restrição do número de arestas da árvore
   for (i = 0; i < numNodes; ++i) {

	 if (DEBUG == 11)
	   cout << "parte 1" << endl;
	 for (a = 0; a < NUMBER_ARB; a++) {
	   for (k = 0; k < numNodes; k++) {
	     if ((k == i) && (edgeCost[i][k] != NO_EDGE_CONST)) {
	       IloInt ind3 = getArcIndex(numNodes, i, k);
	       IloExpr expr4(env);
	       //restrição para as raizes das arborescencias
	       expr4 = z[a][k][ind3];
	       //quarta restrição
	       mod.add(expr4 == 0);
	       expr4.end();
	     }
	   }
	 }

     
     //IloExpr expr8(env);
     for (j = (i + 1); j < numNodes; ++j) {
       if (edgeCost[i][j] != NO_EDGE_CONST) {
	 IloInt ind = getEdgeIndex(numNodes, i, j);
	 if (DEBUG == 10)
	   cout << "ind: " << ind << endl;
	 expr1 += x[ind];

	 
	 if (DEBUG == 11)
	   cout << "parte 1" << endl;
	 for (a = 0; a < NUMBER_ARB; a++) {
	   for (k = 0; k < numNodes; k++) {
	     if (k == j) {
	       IloExpr expr4(env);
	       expr4 = z[a][k][ind];
	       //quarta restrição
	       mod.add(expr4 == 0);
	       expr4.end();
	     }
	   }

	   if (DEBUG == 10)
	     cout << "parte 2" << endl;	 
	   for (k = 0; k < numNodes; k++) {
	     for (l = (k+1); l < numNodes; l++) {
	       if (edgeCost[k][l] != NO_EDGE_CONST) {

		 IloInt ind2 = getEdgeIndex(numNodes, k, l);
		 if (DEBUG == 10)
		   cout << "ind2: " << ind2 << endl;
		 IloExpr expr5(env);//y maior que as diferenças do z ij
		 expr5 = y[a][ind2][ind];
		 //quinta restrição
		 mod.add(expr5 >= z[a][k][ind] - z[a][l][ind]);
		 expr5.end();
		 IloExpr expr5b(env);//y menor que as somas do z ij
		 expr5b = y[a][ind2][ind];
		 //quinta restrição
		 mod.add(expr5b <= z[a][k][ind] + z[a][l][ind]);
		 expr5b.end();
	     

		 IloInt ind3 = getArcIndex(numNodes, j, i);
		 if (DEBUG == 10)
		   cout << "ind3: " << ind3 << endl;
		 IloExpr expr6(env);//y maior que as diferenças do z ji
		 expr6 = y[a][ind2][ind];
		 //sexta restrição
		 mod.add(expr6 >= z[a][k][ind3] - z[a][l][ind3]);
		 expr6.end();
		 IloExpr expr6b(env);//y menor que as somas do z ji
		 expr6b = y[a][ind2][ind];
		 //sexta restrição
		 mod.add(expr6b <= z[a][k][ind3] + z[a][l][ind3]);
		 expr6b.end();	     

		 
	       }
	       
	     }
	   }

	   
	   
	 }
	 
	 
       }
     }
   }
   //return;

   if (DEBUG == 11)
     cout << "parte 3" << endl;
   
   //primeira restrição
   mod.add(expr1 == numNodes - 1);
   expr1.end();

   if (DEBUG == 11)
     cout << "parte 4" << endl;
   
   
   //segunda restricao
   for (a = 0; a < NUMBER_ARB; a++) {
     //for (k = 0; k < numNodes; k++) {

       for (i = 0; i < numNodes; i++) {
	 for (j = (i+1); j < numNodes; j++) {
	   if (edgeCost[i][j] != NO_EDGE_CONST) {
	     IloInt ind = getArcIndex(numNodes, i, j);
	     IloInt ind2 = getArcIndex(numNodes, j, i);
	     for (l = 0; l < numNodes; l++) {
	       IloExpr expr2(env);//relaciona as variaveis x com as variaveis z
	       expr2 = x[ind];
	       mod.add(expr2 >= z[a][i][ind] + z[a][i][ind2]);
	       expr2.end();	   	   
	     }
	   }
	 }
       }

       
       //}
     
   }
   //return;

   if (DEBUG == 11)
     cout << "parte 5" << endl;
   

  
   //objective function
   IloExpr obj(env);
   for (i = 0; i < numNodes; ++i) {
     for (j = (i + 1); j < numNodes; ++j) {       
       if (edgeCost[i][j] != NO_EDGE_CONST) {	 
	 IloInt ind = getEdgeIndex(numNodes, i, j);
	 obj += edgeCost[i][j]*x[ind];
       }
     }
   }
   /*
   i = 0;
   for (tuple_list::const_iterator k = nodeIndex.begin(); k != nodeIndex.end(); ++k, i++) {
     if (DEBUG == 1) {
       env.out() << "adicionando a variavel x[" << i << "]" << endl;
     }
     if ((x[i].getUB() == 0) && (x[i].getLB() == 0))
       continue;
     obj += edgeCost[k->first][k->second]*x[i];
   }
   */
   mod.add(IloMinimize(env, obj));
   if (DEBUG == 6)
   env.out() << "adicionou função objetivo" << endl;
   obj.end();

   //return;
   if (DEBUG == 11)
     cout << "parte 6" << endl;


   //restrição que relaciona as variáveis
   // for (i = 0; i < numEdges; ++i) {
   //   if ((x[i].getUB() == 0) && (x[i].getLB() == 0))
   //     continue;
   //   for (j = 0; j < numEdges; j++) {
   //     if ((x[j].getUB() == 0) && (x[j].getLB() == 0))
   // 	 continue;
   //     IloExpr expr(env);
   //     expr = y[i][j];
   //     mod.add(expr <= x[j]);
   //     expr.end();
   //   }
   // }

   if (DEBUG == 11)
     cout << "parte 7" << endl;
   //return;

   //restrição de spanner
   i = 0;
   for (i = 0; i < numNodes; ++i) {
     for (j = (i + 1); j < numNodes; ++j) {
       if (edgeCost[i][j] != NO_EDGE_CONST) {

	 IloInt ind = getEdgeIndex(numNodes, i, j);
	 IloExpr expr7(env);//restrição de spanner
	 for (k = 0; k < numNodes; k++) {
	   for (l = (k+1); l < numNodes; l++) {
	     if (edgeCost[k][l] != NO_EDGE_CONST) {
	       IloInt ind2 = getEdgeIndex(numNodes, k, l);
	       //expr7 += y[ind][ind2]*edgeCost[k][l];
	     
	       for (a = 0; a < NUMBER_ARB; a++) {
		 expr7 += y[a][ind][ind2]*edgeCost[k][l];
	       }
	     
	     }
	   }
	 }
	 mod.add(expr7 <= strFactor * dist[i][j]);
	 expr7.end();
	 
       }
     }
   }

   if (DEBUG == 11)
     cout << "fim do createMaster" << endl;
}// END createMasterILP
