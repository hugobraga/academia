#include "graph-arb_spanner.h"
#include <string>
#include <cmath>
#include <vector>

#include <lemon/connectivity.h>
//#include <lemon/concepts/maps.h>
//#include <lemon/maps.h>
//#include <lemon/smart_graph.h>
//#include <lemon/preflow.h>
//#include <lemon/edmonds_karp.h>
#include <lemon/min_cost_arborescence.h>

#include <lemon/path.h>

#include <stdio.h>
#include <stdlib.h>

//#include <algorithm> //sort

#include <lemon/adaptors.h>

#include <time.h>

//ILOSTLBEGIN

//using namespace std;
//using namespace lemon;
//using namespace lemon::concepts;


namespace {
  //unsigned const NUM_NODES = 32;
  //  unsigned const NO_EDGE_CONST = 999999;
  unsigned const DEBUG = 12;

  //  unsigned const TIME_LIMIT = 3600;
};

//LemonObjects* lemonObjects;
InputData input;
CplexEnv* cplexLP;
//IloEnv env;

  Digraph digraph;  
  vector<Digraph::Node> digraphNodes;
  LengthMap weight(digraph);
  LengthMap oppWeight(digraph);
  EdgeMap edgeMap;
  ReverseEdgeMap revEdgeMap;
  ReverseNodeMap nodeLabelMap;
  NodeMap labelNodeMap;

  IloNumArray2 xVal;
  IloNumArray3 yVal;
  IloNumArray3 zVal;


/*----------FUNCTIONS DEFINITION----------------*/
void initLemonAndInputVaRiables();
IloNum LB(MinCostArborescence<Digraph, LengthMap> mca, Digraph::Node root);
IloNum UB(MinCostArborescence<Digraph, LengthMap> minca, Digraph::Node root);
void initCplexLP(IloEnv _env, IloModel* _model, IloCplex* _solver);
void initVariables(IloEnv _env, IloModel* _model, IloCplex* _solver);
void calcDistances();
void createLPFormulation(IloEnv env);
void read(int argc, char **argv, IloEnv env);
void setCplexParam();
/*----------------------------------------------*/

 void initLemonAndInputVariables() {
   digraphNodes.reserve(input.nNodes);
   input.nodes.reserve(input.nNodes);
   input.edges.reserve(input.nEdges);
   input.arcs.reserve(input.nEdges);
   if (DEBUG == 10) {
     cout << "dentro de initLemonAndInputVariables" << endl;
     cout << "nEdges: " << input.nEdges << endl;
     cout << "capacidade de digraphNodes: " << digraphNodes.capacity() <<
       ", capacidade de edges: " << input.edges.capacity() << endl;
   }
   //lemonObjects = (LemonObjects*) malloc(sizeof(LemonObjects));
   //digraph = new Digraph();

   for (int i = 0; i < input.nNodes; i++) {
     if ((DEBUG == 10) && (i > 0)) {
       cout << "antes de criar no, no " << i << " - 1: " << digraph.id(*(labelNodeMap[i-1]->nodeRef)) << endl;
     }               
     //digraphNodes.push_back(digraph.addNode());
     digraphNodes[i] = digraph.addNode();
     if ((DEBUG == 10) && (i > 0)) {
       cout << "depois de criar no, no " << i << " - 1: " << digraph.id(*(labelNodeMap[i-1]->nodeRef)) << endl;
     }          
     //Digraph::Node* lemonNode = &(digraphNodes.back());
     Digraph::Node* lemonNode = &(digraphNodes[i]);
     if ((DEBUG == 10) && (i > 0)) {
       cout << "depois de pegar o back, no " << i << " - 1: " << digraph.id(*(labelNodeMap[i-1]->nodeRef)) << endl;
     }
     //Node* node = (Node*) malloc(sizeof(Node));
     Node* node = new Node();
     if ((DEBUG == 10) && (i > 0)) {
       cout << "depois de malloc, no " << i << " - 1: " << digraph.id(*(labelNodeMap[i-1]->nodeRef)) << endl;
     }
     node->nodeRef = lemonNode;
     node->label = i;
     labelNodeMap[i] = node;
     if (DEBUG == 10) {
       cout << "no: " << digraph.id(*(labelNodeMap[i]->nodeRef)) << endl;
     }
     nodeLabelMap[*(node->nodeRef)] = i;
     //input.nodes.push_back(node);
     input.nodes[i] = node;
     if ((DEBUG == 10)) {
       cout << "depois de push_back node, no " << i << ": " << digraph.id(*(labelNodeMap[i]->nodeRef)) << endl;
     }          
   }

   if (DEBUG == 10) {
     for (int i = 0; i < input.nNodes; i++) {
       cout << "no: " << digraph.id(*(labelNodeMap[i]->nodeRef)) << endl;
     }
     cout << "depois do primeiro for" << endl;
   }

   int count = 0;
   for (int u = 0; u < input.nNodes; u++) {
     for (int v = (u+1); v < input.nNodes; v++) {
       if (input.edgeCost[u][v] != NO_EDGE_CONST) {
	if (DEBUG == 10)
	  cout << "u: " << u << ", v: " << v << endl;
		 
	 
	//Edge* edge = (Edge*) malloc(sizeof(Edge));
	Edge* edge = new Edge();
	 edge->u = u;
	 edge->v = v;
	 edge->label = count++;
	 edge->cost = input.edgeCost[u][v];
	 if (DEBUG == 10) {
	  cout << "antes de push_back " << v << endl;
	  cout << "valor de label: " << edge->label << ", size: " << input.edges.size() << ", capacity: " << input.edges.capacity() << endl;
	 }
	 input.edges[edge->label] = edge;
	 //input.edges.push_back(edge);
	if (DEBUG == 10)
	  cout << "depois de push_back " << v << endl; 	 

	if (DEBUG == 10) {
	  cout << "antes de adicionar primeiro arco" << v << endl;
	  cout << "no v: " << digraph.id(*(labelNodeMap[v]->nodeRef)) << endl;
	  cout << "no u: " << digraph.id(*(labelNodeMap[u]->nodeRef)) << endl;
	}
	 Digraph::Arc e = digraph.addArc(*(labelNodeMap[u]->nodeRef), *(labelNodeMap[v]->nodeRef));
	 if (DEBUG == 10) {
	  cout << "antes de adicionar segundo arco" << v << endl;
	 }
	 Digraph::Arc f = digraph.addArc(*(labelNodeMap[v]->nodeRef), *(labelNodeMap[u]->nodeRef));
	if (DEBUG == 10)
	  cout << "antes de weight" << v << endl;	 
	 weight[e] = input.edgeCost[u][v];
	 weight[f] = input.edgeCost[u][v];
	if (DEBUG == 10)
	  cout << "depois de weight" << v << endl;	 
	 oppWeight[e] = -input.edgeCost[u][v];
	 oppWeight[f] = -input.edgeCost[u][v];
	if (DEBUG == 10)
	  cout << "depois de oppWeight" << v << endl;
	 
	 edgeMap[e] = edge;
	 edgeMap[f] = edge;

	 //EdgeArcs* arcs = (EdgeArcs*) malloc(sizeof(EdgeArcs));
	 EdgeArcs* arcs = new EdgeArcs();
	 arcs->in = e;
	 arcs->out = f;
	 arcs->label = u+v;
	 input.arcs[edge->label] = arcs;
	 if (DEBUG == 10) {
	   cout << "adicionou arcos, cujo label eh: " << input.arcs[edge->label]->label << endl;
	   if (edge->label > 0) {
	     for (int p = 0; p <= edge->label; p++) {
	       cout << "in, source (arcos anteriores), label " << p << ": " << digraph.id(digraph.source(input.arcs[p]->in)) <<
	 ", target (arcos anteriores): " << digraph.id(digraph.target(input.arcs[p]->in)) << endl;
	       // if (input.arcs[p]->in == input.arcs[p-1]->in)
	       // 	 cout << "id dos arcos " << p << " e " << p - 1 << " são iguais" << endl;
	       // else
	       // 	cout << "id dos arcos " << p << " e " << p - 1 << " são diferentes" << endl; 
	     }
	     
	 //     cout << "in, source: " << digraph.id(digraph.source(*(input.arcs[edge->label-1]->in))) <<
	 // ", target: " << digraph.id(digraph.target(*(input.arcs[edge->label-1]->in))) << endl;
	   }
	 }

	 revEdgeMap[edge] = arcs;
       }
     }
   }

   //input.nEdges = count;
 }

IloNum LB(MinCostArborescence<Digraph, LengthMap> mca, Digraph::Node root) {
   mca.run(root);
   return mca.arborescenceCost();
 }

IloNum UB(MinCostArborescence<Digraph, LengthMap> minca, Digraph::Node root) {
  minca.run(root);
  return -minca.arborescenceCost();
}


void initCplexLP(IloEnv _env, IloModel* _model, IloCplex* _solver) {
    cplexLP = (CplexEnv*) malloc(sizeof(CplexEnv));

    cplexLP->env = _env;
    cplexLP->model = _model;
    cplexLP->solver = _solver;

    MinCostArborescence<Digraph, LengthMap> mca(digraph,weight);
    cplexLP->lb = LB(mca, *(input.nodes[0]->nodeRef));
    MinCostArborescence<Digraph, LengthMap> minca(digraph,oppWeight);
    cplexLP->ub = UB(minca, *(input.nodes[0]->nodeRef));

    // cplexLP->xSol = IloNumVarArray(cplexLP->env, input.nEdges, 0, 1, ILOINT);
    // cplexLP->x = IloNumVarArray2(cplexLP->env, NUMBER_ARB);
    // cplexLP->y = IloNumVarArray3(cplexLP->env, NUMBER_ARB);
    // cplexLP->z = IloNumVarArray3(cplexLP->env, NUMBER_ARB);
}



 void initVariables(IloEnv _env, IloModel* _model, IloCplex* _solver) {
   initLemonAndInputVariables();
   FOR_EACH_ARC_pair
     if (DEBUG == 10) {
       cout << "imprimindo label dos arcs, arc->label: " << pair->label << endl;
       cout << "in, source: " << digraph.id(digraph.source(pair->in)) <<
	 ", target: " << digraph.id(digraph.target(pair->in)) << endl;
       
     }
   }
   if (DEBUG == 10)
     cout << "depois de initLemonAndInputVariables" << endl;
   calcDistances();
   if (DEBUG == 10)
     cout << "depois de calcDistances" << endl;   
   initCplexLP(_env, _model, _solver);
   if (DEBUG == 10)
     cout << "depois de initCplexLP" << endl;      
 }

 void calcDistances() {
   //input.dist = (float **)malloc(input.nNodes * sizeof(float *));
   input.dist = new float*[input.nNodes];
   //Node *nod = input.nodes[0];
   Dijkstra<Digraph, LengthMap> dijkstra(digraph, weight);
   //cout << "numero de nos: " << input.nNodes << endl;
   FOR_EACH_NODE_u
     if (DEBUG == 10)
       cout << "label (u): " << u->label << endl;
     //input.dist[u->label] = (float *)malloc(input.nNodes * sizeof(float));
     input.dist[u->label] = new float[input.nNodes];
     dijkstra.run(*(u->nodeRef));
     FOR_EACH_NODE_v
	if (DEBUG == 10)
	  cout << "label (v): " << v->label << endl;
	       
       if (u->label < v->label) {
	Digraph::Node nod = *(v->nodeRef);
	IloNum distVal = dijkstra.dist(nod);	
	//IloNum distVal = dijkstra.dist(*(v->nodeRef));
	if (DEBUG == 11) {
	  cout << "dist entre " << u->label << " e " << v->label << ": " << distVal << endl;
	  while (dijkstra.predArc(nod) != INVALID) {
	    Digraph::Arc e = dijkstra.predArc(nod);
	    cout << "arco que começa em " << digraph.id(digraph.source(e)) <<
	      " e termina em " << digraph.id(digraph.target(e)) <<
	      " tem custo: " << weight[e] << endl;
	    nod = digraph.source(e);
	  }
	  // Path<Digraph> pp = dijkstra.path(*(v->nodeRef));
	  // for(Digraph::ArcIt e(pp); e!=INVALID; ++e) {
	  //   cout << "aresta, source: " << digraph.id(digraph.source(e)) <<
	  //     ", target: " << digraph.id(digraph.target(e)) << endl;
	  // }
	}
	 if (DEBUG == 10)
	   cout << "depois de v->nodeRef " << endl;	
	input.dist[u->label][v->label] = distVal;
	
       } else if (u->label > v->label) {
	 input.dist[u->label][v->label] = input.dist[v->label][u->label];
       }
     }
   }
 }

void testSolution() {
    xSolVal = IloNumArray(cplexLP->env, input.nEdges, 0, 1, ILOINT);
    xVal = IloNumArray2(cplexLP->env, NUMBER_ARB);
    yVal = IloNumArray3(cplexLP->env, NUMBER_ARB);
    zVal = IloNumArray3(cplexLP->env, NUMBER_ARB);

  
   for (int a = 0; a < NUMBER_ARB; a++) {
     xVal[a] = IloNumArray(cplexLP->env, input.nEdges, 0, 1, ILOINT);
     zVal[a] = IloNumArray2(cplexLP->env, input.nNodes);
     yVal[a] = IloNumArray2(cplexLP->env, input.nEdges);
     
     for (int k = 0; k < input.nNodes; k++) {
       zVal[a][k] = IloNumArray(cplexLP->env, 2*input.nEdges, 0, 1, ILOINT);
     }

     for (int l = 0; l < input.nEdges; l++) {
       yVal[a][l] = IloNumArray(cplexLP->env, input.nEdges, 0, 1, ILOINT);
     }
     
   }


  
  int numElements =
    (NUMBER_ARB*input.nEdges) + (NUMBER_ARB*input.nEdges*input.nEdges)
    + (NUMBER_ARB*input.nNodes*input.nNodes) + (input.nEdges);
  IloNumVarArray startVar(cplexLP->env);
  IloNumArray startVal(cplexLP->env);

//setting x values
 // startVal[0] = 1;
 // startVal[1] = 1;
 // startVal[2] = 1;
 // startVal[3] = 0;

 // startVal.add(1);
 // startVal.add(1);
 // startVal.add(1);
 // startVal.add(0);

  for (int a = 0; a < NUMBER_ARB; a++) {
	xVal[a][0] = 1;
	xVal[a][1] = 1;
	xVal[a][2] = 1;
	xVal[a][3] = 0;
  }
 int count = 4;
 for (int a = 1; a < NUMBER_ARB; a++) {
   for (int b = 0; b < input.nEdges; b++) {
     //startVal[count] = 0;

     //startVal.add(count);

     xVal[a][b] = 0;
     count++;
   }
 }

 //setting y values
 count = NUMBER_ARB*input.nEdges;
 int val = count;
 for (int a = 0; a < NUMBER_ARB; a++) {
   for (int b = 0; b < input.nEdges; b++) {
     for (int c = 0; c < input.nEdges; c++) {
       // switch (count) {
       // case (val + 0):
       // 	 startVal.add(1);
       // 	 break;
       // case (val + 5):
       // 	 startVal.add(1);
       // 	 break;
       // case (val + 10):
       // 	 startVal.add(1);
       // 	 break;
       // case (val + 12):
       // 	 startVal.add(1);
       // 	 break;
       // case (val + 13):
       // 	 startVal.add(1);
       // 	 break;
       // case (val + 14):
       // 	 startVal.add(1);
       // 	 break;
       // default:
       // 	 startVal.add(0);
       // }      
       // //startVal[count] = 0;
       // 	count++;

       yVal[a][b][c] = 0;
     }
   }
 }
 
 // count = NUMBER_ARB*input.nEdges;
 // startVal[count + 0] = 1;
 // startVal[count + 5] = 1;
 // startVal[count + 10] = 1;
 // startVal[count + 12] = 1;
 // startVal[count + 13] = 1;
 // startVal[count + 14] = 1;

 yVal[0][0][0] = 1;
 yVal[0][1][1] = 1;
 yVal[0][2][2] = 1;
 yVal[0][3][0] = 1;
 yVal[0][3][1] = 1;
 yVal[0][3][2] = 1;

 for (int a = 0; a < NUMBER_ARB; a++) {
 yVal[a][0][0] = 1;
 yVal[a][1][1] = 1;
 yVal[a][2][2] = 1;
 yVal[a][3][0] = 1;
 yVal[a][3][1] = 1;
 yVal[a][3][2] = 1;   
 }
 
 //setting z values
 count = (NUMBER_ARB*input.nEdges) + (NUMBER_ARB*input.nEdges*input.nEdges);
 val = count;
 for (int a = 0; a < NUMBER_ARB; a++) {
   for (int b = 0; b < input.nNodes; b++) {
     for (int c = 0; c < input.nNodes; c++) {
       // switch (count) {
       // case (val + 0):
       // 	 startVal.add(1);
       // 	 break;
       // case (val + 2):
       // 	 startVal.add(1);
       // 	 break;
       // case (val + 4):
       // 	 startVal.add(1);
       // 	 break;
       // case (val + 9):
       // 	 startVal.add(1);
       // 	 break;
       // case (val + 10):
       // 	 startVal.add(1);
       // 	 break;
       // case (val + 12):
       // 	 startVal.add(1);
       // 	 break;
       // case (val + 16):
       // 	 startVal.add(1);
       // 	 break;
       // case (val + 19):
       // 	 startVal.add(1);
       // 	 break;
       // case (val + 20):
       // 	 startVal.add(1);
       // 	 break;
       // case (val + 25):
       // 	 startVal.add(1);
       // 	 break;
       // case (val + 26):
       // 	 startVal.add(1);
       // 	 break;
       // case (val + 29):
       // 	 startVal.add(1);
       // 	 break;
	 
       // default:
       // 	 startVal.add(0);
       // }       
       // //startVal[count] = 0;
       // 	count++;

       zVal[a][b][c] = 0;
     }
   }
 }

 for (int a = 0; a < NUMBER_ARB; a++) {
 zVal[a][0][0] = 1;
 zVal[a][0][2] = 1;
 zVal[a][0][4] = 1;
 zVal[a][1][1] = 1;
 zVal[a][1][2] = 1;
 zVal[a][1][4] = 1;
 zVal[a][2][0] = 1;
 zVal[a][2][3] = 1;
 zVal[a][2][4] = 1;
 zVal[a][3][1] = 1;
 zVal[a][3][2] = 1;
 zVal[a][3][5] = 1;
 }
 
 // count = (NUMBER_ARB*input.nEdges) + (NUMBER_ARB*input.nEdges*input.nEdges);
 // startVal[count + 0] = 1;
 // startVal[count + 2] = 1;
 // startVal[count + 4] = 1;
 // startVal[count + 9] = 1;
 // startVal[count + 10] = 1;
 // startVal[count + 12] = 1;
 // startVal[count + 16] = 1;
 // startVal[count + 19] = 1;
 // startVal[count + 20] = 1;
 // startVal[count + 25] = 1;
 // startVal[count + 26] = 1;
 // startVal[count + 29] = 1;

 //setting xSol values
 // count = (NUMBER_ARB*input.nEdges) + (NUMBER_ARB*input.nEdges*input.nEdges)
 //   + (NUMBER_ARB*input.nNodes*input.nNodes);
 // startVal[count + 0] = 1;
 // startVal[count + 1] = 1;
 // startVal[count + 2] = 1;
 // startVal[count + 3] = 0;
 
 // startVal.add(1);
 // startVal.add(1);
 // startVal.add(1);
 // startVal.add(0);

 xSolVal[0] = 1;
 xSolVal[1] = 1;
 xSolVal[2] = 1;
 xSolVal[3] = 0;

 count = 0;
 for (int a = 0; a < NUMBER_ARB; a++) {
   for (int b = 0; b < input.nEdges; b++) {
     //startVar.add(cplexLP->x[a][b], startVal[count]);
     startVar.add(cplexLP->x[a][b]);
     startVal.add(xVal[a][b]);
     count++;
   }
 }

 count = NUMBER_ARB*input.nEdges;
 for (int a = 0; a < NUMBER_ARB; a++) {
   for (int b = 0; b < input.nEdges; b++) {
     for (int c = 0; c < input.nEdges; c++) {
       //startVar.add(cplexLP->y[a][b][c], startVal[count]);
       startVar.add(cplexLP->y[a][b][c]);
       startVal.add(yVal[a][b][c]);
	count++;       
     }
   }
 }

 count = (NUMBER_ARB*input.nEdges) + (NUMBER_ARB*input.nEdges*input.nEdges);
 for (int a = 0; a < NUMBER_ARB; a++) {
   for (int b = 0; b < input.nNodes; b++) {
     for (int c = 0; c < input.nNodes; c++) {
       //startVar.add(cplexLP->z[a][b][c], startVal[count]);
       startVar.add(cplexLP->z[a][b][c]);
       startVal.add(zVal[a][b][c]);
	count++;       
     }
   }
 }
 
 count = (NUMBER_ARB*input.nEdges) + (NUMBER_ARB*input.nEdges*input.nEdges)
   + (NUMBER_ARB*input.nNodes*input.nNodes);
 for (int a = 0; a < NUMBER_ARB; a++) {
   //startVar.add(cplexLP->xSol[a], startVal[count]);
   startVar.add(cplexLP->xSol[a]);
   startVal.add(xSolVal[a]);
   count++;
 } 
 
 //cplexLP->solver->addMIPStart(startVar,startVal, IloCplex::MIPStartAuto,  "secondMIPStart");
 cplexLP->solver->addMIPStart(startVar,startVal, IloCplex::MIPStartCheckFeas,  "secondMIPStart");
 //cplexLP->solver->addMIPStart(startVar,startVal, IloCplex::MIPStartSolveFixed,  "MIPStartSolveFixed");
 startVar.end();
 startVal.end();


 /****************/
      	 // cout << "Building constraint list ... " << flush; 
      	 // IloConstraintArray cons(cplexLP->env); 
      	 // IloNumArray prefs(cplexLP->env); 
      	 // for (IloModel::Iterator it(cplexLP->solver->getModel()); it.ok(); ++it)
	 // { 
      	 //   IloExtractable e = *it; 
      	 //   if (e.isVariable()) { 
      	 //     IloNumVar v = e.asVariable(); 
      	 //     if (v.getLB() > -IloInfinity) { 
      	 //       cons.add(IloBound(v, IloBound::Lower)); 
      	 //       prefs.add(1.0); 
      	 //     } 
      	 //     if (v.getUB() < IloInfinity) { 
      	 //       cons.add(IloBound(v, IloBound::Upper)); 
      	 //       prefs.add(1.0); 
      	 //     } 
      	 //   } else if (e.isConstraint()) { 
      	 //     cout << "cons: " << e.asConstraint() << endl;
      	 //     cons.add(e.asConstraint()); 
      	 //     prefs.add(1.0); 
      	 //   } 
      	 // } 
      	 // cout << cons.getSize() << " elements." << endl; 
      	 // // Refine the conflict. std::cout << "Refine the conflict ..." << std::endl;
	 // IloInt index = cplexLP->solver->getMIPStartIndex("MIPStartSolveFixed");
	 // cout << "index da mip start: " << index << endl;
	 // if (!cplexLP->solver->refineMIPStartConflict(index, cons, prefs)) {
	 // //if ( !cplexLP->solver->refineConflict(cons, prefs) ) { 
      	 //   cout << "No conflict found!" << endl; 
      	 // } else { 
      	 //   // Print out minimal conflict. 
      	 //   cout << "Conflict found. Minimal conflict is:" << endl; 
      	 //   int count = 0; 
      	 //   for (IloInt i = 0; i < cons.getSize(); ++i) { 
      	 //     if (cplexLP->solver->getConflict(cons[i]) == IloCplex::ConflictMember) { 
      	 //       cout << " " << cons[i] << endl; 
      	 //       ++count; 
      	 //     } 
      	 //   } 
      	 //   cout << count << " constraints in minimal conflict." << endl; 
      	 // } 
 /****************/
 
}

void createLPFormulation(IloEnv env) {
  if (DEBUG == 10)
    cout << "dentro de createLPFormulation" << endl;
  
  int j, l, k, a;
  char varName[100];
  // IloNumVarArray2 x2 = IloNumVarArray2(cplexLP->env, NUMBER_ARB);
  // x2[0] = IloNumVarArray(cplexLP->env, input.nEdges, 0, 1, ILOINT);
  // sprintf(varName, "teste");
  // x2[0][0].setName(varName);
  // if (DEBUG == 10)
  //   cout << "testando x2: " << varName << endl;  
  
    // cplexLP->xSol = IloNumVarArray(cplexLP->env, input.nEdges, 0, 1, ILOINT);
    // cplexLP->x = IloNumVarArray2(cplexLP->env, NUMBER_ARB);
    // cplexLP->y = IloNumVarArray3(cplexLP->env, NUMBER_ARB);
    // cplexLP->z = IloNumVarArray3(cplexLP->env, NUMBER_ARB);

    cplexLP->xSol = IloNumVarArray(cplexLP->env, input.nEdges, 0, 1, ILOINT);
    cplexLP->x = IloNumVarArray2(cplexLP->env, NUMBER_ARB);
    cplexLP->y = IloNumVarArray3(cplexLP->env, NUMBER_ARB);
    cplexLP->z = IloNumVarArray3(cplexLP->env, NUMBER_ARB);

  
   for (a = 0; a < NUMBER_ARB; a++) {
     cplexLP->x[a] = IloNumVarArray(cplexLP->env, input.nEdges, 0, 1, ILOINT);
     cplexLP->z[a] = IloNumVarArray2(cplexLP->env, input.nNodes);
     cplexLP->y[a] = IloNumVarArray2(cplexLP->env, input.nEdges);
     
     for (k = 0; k < input.nNodes; k++) {
       cplexLP->z[a][k] = IloNumVarArray(cplexLP->env, 2*input.nEdges, 0, 1, ILOINT);
     }

     for (l = 0; l < input.nEdges; l++) {
       cplexLP->y[a][l] = IloNumVarArray(cplexLP->env, input.nEdges, 0, 1, ILOINT);
     }
     
   }
  if (DEBUG == 10)
    cout << "criou os arrays x, y, z" << endl;
   

   for (a = 0; a < NUMBER_ARB; a++) {
     FOR_EACH_EDGE_e 
       sprintf(varName, "x.%d.%d_%d",  a, e->u, e->v);
       if (DEBUG == 10)
	cout << "antes de adicionar o nome da variavel x: " << varName << endl;
       cplexLP->x[a][e->label].setName(varName);
       sprintf(varName, "xSol.%d_%d",  e->u, e->v);
       cplexLP->xSol[e->label].setName(varName);
       if (DEBUG == 10)
	cout << "depois de adicionar o nome da variavel x: " << varName << endl;
       FOR_EACH_EDGE_f
       //for(j = 0; j < input.nEdges; j++) {
	 //Edge* f = input.edges[j];
	 sprintf(varName, "y.%d.%d_%d.%d_%d",  a, e->u, e->v, f->u, f->v);
	 cplexLP->y[a][e->label][f->label].setName(varName);
	 if (DEBUG == 11)
	   cout << "variavel " << varName << ", pos: " << f->label << endl;
       }
       cplexLP->model->add(cplexLP->y[a][e->label]);
	 if (DEBUG == 10)
	   cout << "depois de adicionar a variavel y ao model" << endl;       
     }
     if (DEBUG == 10)
	cout << "antes de adicionar a variavel x" << endl;
   //cplexLP->model->add(cplexLP->y[a]);
     cplexLP->model->add(cplexLP->x[a]);
     if (DEBUG == 10)
	cout << "depois de adicionar a variavel x" << endl;

   
     FOR_EACH_NODE_u
     if (DEBUG == 10)
       cout << "dentro do loop com arco, u: " << u->label << endl;
       FOR_EACH_ARC_pair
	if (DEBUG == 10)
	  cout << "label do arco: " << pair->label << endl;
//cout << "id da origem do arco: " << digraph.id(digraph.source(*(pair->in))) << endl;
	int source = nodeLabelMap[digraph.source(pair->in)];
	if (DEBUG == 10)
	  cout << "depois de source: " << digraph.id(digraph.source(pair->in)) << endl;
	int tar = nodeLabelMap[digraph.target(pair->in)];
	if (DEBUG == 10)
	  cout << "depois de target: " << digraph.id(digraph.target(pair->in)) << endl;
	Edge* edge = edgeMap[pair->in];
	sprintf(varName, "z.%d.%d.%d_%d", a, u->label, source, tar);
	if (DEBUG == 11)
	  cout << "variavel " << varName << ", pos: " << edge->label*2 << endl;

	cplexLP->z[a][u->label][edge->label*2].setName(varName);
	sprintf(varName, "z.%d.%d.%d_%d", a, u->label, tar, source);
	cplexLP->z[a][u->label][edge->label*2+1].setName(varName);
	if (DEBUG == 11)
	  cout << "variavel " << varName << ", pos: " << edge->label*2+1 << endl;

       }
       cplexLP->model->add(cplexLP->z[a][u->label]);
     }
//cplexLP->model->add(cplexLP->z[a]);
   }
   cplexLP->model->add(cplexLP->xSol);
     if (DEBUG == 10)
	cout << "depois de adicionar a variavel xSol" << endl;

   IloExpr minExpr(cplexLP->env);
   FOR_EACH_EDGE_e
	minExpr += cplexLP->xSol[e->label] * e->cost;

	IloExpr spanExpr(cplexLP->env);
	for (a = 0; a < NUMBER_ARB; a++) {
	  FOR_EACH_EDGE_f
	    spanExpr += cplexLP->y[a][e->label][f->label] * f->cost;
	  }
	}
	cplexLP->model->add(spanExpr <=
			   3 * input.strFactor * input.dist[e->u][e->v]);
	spanExpr.end();
   }
   cplexLP->model->add(IloMinimize(cplexLP->env, minExpr));
   minExpr.end();


   for (a = 0; a < NUMBER_ARB; a++) {
	FOR_EACH_NODE_u
	  int l1 = u->label;
	  IloExpr rootInDegree(cplexLP->env);	  
	  FOR_EACH_ARC_pair
	    int l2 = nodeLabelMap[digraph.target(pair->in)];
	    Edge* edge = edgeMap[pair->in]; 
	    if (l1 == l2) {
	      rootInDegree += cplexLP->z[a][l1][edge->label*2];
	    } else {
	      l2 = nodeLabelMap[digraph.target(pair->out)];
	      if (l1 == l2) {
		edge = edgeMap[pair->out];
		rootInDegree += cplexLP->z[a][l1][edge->label*2+1];		
	      }
	    }
	  }
	  cplexLP->model->add(rootInDegree == 0);
	  rootInDegree.end();

	  FOR_EACH_NODE_v
	    int l2 = v->label;
	    if (l1 != l2) {
	      IloExpr inDegree(cplexLP->env);
	      FOR_EACH_ARC_pair
		int l3 = nodeLabelMap[digraph.target(pair->in)];
		Edge* edge = edgeMap[pair->in];
		if (l2 == l3) {
		  inDegree += cplexLP->z[a][l1][edge->label*2];
		} else {
		  l3 = nodeLabelMap[digraph.target(pair->out)];
		  if (l2 == l3) {
		    edge = edgeMap[pair->out];
		    inDegree += cplexLP->z[a][l1][edge->label*2+1];
		  }
		}
	      }
	      cplexLP->model->add(inDegree == 1);
	      inDegree.end();
	    }
	  }
	}
	FOR_EACH_EDGE_e
	  FOR_EACH_NODE_u
	    IloExpr relateArbs(cplexLP->env);

	    relateArbs = cplexLP->x[a][e->label];
	    cplexLP->model->add(relateArbs ==
		    (cplexLP->z[a][u->label][e->label*2] +
		     cplexLP->z[a][u->label][e->label*2+1]));
	    relateArbs.end();
	  }

	  IloExpr defFinalGraph(cplexLP->env);
	  defFinalGraph = cplexLP->x[a][e->label];
	  cplexLP->model->add(cplexLP->xSol[e->label] >= defFinalGraph);
	  defFinalGraph.end();

	  FOR_EACH_ARC_pair
	    Edge* edge = edgeMap[pair->in];
//FOR_EACH_EDGE_f
		IloExpr ijDiff(cplexLP->env);
		ijDiff = cplexLP->y[a][e->label][edge->label];
		cplexLP->model->add(
				    (cplexLP->z[a][e->u][edge->label*2] -
		     cplexLP->z[a][e->v][edge->label*2]) <= ijDiff);
		IloExpr ijSum(cplexLP->env);
		ijSum = cplexLP->y[a][e->label][edge->label];
		cplexLP->model->add(ijSum <=
				    (cplexLP->z[a][e->u][edge->label*2] +
		     cplexLP->z[a][e->v][edge->label*2]));
		if (DEBUG == 11) {
		  cout << "label: " << edge->label << ", label*2: " << edge->label*2 << ", ijDiff: " << cplexLP->z[a][e->u][edge->label*2].getName() << " - " << cplexLP->z[a][e->v][edge->label*2].getName() << " <= " << cplexLP->y[a][e->label][edge->label].getName() << endl;
		}

		IloExpr jiDiff(cplexLP->env);
		jiDiff = cplexLP->y[a][e->label][edge->label];
		cplexLP->model->add(
				    (cplexLP->z[a][e->u][edge->label*2+1] -
		     cplexLP->z[a][e->v][edge->label*2+1]) <= jiDiff);
		IloExpr jiSum(cplexLP->env);
		jiSum = cplexLP->y[a][e->label][edge->label];
		cplexLP->model->add(jiSum <=
				    (cplexLP->z[a][e->u][edge->label*2+1] +
		     cplexLP->z[a][e->v][edge->label*2+1]));
		ijDiff.end();
		ijSum.end();
		jiDiff.end();
		jiSum.end();
//}
	  }
	}
   }

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

void read(int argc, char **argv, IloEnv env) {

  ifstream costs(argv[1]);



  // char a[100];
  //           while(!costs.eof())

  //           {

  //                       costs >> a;

  //                       cout << a << endl;

  //           }
  
  //IloNumArray2 edgeCost2(env);

  //input = (InputData*) malloc(sizeof(InputData));

  input.edgeCost = IloNumArray2(env);
  //costs >> edgeCost2;
  costs >> input.edgeCost;
  costs.close();
  input.strFactor = atof(argv[2]);
  input.nNodes = input.edgeCost.getSize();
  input.nEdges = getNEdges();
  if (DEBUG == 11) {
    cout << "tam do vetor cost: " << input.edgeCost.getSize() << endl;
    cout << "getNEdges: " << getNEdges() << ", nEdges: " << input.nEdges << endl;
  }
  

  //costs.close();
}

void setCplexParam() {
  IloNum tol = cplexLP->solver->getParam(IloCplex::EpInt);
  cplexLP->solver->setParam(IloCplex::CutUp, cplexLP->ub);
  cplexLP->solver->setParam(IloCplex::CutLo, cplexLP->lb);
  cplexLP->solver->setParam(IloCplex::TiLim, TIME_LIMIT);
  cplexLP->solver->exportModel("model.lp");
}

int main(int argc, char* argv[]) {
  time_t start, end;
  double total_time;
  start = clock();
  ofstream timeOutFile;
  ofstream edgesOutFile;  
  int noSolutionFlag = 0;  
  
  IloEnv env;
  IloModel mod(env, "MWSP");
  IloCplex solver(mod);

  //IloNumVarArray2 xVal(env, NUMBER_ARB);;

  try {
  ifstream costs(argv[1]);
  // IloNumArray2 edgeCost2(env);
  // costs >> edgeCost2;
  // costs.close();
  // if (DEBUG == 10)
  //   cout << "antes do read, tam do vetor cost: " << edgeCost2.getSize() << endl;
    
    read(argc, argv, env);
  if (DEBUG == 10)
    cout << "depois de read, tam  vetor cost: " << input.edgeCost.getSize() << endl;
    
  
    initVariables(env, &mod, &solver);
    if (DEBUG == 10)
      cout << "ja inicializou as variaveis" << endl;
    createLPFormulation(env);

    setCplexParam();
    testSolution();

    if ( cplexLP->solver->solve() ) {
      end = clock();

      IloAlgorithm::Status solStatus= cplexLP->solver->getStatus();

      if ( solStatus == IloAlgorithm::Optimal ) {
	
	cout << "Solution is optimal" << endl;

	// IloNumArray sol(env, input.nEdges);
	// IloNumArray3 ySol(env, NUMBER_ARB);
	// IloNumArray2 xSol(env, NUMBER_ARB);
	// sol = IloNumArray(env);
	// cplexLP->solver->getValues(sol, cplexLP->xSol);
	// cout << "numero de edges: " << input.nEdges << endl;
	// for (int i = 0; i < NUMBER_ARB; i++) {
	//   ySol[i] = IloNumArray2(env, input.nEdges);
	//   xSol[i] = IloNumArray(env, input.nEdges);
	//   cplexLP->solver->getValues(xSol[i], cplexLP->x[i]);
	//   for (int j = 0; j < input.nEdges; j++) {
	//     ySol[i][j] = IloNumArray(env, input.nEdges);
	//     cplexLP->solver->getValues(ySol[i][j], cplexLP->y[i][j]);
	//   }
	// }
	
	// for (int i = 0; i < input.nEdges; i++) {
	//   if ( sol[i] > 1e-03 ) {
	//     cout << cplexLP->xSol[i].getName() << " = " << sol[i] << endl;
	//   }
	//   for (int j = 0; j < NUMBER_ARB; j++) {
	//     if ( xSol[j][i] > 1e-03 ) {
	//       //cout << cplexLP->x[j][i].getName() << " = " << xSol[j][i] << endl;
	//     }
	//     for (int k = 0; k < input.nEdges; k++) {
	//       if ( ySol[j][i][k] > 1e-03 ) {
	// 	cout << cplexLP->y[j][i][k].getName() << " = " << ySol[j][i][k] << endl;
	//       }
	//     }
	//   }
	// }

	
      } else {
	cout << "Solution status is not Optimal" << endl;
      }
    } else {
      noSolutionFlag = 1;
      end = clock();
      cout << "No solution available" << endl;

      	 /*-----------------------*/
      /*
      	 // No feasible solution was found. Perform conflict analysis. 
      	 // We allow any constraint to be part of the conflict. 
      	 cout << "Model is infeasible. Running conflict analysis." << endl;
      // Build a list of constraints that can be part of the conflict. 
      // Since bounds of variables may also render the model infeasible 
      // we also add bounds to the set of constraints that are considered 
      // for a conflict. 
      	 cout << "Building constraint list ... " << flush; 
      	 IloConstraintArray cons(env); 
      	 IloNumArray prefs(env); 
      	 for (IloModel::Iterator it(cplexLP->solver->getModel()); it.ok(); ++it)
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
      	     env.out() << "cons: " << e.asConstraint() << endl;
      	     cons.add(e.asConstraint()); 
      	     prefs.add(1.0); 
      	   } 
      	 } 
      	 cout << cons.getSize() << " elements." << endl; 
      	 // Refine the conflict. std::cout << "Refine the conflict ..." << std::endl; 
      	 if ( !cplexLP->solver->refineConflict(cons, prefs) ) { 
      	   cout << "No conflict found!" << endl; 
      	 } else { 
      	   // Print out minimal conflict. 
      	   cout << "Conflict found. Minimal conflict is:" << endl; 
      	   int count = 0; 
      	   for (IloInt i = 0; i < cons.getSize(); ++i) { 
      	     if (cplexLP->solver->getConflict(cons[i]) == IloCplex::ConflictMember) { 
      	       cout << " " << cons[i] << endl; 
      	       ++count; 
      	     } 
      	   } 
      	   cout << count << " constraints in minimal conflict." << endl; 
      	 } 
      */
      	 /*-----------------------*/

      
    }
    
  }
   catch (const IloException& e) {
      cerr << "Exception caught: " << e << endl;
   }
   catch (...) {
      cerr << "Unknown exception caught!" << endl;
   }

  env.end();

   //armazenando estatísticas
   total_time = (double)( end - start )/(double)CLOCKS_PER_SEC ;
   cout << "total time: " << total_time << endl;;
   //timeOutFile.open (argv[3], ios::out | ios::app);
   timeOutFile.open (argv[3], ios::out);
   //cout << argv[3] << endl;
   if (total_time >= TIME_LIMIT) {
     timeOutFile << 0 << endl;
   } else if (noSolutionFlag) {
     timeOutFile << -1 << endl;
   } else
     timeOutFile << total_time << endl;
   timeOutFile.close();

   //edgesOutFile.open (argv[4], ios::out | ios::app);
   edgesOutFile.open (argv[4], ios::out);
   edgesOutFile << input.nEdges << endl;
   edgesOutFile.close();

   
   return 0;
  
  
}
