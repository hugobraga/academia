#include "tree_labels_dist_spanner.h"
#include "separate.h"
#include "common.h"

#include <vector>
#include <lemon/preflow.h>
#include <stdlib.h>  //abs
#include <stdio.h> //cout

#include <lemon/list_graph.h> //ListDigraph
#include <lemon/maps.h> //ArcMap

std::vector<int> separate(IloNumArray sol, IloNum tol, int nNodes, int nEdges) {
  NodeMap LNMap;
  ReverseNodeMap NLMap;

  cout << "dentro de separate, sol[3]: " << sol[3] << endl;
  
  Digraph flowDigraph;
  LengthMap flowLength(flowDigraph);
  vector<Digraph::Node> flowNodes;
  Digraph::ArcMap<IloNum> cap(flowDigraph);

  flowNodes.reserve(2 + nNodes + nEdges);
  //s, t
  int sNodeId = 0;
  int tNodeId = 1;
  flowNodes[sNodeId] = flowDigraph.addNode();
  flowNodes[tNodeId] = flowDigraph.addNode();

  //creating arcs between nodes and t
  for (int i = 2; i <= (nNodes+1); i++) {
    flowNodes[i] = flowDigraph.addNode();
    NLMap[flowNodes[i]] = i;

    Digraph::Arc f = flowDigraph.addArc(flowNodes[i], flowNodes[tNodeId]);
    Digraph::Arc g = flowDigraph.addArc(flowNodes[1], flowNodes[tNodeId]);
    flowLength[f] = 1;
    flowLength[g] = 1;
  }

  //double inf = std::numeric_limits<double>::infinity();
  IloNum inf = IloInfinity;
  int start = nNodes + 2;
  int ind = start;
  FOR_EACH_EDGE_e
    //se o valor eh proximo de 0 (pode ser negativo ou positivo)
    cout << "sol[" << e->label << "]: " << sol[e->label] << endl;
    if (abs(sol[e->label]) > tol) {
      flowNodes[ind] = flowDigraph.addNode();
      NLMap[flowNodes[ind]] = ind;      
      //creating arcs between s and edges
      Digraph::Arc f = flowDigraph.addArc(flowNodes[sNodeId], flowNodes[ind]);
      Digraph::Arc g = flowDigraph.addArc(flowNodes[ind], flowNodes[sNodeId]);
      flowLength[f] = abs(sol[e->label]);
      flowLength[g] = abs(sol[e->label]);

      //creating arcs between edges and nodes
      Digraph::Arc fu = flowDigraph.addArc(flowNodes[ind], flowNodes[e->u+2]);
      Digraph::Arc uf = flowDigraph.addArc(flowNodes[e->u+2], flowNodes[ind]);
      flowLength[fu] = inf;
      flowLength[uf] = inf;
      Digraph::Arc fv = flowDigraph.addArc(flowNodes[ind], flowNodes[e->v+2]);
      Digraph::Arc vf = flowDigraph.addArc(flowNodes[e->v+2], flowNodes[ind]);
      flowLength[fv] = inf;
      flowLength[vf] = inf;           

      ind++;
    }
  }

  std::vector<int> SetS;
  /*
    Preflow provides an implementation of Goldberg-Tarjan's preflow 
push-relabel algorithm producing a flow of maximum value in a digraph
   */
  Preflow<Digraph, Digraph::ArcMap<IloNum>> pre(flowDigraph,flowLength,flowNodes[sNodeId],flowNodes[tNodeId]);
  pre.init();
  pre.run();

  if (pre.flowValue() < (nNodes - tol)) {
    Digraph:: NodeMap<bool> cut(flowDigraph);
    pre.minCutMap(cut);
    
    for(Digraph::ArcIt e(flowDigraph); e!=INVALID; ++e) {
      /*Observe que como estou considerando um digrafo e cada aresta do grafo 
original foi transformada em dois arcos, apenas um dos arcos será considerado 
como "aresta" de corte, pois os arcos estao em sentidos contrarios (visto que 
se se o source de um arco estiver dentro de um dos lados do corte, o source do 
outro arco nao estara dentro deste mesmo lado do corte).
       */
      if ((cut[flowDigraph.source(e)] && !cut[flowDigraph.target(e)])) {

	if (NLMap[flowDigraph.target(e)] == tNodeId) {
	  SetS.push_back(NLMap[flowDigraph.source(e)]-2);
	}
      }
    }
  }

  return SetS;

}
