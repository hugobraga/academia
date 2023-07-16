#ifndef COMMON_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define COMMON_H

/* #include "tree_labels_dist_spanner.h" */

/* extern InputData input; */

#include <list>
#include <map>
#include <ilcplex/ilocplex.h>


//------------lemon---------
#include <lemon/maps.h> //ArcMap
#include <lemon/list_graph.h> //ListDigraph
#include <lemon/dijkstra.h>

using namespace lemon;
using std::list;

typedef ListDigraph Digraph;
typedef Digraph::ArcMap<IloNum> LengthMap;
typedef map<Digraph::Arc,Edge*> EdgeMap;


void findBridges(Digraph& digraph, LengthMap& weight, list<Edge*>& bridges, EdgeMap& edgeMap);

#endif
