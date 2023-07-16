#include "common.h"
#include "tree_labels_dist_spanner.h"

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
      // cplexLP->x[edge->label].setBounds(1, 1);
    }

    Digraph::Arc e = tempDigraph.addArc(nr[s], nr[t]);
    Digraph::Arc f = tempDigraph.addArc(nr[t], nr[s]);
    tempLength[e] = len;
    tempLength[f] = len;
  }
}
