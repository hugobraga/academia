package coverage;

import java.util.PriorityQueue;

import support.Edge;

import dataStructure.AdjMatrix;
import dataStructure.Weight;

public abstract class UndirectedGraphTCAlgorithm extends TCAlgorithm {
	PriorityQueue<Edge> edges;	

	public UndirectedGraphTCAlgorithm(Weight w, int size) {
		super(w, size);
		// TODO Auto-generated constructor stub
	}

	protected abstract void execute(AdjMatrix globalAdjMatrix);
	
	public void setEdgesPQ(PriorityQueue<Edge> pq) {
		edges = pq;
	}	

}
