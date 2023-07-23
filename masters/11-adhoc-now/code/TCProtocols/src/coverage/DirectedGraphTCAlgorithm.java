package coverage;

import dataStructure.AdjList;
import dataStructure.AdjMatrix;
import dataStructure.Weight;

public abstract class DirectedGraphTCAlgorithm extends TCAlgorithm {
	AdjList globalAdjl;	

	public DirectedGraphTCAlgorithm(Weight w, int size) {
		super(w, size);
		// TODO Auto-generated constructor stub
	}


	protected abstract void execute(AdjMatrix globalAdjMatrix);
	
	public void setGlobalAdjList(AdjList global) {
		globalAdjl = global;
	}	

}
