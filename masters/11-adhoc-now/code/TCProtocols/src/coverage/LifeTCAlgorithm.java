package coverage;

import dataStructure.AdjMatrix;
import dataStructure.Weight;

import support.Edge;

public class LifeTCAlgorithm extends UndirectedGraphTCAlgorithm {

	public LifeTCAlgorithm(Weight w, int size) {
		super(w, size);
		// TODO Auto-generated constructor stub
	}

	@Override
	protected void execute(AdjMatrix globalAdjMatrix) {
		//until here, globalAdjMatrix is empty and so covAjdl
    	while (!edges.isEmpty()) {
    		Edge key = (Edge)edges.poll();
    		if (!globalOptAdjl.connected(key.id1, key.id2)) {
    			globalOptAdjl.insertEdge (key.id1, key.id2, 0, 0, key.getIS1());
    			globalOptAdjl.insertEdge (key.id2, key.id1, 0, 0, key.getIS2());
    		}
    	}
	}

}
