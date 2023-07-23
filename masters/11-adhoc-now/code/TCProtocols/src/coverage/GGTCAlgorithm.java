package coverage;

import dataStructure.AdjMatrix;
import dataStructure.Weight;

import support.Edge;

public class GGTCAlgorithm extends UndirectedGraphTCAlgorithm {

	public GGTCAlgorithm(Weight w, int size) {
		super(w, size);
		// TODO Auto-generated constructor stub
	}
	
	@Override
	protected void execute(AdjMatrix globalAdjMatrix) {
		//GG doesn't need the set edges ordered by the interference number
    	while (!edges.isEmpty()) {
    		Edge key = (Edge)edges.poll();
    		if (key.getCoverage() > 0)
    			continue;
    		
			globalOptAdjl.insertEdge (key.id1, key.id2, 0, 0, key.getIS1());
			globalOptAdjl.insertEdge (key.id2, key.id1, 0, 0, key.getIS2());
    	}
	}

}
