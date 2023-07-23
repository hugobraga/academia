package coverage;

import dataStructure.AdjMatrix;
import dataStructure.Weight;
import support.Edge;

public class RNGTCAlgorithm extends UndirectedGraphTCAlgorithm {

	public RNGTCAlgorithm(Weight w, int size) {
		super(w, size);
		// TODO Auto-generated constructor stub
	}
	
	@Override
	protected void execute(AdjMatrix globalAdjMatrix) {
		//RNG doesn't need the set edges ordered by the interference number
    	while (!edges.isEmpty()) {
    		Edge key = (Edge)edges.poll();
    		if (key.getCoverage() > 0)
    			continue;
    		
			globalOptAdjl.insertEdge (key.id1, key.id2, key.getIS1(), 0, key.getIS1());
			globalOptAdjl.insertEdge (key.id2, key.id1, key.getIS2(), 0, key.getIS2());
    	}
	}

}
