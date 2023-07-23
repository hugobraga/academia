package coverage;

import dataStructure.ALNode;
import dataStructure.AdjList;
import dataStructure.AdjMatrix;
import dataStructure.Weight;

public class APITCAlgorithm extends DirectedGraphTCAlgorithm {

	public APITCAlgorithm(Weight w, int size) {
		super(w, size);
		// TODO Auto-generated constructor stub
	}
	
	@Override
	protected void execute(AdjMatrix globalAdjMatrix) {
		AdjList tempAdjl = new AdjList(globalOptAdjl.getNumberOfVertices());
		
		for (int u = 0; u < globalAdjl.getNumberOfVertices(); u++) {			
			for (ALNode nodeV = globalAdjl.getList(u); nodeV != null; nodeV = nodeV.getProx()) {
				if (nodeV.getCoverage() == 0) {
					//the edge's weight is the buckhart coverage
					tempAdjl.insertEdge (u, nodeV.getVid(), nodeV.getBuckhartCoverage(), 0, nodeV.getIS());
				}
			}
		}
    	
    	for (int i = 0; i < tempAdjl.getNumberOfVertices(); i++) {
    	    ALNode alnode = tempAdjl.getList(i); //alnode is v
    	    int id1, id2, id1Other, id2Other;
    	    double buckhartCov = -1;
    	    double buckhartCovOther = -1;
    	    int is1, is1Other;
    	    id1 = id2 = is1 = id1Other = id2Other = is1Other = -1;
    	    while (alnode != null) {
    	    	ALNode relayNode = tempAdjl.getList(i); //relaynode is w
    	    	while (relayNode != null) {
    	    		if (alnode.getVid() != relayNode.getVid()) { //v != w
    	    			double covWV = -1;
    	    			for (int j = 0; j < tempAdjl.getNumberOfVertices(); j++) {
    	    				if (tempAdjl.getList(j).getVid() == relayNode.getVid()) {
    	    					ALNode temp = tempAdjl.getList(0); //temp corresponde ao vizinho de w
    	    					//double covWV = -1;
    	    					while (temp != null) {
    	    						if (temp.getVid() == alnode.getVid()) {
    	    							covWV = temp.getWeight();
    	    							is1Other = temp.getIS();
    	    							break;
    	    						}
    	    						temp = temp.getProx();
    	    					}
    	    					break;
    	    				}
    	    			}
    	    			
    	    			if ((relayNode.getWeight() + covWV) < alnode.getWeight()) {
    	    				id1 = i;
    	    				id2 = relayNode.getVid();
    	    				buckhartCov = relayNode.getWeight();
    	    				
    	    				id1Other = id2;
    	    				id2Other = alnode.getVid();
    	    				buckhartCovOther = covWV;
    	    				break;
    	    			}
    	    		}
    	    		relayNode = relayNode.getProx();
    	    	}
    	    	
    	    	if (buckhartCov != -1) {
    	    		globalOptAdjl.insertEdge (id1, id2, buckhartCov, 0, is1);
    	    		globalOptAdjl.insertEdge (id1Other, id2Other, buckhartCovOther, 0, is1Other);
    	    	} else {
    	    		globalOptAdjl.insertEdge (i, alnode.getVid(), alnode.getWeight(), 0, alnode.getIS());
    	    	}
    	    	
    	    	alnode = alnode.getProx();
    	    }
    	}
	}	

}
