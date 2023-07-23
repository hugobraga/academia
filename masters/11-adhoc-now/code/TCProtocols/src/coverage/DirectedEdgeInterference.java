package coverage;

import java.util.Set;

import dataStructure.ALNode;
import dataStructure.AdjList;

public class DirectedEdgeInterference implements ListCoverage {
	private boolean buckCov;
	
	public DirectedEdgeInterference() {
		buckCov = true;
	}
	
	public void unSetBuckhartCoverage() {
		buckCov = false;
	}
	
	public void setALNodeCoverage(ALNode node, int i, int j, AdjList adjl, Coverage coverage) {
		if (buckCov)
			node.setBuckhartCov(Coverage.getBuckhartCoverage(i, j, adjl, coverage.getWeight()));
		node.setCov(coverage.getCoverage(i, j, adjl));
	}
	
	@Override
	public void setListCoverage(AdjList adjl, Coverage coverage) {
		int nv = adjl.getNumberOfVertices();
    	ALNode node;

    	for (int i=0; i < nv; i++) {
    	    node = adjl.getList(i);
    	    while (node != null) {
    	    	int j = node.getVid();
    	    	setALNodeCoverage(node, i, j, adjl, coverage);
    	    	
    	    	node = node.getProx();
    	    }
    	} 
	}

}
