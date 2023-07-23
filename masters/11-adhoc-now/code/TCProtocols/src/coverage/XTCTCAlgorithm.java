package coverage;

import java.util.HashSet;
import java.util.Set;

import dataStructure.ALNode;
import dataStructure.AdjMatrix;
import dataStructure.Weight;

public class XTCTCAlgorithm extends DirectedGraphTCAlgorithm {

	public XTCTCAlgorithm(Weight w, int size) {
		super(w, size);
		// TODO Auto-generated constructor stub
	}

	/**
	 * Check if node w appears before u in vList.
	 * Case yes, return true;
	 * otherwise, return false;
	 */
    private boolean nodeBeforeInList(int w, int u, ALNode vList) {
    	for (ALNode aln = vList; aln != null; aln = aln.getProx()) {
    		if (aln.getVid() == w)
    			return true;
    		else if (aln.getVid() == u)
    			return false;
    	}
    	return true;
    }	
	
	@Override
	protected void execute(AdjMatrix globalAdjMatrix) {
    	//sets of the XTC algorithm description
    	Set<Integer> Nu = new HashSet<Integer>();
    	Set<Integer> NuBar = new HashSet<Integer>();
    	
    	for (int u = 0; u < globalOptAdjl.getNumberOfVertices(); u++) {
	    	for (ALNode nodeV = globalAdjl.getList(u); nodeV != null; nodeV = nodeV.getProx()) {//the neighbors are ordered
	    		//aln is v
	    		for (ALNode nodeW = globalAdjl.getList(nodeV.getVid()); nodeW != null; nodeW = nodeW.getProx()) {
	    			if (nodeW.getVid() == nodeV.getVid())
	    				continue;
	    			if (((Nu.contains(nodeW.getVid())) || (NuBar.contains(nodeW.getVid()))) && (nodeBeforeInList(nodeW.getVid(), u, globalAdjl.getList(nodeV.getVid()))))
	    				NuBar.add(nodeV.getVid());
	    			else {
	    				Nu.add(nodeV.getVid());
	    				globalOptAdjl.insertEdge (u, nodeV.getVid(), 0, 0, nodeV.getIS());
	    			}
	    		}
	    	}
    	}
	}	

}
