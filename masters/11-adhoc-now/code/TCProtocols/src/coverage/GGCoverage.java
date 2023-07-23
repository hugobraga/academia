package coverage;

import java.util.HashSet;
import java.util.Set;

import dataStructure.ALNode;
import dataStructure.AdjList;
import dataStructure.Weight;


public class GGCoverage extends Coverage {

	public GGCoverage(Weight w) {
		super(w);
		listCov = new UndirectedEdgeInterference();
		// TODO Auto-generated constructor stub
	}	

    /* 
     * Get the GG coverage in the transmition from node i to node j based on the node sender-centric.
    */   	
	@Override
	public Set<Integer> getCoverage(int i, int j, AdjList adjl) {
    	double dist = weight.distance(i, j);
    	Set<Integer> cov = new HashSet<Integer>();
    	for (ALNode aln = adjl.getList(i); aln != null; aln = aln.getProx()) {
    		if (aln.getVid() == j)
    			continue;
    		double distUW = weight.distance(i, aln.getVid());
    	    if (distUW <= dist) {
    	    	double distVW = weight.distance(aln.getVid(), j);
    	    	if (Math.pow(dist, 2) >= (Math.pow(distUW, 2) + Math.pow(distVW, 2))) {
    	    		cov.add(aln.getVid());
    	    	}
    	    }
    	}
    	return cov;
	}

	@Override
	public void setCoverage(AdjList adjl) {
		listCov.setListCoverage(adjl, this);
	}
}
