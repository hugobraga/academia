package coverage;

import java.util.HashSet;
import java.util.Set;

import dataStructure.ALNode;
import dataStructure.AdjList;
import dataStructure.Weight;

public class IRNGCoverage extends Coverage {

	public IRNGCoverage(Weight w) {
		super(w);
		listCov = new DirectedEdgeInterference();
		DirectedEdgeInterference nodBCov = (DirectedEdgeInterference)listCov;
		nodBCov.unSetBuckhartCoverage();
	}

	/**
	 * Return -1 if rank(xy) < rank(xz) and
	 * 1 if rank(xy) > rank(xz)
	 */
	private int rank(int x, int y, int z, int aIC, int bIC, boolean former) {
		int a1, a2, b1, b2;
		if (former) {
			a1 = b1 = x;
			a2 = y;
			b2 = z;
		} else {
			a1 = x;
			a2 = b1 = y;
			b2 = z;			
		}
		if (aIC > bIC)
			return 1;
		else if ((aIC == bIC) && (Math.max(a1, a2) > Math.max(b1, b2)))
			return 1;
		else if ((aIC == bIC) && (Math.max(a1, a2) == Math.max(b1, b2)) && (Math.min(a1, a2) < Math.min(b1, b2)))
			return 1;
		else 
			return -1;
	}
	
	@Override
	public Set<Integer> getCoverage(int u, int v, AdjList adjl) {
    	int uvIC = -1;
    	Set<Integer> cov = new HashSet<Integer>();

    	uvIC = Coverage.getBuckhartCoverage(u, v, adjl, weight).size();
    	
    	for (ALNode wAln = adjl.getList(u); wAln != null; wAln = wAln.getProx()) {
    		if (wAln.getVid() == v)
    			continue;

    		int vwIC = Coverage.getBuckhartCoverage(v, wAln.getVid(), adjl, weight).size();
    		int uwIC = Coverage.getBuckhartCoverage(u, wAln.getVid(), adjl, weight).size();
    		
    		if ((rank(u, v, wAln.getVid(), uvIC, uwIC, true)== 1) && (rank(u, v, wAln.getVid(), uvIC, vwIC, false)== 1))
    			cov.add(wAln.getVid());
    	}
    	return cov;
	}

	@Override
	public void setCoverage(AdjList adjl) {
		listCov.setListCoverage(adjl, new IRNGCoverage(weight));
	}

}
