package coverage;

import java.util.HashSet;
import java.util.Set;

import radios.RadioParameters;
//import tudo.OurProtocol;

import dataStructure.ALNode;
import dataStructure.AdjList;
import dataStructure.Weight;


public class APICoverage extends Coverage {
	public APICoverage(Weight w) {
		super(w);
		listCov = new DirectedEdgeInterference();
		// TODO Auto-generated constructor stub
	}	

    private double getAPIWeight(int i, int j) {
    	//double dist = RadioParameters.getRealDistance(weight.distance(i, j));
    	//since this weigh is based on GG weight, I don't consider the real distance due to geometrical characteristics
    	double dist = weight.distance(i, j);
    	return Math.pow(dist, RadioParameters.npathLossExp);
    } 
    
	@Override
	public Set<Integer> getCoverage(int u, int v, AdjList adjl) {
    	double dist = weight.distance(u, v);
    	Set<Integer> cov = new HashSet<Integer>();
    	for (ALNode w = adjl.getList(u); w != null; w = w.getProx()) {
    		if (w.getVid() == v)
    			continue;
    		double distUW = weight.distance(u, w.getVid());
    	    if (distUW <= dist) {
    	    	if (getAPIWeight(u, w.getVid()) + getAPIWeight(w.getVid(), v) <= getAPIWeight(u, v)) {
    	    		cov.add(w.getVid());
    	    	}
    	    }
    	}
    	return cov;
	}

	@Override
	public void setCoverage(AdjList adjl) {
		Coverage apiCov = new APICoverage(weight);
		listCov.setListCoverage(adjl, apiCov);
	}
}
