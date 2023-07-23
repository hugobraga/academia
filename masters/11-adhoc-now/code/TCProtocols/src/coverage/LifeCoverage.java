package coverage;

import java.util.HashSet;
import java.util.Set;

import radios.RadioParameters;

import dataStructure.ALNode;
import dataStructure.AdjList;
import dataStructure.Weight;


public class LifeCoverage extends Coverage {

	public LifeCoverage(Weight w) {
		super(w);
		listCov = new UndirectedEdgeInterference();
		// TODO Auto-generated constructor stub
	}

    /* 
     * Get the (standard, based on the initial definition of coverage) coverage in the transmition from node i to node j
     * Observes that we consider the real distance due to radio power levels.
    */	
	@Override
	public Set<Integer> getCoverage(int i, int j, AdjList adjl) {
		return Coverage.getAssymetricBuckhartCoverage(i, j, adjl, weight);
	}

	@Override
	public void setCoverage(AdjList adjl) {
		listCov.setListCoverage(adjl, new LifeCoverage(weight));
		
	}
}
