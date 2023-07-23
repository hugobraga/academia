package tcAlgorithm;

import coverage.Coverage;
import coverage.DirectedGraphTCAlgorithm;
import dataStructure.AdjMatrix;
import dataStructure.PositionTable;
import dataStructure.Weight;

public abstract class DirectedGraphSimpleTCProtocol extends SimpleTCProtocol {
	//AdjList globalAdjl;

	DirectedGraphSimpleTCProtocol(int nsize, PositionTable npt, double nmprange,
			Weight nedgeWeight) {
		super(nsize, npt, nmprange, nedgeWeight);
		// TODO Auto-generated constructor stub
	}

	//@Override
    public boolean executeTCAlgorithm(Coverage coverage, DirectedGraphTCAlgorithm tcAlg) {
		if (!buildLocalInformation())
			return false;
		
    	//set the coverage for each edge in the local neighborhood
    	coverage.setCoverage(globalMaxPowerAdjl); //for the XTC algorithm, it's not necessary		
    	
    	globalMaxPowerAdjMatrix = new AdjMatrix(size);
    	globalMaxPowerAdjMatrix.fillMatrixWithISFromAdjList(globalMaxPowerAdjl);
    	
    	globalAdjMatrix = new AdjMatrix(size);
    	tcAlg.setGlobalAdjList(globalMaxPowerAdjl);
    	tcAlg.executeAndFillMatrixWithIS(globalAdjMatrix);
    	globalOptAdjl = tcAlg.getOptimizedSetOfNeighbours();
    	
    	//calculating PICS spanning factor
    	globalAdjMatrix.minCostPaths();
    	globalMaxPowerAdjMatrix.minCostPaths();
    	calcPICS();
    	return true;
    }

  	
	
	public abstract boolean execute();
	
	public abstract String getName();

}
