package algorithms;

import java.util.ArrayList;

import support.S2DDelay;
import dataStructure.AdjList;
import dataStructure.AdjMatrix;
import dataStructure.PositionTable;
import dataStructure.Weight;

public class SPT extends DSMDStAlgorithm implements Algorithm {
	
	AdjMatrix arbAdjMatrix;
	
	public SPT(int nsize, double nmprange, int sourceId, int[] terminalsId,
			double[] delaysTerm, PositionTable npt, Weight w) {
		super(nsize, nmprange, sourceId, terminalsId, delaysTerm, npt, w);
		// TODO Auto-generated constructor stub
	}
	
	protected void clearVar() {
		//System.out.println("limpou 1");
		super.clearVar();
	}		
	
	@Override
	public boolean execute(double perc) {
		clearVar();	
		
		preExecute(perc);
    	
    	arbAdjMatrix = new AdjMatrix(size);
		
		arborescence = buildSPT2Terminals(arbAdjMatrix, globalAdjl, terminals);
		// TODO Auto-generated method stub
		return true;
	}
	
	public int getArbMaxOutDegree() {
		return getMaxOutDegree(arborescence);
	}	
	
	public double[] getArbCosts() {
		return getCosts(arbAdjMatrix, terminals);
	}
	
	@Override
	public double[] getTerViolatedCosts() {
		// TODO Auto-generated method stub
		return getCosts(arbAdjMatrix, terminals);
	}
	
	public double[] getViolatedTerSpannerQuant() {
		return getSpannerQuant(terminals);
	}		

	@Override
	public String getName() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public double getSecondPhaseTerminalsCVR(double[] minCVR, double[] maxCVR) {
		double terFinalCosts[] = getCosts(arbAdjMatrix, terminals);
		double terSpannerCosts[] = getSpannerQuant(terminals);
		
		double ratios = 0;
		
		minCVR[0] = -1;
		maxCVR[0] = -1;		
		
		for (int i = 0; i < terFinalCosts.length; i++) {
			double val = terFinalCosts[i]/terSpannerCosts[i];
			if (val > maxCVR[0])
				maxCVR[0] = val;
			if ((val < minCVR[0]) || (minCVR[0] == -1))
				minCVR[0] = val;
			ratios += terFinalCosts[i]/terSpannerCosts[i];	
		}
		
		return (ratios/terFinalCosts.length);
	}

	@Override
	public double getTotalTerminalsCVR(double[] minCVR, double[] maxCVR) {
		
		double terFinalCosts[] = getCosts(arbAdjMatrix, terminals);
		double terSpannerCosts[] = getSpannerQuant();		
		
		minCVR[0] = -1;
		maxCVR[0] = -1;
		double ratios = 0;
		for (int i = 0; i < terFinalCosts.length; i++) {
			double val = terFinalCosts[i]/terSpannerCosts[i];
			if (val > maxCVR[0])
				maxCVR[0] = val;
			if ((val < minCVR[0]) || (minCVR[0] == -1))
				minCVR[0] = val;
			ratios += terFinalCosts[i]/terSpannerCosts[i];
		}
		
		return (ratios/terFinalCosts.length);
	}

	@Override
	public double getViolatedNodesRatio() {
		return 0;
	}

	@Override
	public double getViolatedTerminalsCVR(double[] minCVR, double[] maxCVR) {
		double terFinalCosts[] = getCosts(arbAdjMatrix, terminals);
		double terSpannerCosts[] = getSpannerQuant(terminals);
		
		double ratios = 0;
		
		minCVR[0] = -1;
		maxCVR[0] = -1;		
		
		for (int i = 0; i < terFinalCosts.length; i++) {
			double val = terFinalCosts[i]/terSpannerCosts[i];
			if (val > maxCVR[0])
				maxCVR[0] = val;
			if ((val < minCVR[0]) || (minCVR[0] == -1))
				minCVR[0] = val;
			ratios += terFinalCosts[i]/terSpannerCosts[i];	
		}
		
		return (ratios/terFinalCosts.length);
	}

}
