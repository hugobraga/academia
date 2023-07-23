package algorithms;

import java.util.ArrayList;
import java.util.Iterator;


import main.Graph;
import msc.Greedy;
import msc.Group;
import msc.Item;
import msc.ElkinItem;
import msc.Node;
import msc.PseudoNode;
import msc.Set;
import dataStructure.ALNode;
import dataStructure.AdjList;
import dataStructure.AdjMatrix;
import dataStructure.PositionTable;
import dataStructure.Weight;

import java.util.HashSet;

public class ApproximationAlgorithm extends MSCBasedAlgorithm implements Algorithm {

	int sourceIndex;
    AdjList localAdjl;    
	ArrayList<Integer> roots;
	
	int[] violatedTer;
	boolean finalMatrix;
	double qtViolated;
	double violatedCVR;
	
	protected void clearVar() {
		super.clearVar();
		roots = new ArrayList<Integer>();
		finalMatrix = false;
		qtViolated = 0;		
	}
    
    public ApproximationAlgorithm(int nsize, double nmprange, int sourceId,
			int[] terminalsId, double[] delaysTerm, PositionTable npt, Weight w) {
		super(nsize, nmprange, sourceId, terminalsId, delaysTerm, npt, w);
		// TODO Auto-generated constructor stub
		
		roots = new ArrayList<Integer>();
		
		finalMatrix = false;
		qtViolated = 0;
	}
    
    public boolean execute(double perc) {
    	clearVar();
    	preExecute(perc);
    	
    	
    	AdjList Tu;
    	int terminalsLeft[] = new int[terminals.length];
    	double delaysLeft[] = new double[spTerCosts.length];  
    	
    	informedSet.add(source);
    	
		uninformedAdjl.deleteNode(source);
    	uninformedAdjMatrix.fillMatrixFromAdjList(uninformedAdjl);
    	uninformedAdjMatrix.minCostPaths();
    	
    	terminalsLeft = terminals.clone();
    	delaysLeft = spTerCosts.clone();
    	qtTerminalsLeft = terminals.length;
    	
    	int [][]P = uninformedAdjMatrix.getP2PPath();
    	double [][]MCPmatrix = uninformedAdjMatrix.getP2PCost();
    	
    	globalAdjMatrix.fillMatrixFromAdjList(globalAdjl);
    	
    	int []notAllowedToBeRoot = new int[size];
    	
    	//set if the the node can not be a root due to the impossibility of the existence of the node in the optimum tree
    	for (int r = 0; r < size; r++) {
    		if (MCPmatrix[source][r] > delayMax)
    			notAllowedToBeRoot[r] = 1;
    	}
    	
    	//procedure Comp-Par
    	int []root = new int[1];
    	double sqrtT = Math.sqrt(terminals.length);
    	while ((Tu = uninformedAdjMatrix.getSqrtKUninformedTerminalsTreeWithRestriction(globalAdjMatrix, terminalsLeft, delaysLeft, sqrtT, 
    			root, notAllowedToBeRoot, source, informedSet, edgeWeight, globalAdjl)) != null) {
    		updateInformedSet(Tu, root[0]);
    		roots.add(root[0]);
        	for (int j = 0; j < Tu.getNumberOfVertices(); j++)
        		quasiArborescence.copyList(Tu, j);
        	qtTerminalsLeft = qtTerminalsLeft - (int)Math.floor(sqrtT);
        	if (qtTerminalsLeft <= sqrtT)
        		break;
    	}

    	uninformedAdjMatrix.refillMatrix();
    	uninformedAdjMatrix.minCostPaths();
    	
    	qtTerminalsLeft = 0;
    	for (int z = 0; z < terminals.length; z++){
    		if (!informedSet.contains(terminals[z])) {
    				terminalsLeft[z] = terminals[z];
    				qtTerminalsLeft++;
    		} else
    			terminalsLeft[z] = -1;
    	}    	
    	
    	ArrayList<ElkinItem> v2Set = new ArrayList<ElkinItem>(qtTerminalsLeft);
    	
    	uninformedTerminals = new ArrayList<Integer>(qtTerminalsLeft);
    	uninformedTerminalsDelay = new ArrayList<Double>(qtTerminalsLeft);
    	
    	int j = 0;
    	for (int i = 0; i < terminalsLeft.length; i++) {
    		if (terminalsLeft[i] != -1) {//the value different from -1 indicates that the terminal is in UT
    			uninformedTerminals.add(terminalsLeft[i]);
    			uninformedTerminalsDelay.add(delaysLeft[i]);
    			v2Set.add(new ElkinItem(null, new Node(terminalsLeft[i])));
    		}
    	}
    	
    	//finding out dominating set D
    	applyDistDomProcedure(informedSet, v2Set);
    	addingTreesForDomSet(uninformedTerminals);
    	
    	//connecting source to Roots
    	P = globalAdjMatrix.getP2PPath();
    	MCPmatrix = globalAdjMatrix.getP2PCost();
    	for (int c = 0; c < roots.size(); c++) {
    		int prev = roots.get(c);
    		while (P[source][prev] != source) {
    			quasiArborescence.checkInsertEdge(P[source][prev], prev, MCPmatrix[P[source][prev]][prev]);
    			prev = P[source][prev];
    		}
    		quasiArborescence.checkInsertEdge(P[source][prev], prev, MCPmatrix[P[source][prev]][prev]);
    	}
    	
    	//this procedure is necessary to guarantee that the final graph is a tree
    	arborescence = buildSPT2Terminals(uninformedAdjMatrix, quasiArborescence, terminals);
    	
    	uninformedAdjMatrix.fillMatrixFromAdjList(arborescence);
		uninformedAdjMatrix.minCostPaths();	
    	return true;
    }

	@Override
	public String getName() {
		// TODO Auto-generated method stub
		return new String("ElkinVariation");
	}
	
	private void setFinalMatrix(AdjList arb) {
		if (!finalMatrix) {
			uninformedAdjMatrix.fillMatrixFromAdjList(arb);
			uninformedAdjMatrix.minCostPaths();	
			finalMatrix = true;
		}
	}	
	
	public double getTotalTerminalsCVR(double minCVR[], double maxCVR[]) {
		setFinalMatrix(arborescence);
		
		double terFinalCosts[] = getCosts(uninformedAdjMatrix, terminals);
		double terSpannerCosts[] = getSpannerQuant();
		//double min, max;
		
		minCVR[0] = 0;
		maxCVR[0] = 0;
		double ratios = 0;
		for (int i = 0; i < terFinalCosts.length; i++) {
			double val = terFinalCosts[i]/terSpannerCosts[i];
			if (val > maxCVR[0])
				maxCVR[0] = val;
			if ((val < minCVR[0]) || (minCVR[0] == 0))
				minCVR[0] = val;
			ratios += terFinalCosts[i]/terSpannerCosts[i];
		}
		
		if (terFinalCosts.length == 0)
			return 0;
		else
			return (ratios/terFinalCosts.length);
	}
	
	public double getSecondPhaseTerminalsCVR(double minCVR[], double maxCVR[]) {
		setFinalMatrix(arborescence);
		
		violatedTer = new int[uninformedTerminals.size()];
		for (int i = 0; i < uninformedTerminals.size(); i++) {
			violatedTer[i] = uninformedTerminals.get(i);
		}
		
		double terFinalCosts[] = getCosts(uninformedAdjMatrix, violatedTer);
		double terSpannerCosts[] = getSpannerQuant(violatedTer);
		
		double ratios = 0;
		//double violatedRatio = 0;
		
		minCVR[0] = 0;
		maxCVR[0] = 0;		
		
		for (int i = 0; i < terFinalCosts.length; i++) {
			double val = terFinalCosts[i]/terSpannerCosts[i];
			if (val > maxCVR[0])
				maxCVR[0] = val;
			if ((val < minCVR[0]) || (minCVR[0] == 0))
				minCVR[0] = val;
			ratios += terFinalCosts[i]/terSpannerCosts[i];
			/*
			if (terFinalCosts[i] > terSpannerCosts[i]) {
				violatedRatio += val;
				qtViolated++;
			}
			*/		
		}
		
		if (terFinalCosts.length == 0)
			return 0;
		else
			return (ratios/terFinalCosts.length);
	}
	
	public double getViolatedTerminalsCVR(double minCVR[], double maxCVR[]) {
		///*
		setFinalMatrix(arborescence);
		///*
		violatedTer = new int[uninformedTerminals.size()];
		for (int i = 0; i < uninformedTerminals.size(); i++) {
			violatedTer[i] = uninformedTerminals.get(i);
		}
		//*/
		
		double terFinalCosts[] = getCosts(uninformedAdjMatrix, violatedTer);
		double terSpannerCosts[] = getSpannerQuant(violatedTer);
		
		qtViolated = 0;
		double ratios = 0;
		
		minCVR[0] = 0;
		maxCVR[0] = 0;			
		
		for (int i = 0; i < terFinalCosts.length; i++) {
			if (terFinalCosts[i] > terSpannerCosts[i]) {
				double val = terFinalCosts[i]/terSpannerCosts[i];
				if (val > maxCVR[0])
					maxCVR[0] = val;
				if ((val < minCVR[0]) || (minCVR[0] == 0))
					minCVR[0] = val;
				ratios += terFinalCosts[i]/terSpannerCosts[i];
				qtViolated++;
			}
		}
		if (qtViolated == 0)
			return 0;
		else {
			return (ratios/qtViolated);
		}
	}
	
	public double getViolatedNodesRatio() {
		return qtViolated/terminals.length;
	}

	@Override
	public double[] getArbCosts() {
		uninformedAdjMatrix.fillMatrixFromAdjList(arborescence);
		uninformedAdjMatrix.minCostPaths();		
		return getCosts(uninformedAdjMatrix, terminals);
	}
	
	@Override
	public double[] getTerViolatedCosts() {
		
		violatedTer = new int[uninformedTerminals.size()];
		for (int i = 0; i < uninformedTerminals.size(); i++) {
			violatedTer[i] = uninformedTerminals.get(i);
		}
		return getCosts(uninformedAdjMatrix, violatedTer);
	}
	
	public double[] getViolatedTerSpannerQuant() {
		return getSpannerQuant(violatedTer);
	}

	@Override
	public int getArbMaxOutDegree() {
		// TODO Auto-generated method stub
		return getMaxOutDegree(arborescence);
	}
}
