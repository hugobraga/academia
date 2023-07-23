package algorithms;

import java.util.ArrayList;
import java.util.HashSet;

import main.Graph;
import msc.ElkinItem;
import msc.Greedy;
import dataStructure.AdjList;
import dataStructure.AdjMatrix;
import dataStructure.PositionTable;
import dataStructure.Weight;

public class DSMDStAlgorithm {
    int size;
    PositionTable pt;
    double mprange;  // max power range
    Weight edgeWeight;
    
    int qtTerminalsLeft;

    AdjList globalAdjl;
    AdjList uninformedAdjl;
    
    //Steiner parameters
    int source;
    int terminals[];
    double spTerCosts[];
    double [][]spannerQuant;
    double spCosts[];
    double delayMax;

    AdjMatrix globalAdjMatrix;    
    
	AdjList arborescence;
	AdjList arborescenceSPT;
	
	public DSMDStAlgorithm(int nsize, double nmprange, int sourceId, int terminalsId[], double delaysTerm[], PositionTable npt, Weight w) {
    	pt = npt;
    	edgeWeight = w;
    	size = nsize;
    	mprange = nmprange;
    	
    	qtTerminalsLeft = 0;
    	
    	source = sourceId;
    	terminals = terminalsId.clone();
    	spTerCosts = new double[terminalsId.length];
    	spannerQuant = new double[spTerCosts.length][Graph.SPANNER_FACTORS.length];
    	spCosts = new double[spTerCosts.length];
    	
    	arborescence = new AdjList(size);
    	
    	globalAdjMatrix = new AdjMatrix(size);
	}
	
	public double[] getSpannerQuant() {
		return spTerCosts;
	}
	
	public double[] getSpannerQuant(int ter[]) {
		double [] spanQuant = new double[ter.length];
		for (int i = 0; i < ter.length; i++) {
			for (int j = 0; j < terminals.length; j++) {
				if (ter[i] == terminals[j]) {
					spanQuant[i] = spTerCosts[j];
					break;
				}
			}
		}
		return spanQuant;
	}	
	
	protected void clearVar() {
		spTerCosts = new double[terminals.length];
		
    	arborescence = new AdjList(size);		
	}
	
	public boolean pseudoConstructor() {
    	if (!beforeExecute())
    		return false;
    	return true;
	}
	
    public void preExecute(double perc) {
    	uninformedAdjl = new AdjList(size);
    	for (int i = 0; i < size; i++) {
    		uninformedAdjl.copyList(globalAdjl, i);
    	}

    	int i = -1;
    	for (int j = 0; j < Graph.SPANNER_FACTORS.length; j++) {
    		if (Graph.SPANNER_FACTORS[j] == perc) {
    			i = j;
    			break;
    		}
    	}
    	
    	double max = -1;
		for (int j = 0; j < terminals.length; j++) {
			spTerCosts[j] = spannerQuant[j][i];
    		if (spTerCosts[j] > max)
    			max = spTerCosts[j];
		}
		delayMax = max;
    }	
	
    
    public double getTermCoverFstPhRatio() {
    	return (terminals.length*1.0 - qtTerminalsLeft)/terminals.length;
    }	
	
	protected int getMaxOutDegree(AdjList arborescence) {
		int max = 0;
		
    	for (int j = 0; j < size; j++) {
    		if (arborescence.getList(j) != null) {
    			int degree = arborescence.getOutDegree(j);
    			if (degree > max)
    				max = degree;
    		}
    	}
		return max;		
	}
	
	protected double[] getCosts(AdjMatrix arbMatrix, int []terVector) {
		double[] costs = new double[terVector.length];
		double[][] P2PCosts = arbMatrix.getP2PCost();
		for (int i = 0; i < terVector.length; i++) {
			costs[i] = Weight.roundTwoDecimals(P2PCosts[source][terVector[i]]);
		}
		return costs;
	}	
	
	public double[] getSPCosts() {
		return spCosts;
	}
	
    double distance (int v1, int v2) {
    	return Weight.roundTwoDecimals(Math.sqrt(Math.pow(pt.getX(v1) - pt.getX(v2), 2) + Math.pow(pt.getY(v1) - pt.getY(v2), 2)));
    }
    
    public boolean beforeExecute() {
		if (!buildLocalInformation())
			return false;
    	
		return setSpannerQuant();
    }
    
    protected boolean setSpannerQuant() {
    	globalAdjMatrix.fillMatrixFromAdjList(globalAdjl);
    	globalAdjMatrix.minCostPaths();    	
    	
    	double [][]MCPmatrix = globalAdjMatrix.getP2PCost();
    	for (int j = 0; j < Graph.SPANNER_FACTORS.length; j++) {
    		double spanFactor = Graph.SPANNER_FACTORS[j];
        	for (int i = 0; i < terminals.length; i++) {
        		if (MCPmatrix[source][terminals[i]] == -1)
        			return false;
        		
        		double newSpannerQuant = MCPmatrix[source][terminals[i]]*spanFactor;
        		spannerQuant[i][j] = Weight.roundTwoDecimals(newSpannerQuant);
        	}
    	}
    	return true;
    }    
    
    protected boolean buildLocalInformation() {
    	double maxDist[] = new double[size];
    	
    	// build adjacent list
    	globalAdjl = new AdjList(size);
    	uninformedAdjl = new AdjList(size);
    	for (int i=0; i < size; i++) {
    		maxDist[i] = -1;
    		boolean flag = false;
    	    for (int j=0; j < size; j++) {
	    		if ((i != j) && (distance(i, j) <= mprange)) {
	    			flag = true;
	    		    globalAdjl.insertEdge (i, j, 0);   // weight is initially set zero
	    		    uninformedAdjl.insertEdge (i, j, 0);   // weight is initially set zero;
	    		}
    	    }
    	    
    	    if (!flag)
    	    	return false;    	    
    	}
    	globalAdjl.setWeight(edgeWeight);
    	uninformedAdjl.setWeight(edgeWeight);
    	return true;
    }
    
    protected boolean checkDelayConstrainedPaths(AdjMatrix adjMatrix, double perc) {
    	double [][]MCPmatrix = adjMatrix.getP2PCost();
    	for (int i = 0; i < terminals.length; i++) {

    		
    		if ((MCPmatrix[source][terminals[i]] == -1) || (MCPmatrix[source][terminals[i]] > spTerCosts[i]))
    			return false;
    		spCosts[i] = MCPmatrix[source][terminals[i]];
    	}
    	return true;
    }
    
    protected AdjList buildSPT2Terminals(AdjMatrix adjMatrix, AdjList adjList, int[] terminalsPar) {
    	AdjList spt = new AdjList(size);
    	
    	adjMatrix.fillMatrixFromAdjList(adjList);
    	adjMatrix.minCostPaths();
    	
    	int [][]P = adjMatrix.getP2PPath();
    	double [][]MCPmatrix = adjMatrix.getP2PCost();      	
    	
    	for (int c = 0; c < terminalsPar.length; c++) {
    		int prev = terminalsPar[c];
    		while (P[source][prev] != source) {
    			spt.checkInsertEdge(P[source][prev], prev, MCPmatrix[P[source][prev]][prev]);
    			prev = P[source][prev];
    		}
    		spt.checkInsertEdge(P[source][prev], prev, MCPmatrix[P[source][prev]][prev]);
    	}   
    	return spt;
    }
}
