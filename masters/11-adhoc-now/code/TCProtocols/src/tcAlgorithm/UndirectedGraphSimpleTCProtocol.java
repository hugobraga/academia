package tcAlgorithm;

import java.util.PriorityQueue;

import coverage.Coverage;
import coverage.UndirectedGraphTCAlgorithm;
import dataStructure.ALNode;
import dataStructure.AdjMatrix;
import dataStructure.PositionTable;
import dataStructure.Weight;
import support.Edge;

/**
 * The way the algorithms that inherits from this class were implemented forces to have global information about
 * the coverage (generally link-based coverage)
 *
 */
public abstract class UndirectedGraphSimpleTCProtocol extends SimpleTCProtocol {

	UndirectedGraphSimpleTCProtocol(int nsize, PositionTable npt, double nmprange,
			Weight nedgeWeight) {
		super(nsize, npt, nmprange, nedgeWeight);
		// TODO Auto-generated constructor stub
	}

	//@Override
	public boolean executeTCAlgorithm(Coverage coverage, UndirectedGraphTCAlgorithm tcAlg) {
		if (!buildLocalInformation())
			return false;

    	//set the coverage for each edge in the local neighborhood
    	coverage.setCoverage(globalMaxPowerAdjl); //for the XTC algorithm, it's not necessary		
	    
    	
    	/**
    	 * The set ISEdgeInterference is important to treat the graph as an undirected graph for the algorithm notion of interference (one edge for each pair of nodes),
    	 * but as a directed graph for the notion of INTERFERENCE BASED ON SENDER (IS), which is a node-based interference approach.
    	 * Beside this, the set is important to order the edges (undirected graph) by the interference number.
    	 * Some TC algorithms don't need edges (undirected graph) ordered.
    	 */
	    PriorityQueue<Edge> ISEdgeInterference = new PriorityQueue<Edge>();
	    
	    int flagEdge[][] = new int[size][size];
	    for (int i = 0; i < size; i++) {
	    	for (int j = 0; j < size; j++) {
	    		flagEdge[i][j] = 0;
	    	}
	    }
	    
	  //setting the IS for all edges, considering the local connectivity graph
    	for (int i = 0; i < size; i++) {
    		ALNode aln = globalMaxPowerAdjl.getList(i);
    		while (aln != null) {
	    		int menor, maior;
	    		if (aln.getVid() < i) {
	    			menor = aln.getVid();
	    			maior = i;
	    		} else {
	    			maior = aln.getVid();
	    			menor = i;	    			
	    		}    			
    			if (flagEdge[menor][maior] == 0) {
	    			int IS, IS2 = 0;
		    		IS = aln.getIS();
		    		ALNode aln2 = globalMaxPowerAdjl.getList(aln.getVid());
		    		while (aln2 != null) {
		    			if (aln2.getVid() == i) {
		    				IS2 = aln2.getIS();
		    				break;
		    			}
		    			aln2 = aln2.getProx();
		    		}
		    		Edge e = new Edge(i, aln.getVid(), IS, IS2, aln.getCoverage(), aln.getBuckhartCoverage());	    			
	    			ISEdgeInterference.add(e);
	    			flagEdge[menor][maior] = 1;
    			}
	    		aln = aln.getProx();
    		}
    	}
    	
    	/*
    	 *now we are filling the global matrix with IS considering with and without the topology control being applied 
    	 *in order to calculate the PICS spanning factor
    	 */    	    	
    	//System.out.println("matrix reduzida");
    	globalAdjMatrix = new AdjMatrix(size);
    	tcAlg.setEdgesPQ(ISEdgeInterference);
    	tcAlg.executeAndFillMatrixWithIS(globalAdjMatrix);
    	globalOptAdjl = tcAlg.getOptimizedSetOfNeighbours();
    	
    	//System.out.println("matrix max power");
    	globalMaxPowerAdjMatrix = new AdjMatrix(size);
    	globalMaxPowerAdjMatrix.fillMatrixWithISFromAdjList(globalMaxPowerAdjl);    	
    	
    	//calculating PICS spanning factor
    	globalAdjMatrix.minCostPaths();
    	globalMaxPowerAdjMatrix.minCostPaths();
    	calcPICS();
    	return true;
    }

	public abstract boolean execute();
	
	public abstract String getName();

}
