package dataStructure;

import java.io.*;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.Map;
import java.util.Stack;

import support.S2DDelay;
import support.S2DDelayComparator;


public class AdjMatrix {
    double[][] AM;
    int size;
    double[][] MCPmatrix;
    int[][] P;
    
    double[][] MCPmatrix2Check;
    int[][] P2Check;    
    
    int lastPossibleRoot;
    boolean violatedNodesFlag;
    AdjList local2CheckAdjList;
	HashSet prohibitedNodes;
	HashSet allowedNodes;
	
	Map<Integer, Integer> prohibitedIndexMap;

    public static final int INFINITO = -1;
    
    AdjList localAdjList; //copia mutavel

    public AdjMatrix (int nverts) {
    	violatedNodesFlag = false;
    	prohibitedNodes = new HashSet();
    	allowedNodes = new HashSet();
    	prohibitedIndexMap = new HashMap<Integer, Integer>();
    	
    	lastPossibleRoot = -1;

    	
	AM = new double[nverts][nverts];
	size = nverts;
	makeEmpty();
    }

    public void makeEmpty() {
	for (int i = 0; i < size; i++)
	    for (int j = 0; j < size; j++)
		if (i == j) 
		    AM[i][j] = 0;
		else
		    AM[i][j] = INFINITO;	
    }

    public int getSize() { return size; }
    
    private void fillMatrix(AdjList adjl) {
    	ALNode alnode;
    	makeEmpty();
    	
    	for (int i = 0; i < adjl.getNumberOfVertices(); i++) {
    	    alnode = adjl.getList(i);
    	    while (alnode != null) {
    		AM[i][alnode.getVid()] = alnode.getWeight();
    		alnode = alnode.getProx();
    	    }
    	}    	
    }
    
    public void refillMatrix() {
    	fillMatrix(localAdjList);
    }

    public void fillMatrixFromAdjList (AdjList adjl) {	
	localAdjList = adjl;
	
	fillMatrix(localAdjList);
    }


    public void printAdjMatrix() {
	int i, j;

	System.out.println ("Adj Matrix:");
	for (i = 0; i < size; i++) {
	    System.out.print (i + ": ");
	    for (j = 0; j < size; j++)
		System.out.print ("  [" + AM[i][j] + "]");
	    System.out.println();
	}
    }


    ALNode insertInList (ALNode adjl, int j, double w) {
    	ALNode p;
    
	p = new ALNode(j, w);
	p.setProx(adjl);
	adjl = p;
    	return adjl;
    }
    
    ALNode checkAndInsertInList (ALNode adjl, int j, double w) {
    	ALNode p;
    
	p = new ALNode(j, w);
	p.setProx(adjl);
	adjl = p;
    	return adjl;
    }
    
    public double[][] getP2PCost() {
    	return MCPmatrix;
    }
    
    public int [][] getP2PPath() {
    	return P;
    }
    
    public double getMatrixCost(int x, int y, double[][] globalMatrix) {
    	if (globalMatrix != null)
    		return MCPmatrix2Check[x][y];
    	else
    		return MCPmatrix[x][y];
    }
    
    private void deleteNodeTemporarily(int index, int terminalsIndex[]) {
		prohibitedIndexMap.put(terminalsIndex[index], index);
    	local2CheckAdjList.deleteTemporaryNode(terminalsIndex[index]);
    	terminalsIndex[index] = -1;
    }
    
    private void restoreTemporarilyDeletedNodes(int terminalsIndex[]) {
		java.util.Set mapSet = prohibitedIndexMap.entrySet();
		Iterator<Map.Entry<Integer, Integer>> it = mapSet.iterator();
		while (it.hasNext()) {
			Map.Entry entry = it.next();
			int term = (Integer)entry.getKey();
			int index = (Integer)entry.getValue();
			terminalsIndex[index] = term;
		}
		local2CheckAdjList.restoreFromTemporaryList();
		prohibitedIndexMap.clear();
    }
    
    public AdjList getSqrtKUninformedTerminalsTreeWithRestriction(AdjMatrix globalAdjMat, int terminalsIndex[], double spannerQuant[], double sqrtT, int root[], int notAllowedToBeRoot[], 
    		int source, HashSet<Integer> informedSet, Weight w, AdjList gAdjl) {
    	AdjList TuAdjl; //it is part of the solution
    	
		local2CheckAdjList = localAdjList;
    	
    	for (int i = (lastPossibleRoot+1); (i < size); i++) {//searching for the root
    		int j = 0; 
    		int cTerminals = 0;
    		S2DDelay []pairDelays = new S2DDelay[terminalsIndex.length];
    		boolean terminalRoot = false; //the root of Tu is a member of UT
    		int rootIndex = i;
    		
    		if (notAllowedToBeRoot[i] == 1) {
    			continue;
    		}
    		
    		if (localAdjList.getList(rootIndex) == null) {
    			continue;
    		}

    		if (informedSet.contains(i)) {
    			continue;
    		}
    		
    		boolean temporaryNodeFlag = false;    		
    		
    		int runCounter = 0;
    		//it is necessary to execute the loop 2 times: in the first we will disable (temporarily) the terminals for which the delay is not respected, so
    		// they can't be part of the path to other terminals. In the second run we get the terminals
    		while (runCounter < 2) {
        		j = 0;
        		terminalRoot = false; //the root of Tu is a member of UT
        		
	    		while ((j < terminalsIndex.length) ) {//here, sqrt must contain the original value
	    			if (terminalsIndex[j] != -1) {//the value -1 indicates that the terminal has been removed from UT
	    				//I'm supposing that MCPmatrix[i][i] is INFINITO, so when rootIndex corresponds to a terminal
	    				//the root will not be accounted for the Tu terminals set. So I have to account for it explicitly.
	    				if (rootIndex == terminalsIndex[j]) {
	    					terminalRoot = true;
	    				} else if ((((globalAdjMat.getP2PCost()[source][rootIndex] + MCPmatrix[rootIndex][terminalsIndex[j]]) <= spannerQuant[j]) && ((MCPmatrix[rootIndex][terminalsIndex[j]] != INFINITO) && (globalAdjMat.getP2PCost()[source][rootIndex] != INFINITO)))) {
	    					if (runCounter == 1) {
			    				pairDelays[cTerminals] = new S2DDelay(rootIndex, terminalsIndex[j], MCPmatrix[rootIndex][terminalsIndex[j]]);
			    				cTerminals++;
	    					}
		    			} else if ((MCPmatrix[rootIndex][terminalsIndex[j]] != INFINITO) && (globalAdjMat.getP2PCost()[source][rootIndex] != INFINITO)) { //MCPmatrix[rootIndex][terminalsIndex[j]] > delays[j]
		    				if (runCounter == 0) {
		    					deleteNodeTemporarily(j, terminalsIndex);
		    					temporaryNodeFlag = true;
		    				}
		    			}
	    			}
	        	    j++;
	    		}
	    		
	    		runCounter++;
	    		//we need to update the Matrix to reflect the the deletion (temporarily) of some terminals
	    		fillMatrixFromAdjList(local2CheckAdjList);
	    		minCostPaths();
    		}
    		
    		if (terminalRoot) {//when the root is terminal, we didn't account it so we do now
    			cTerminals++;
    		}
    		
    		//in this part it is important to restore nodes temporarily disabled before nodes being really removed
    		if (temporaryNodeFlag) {//if at least one terminal was temporarily removed, we need to restore the local2CheckAdjList
    			restoreTemporarilyDeletedNodes(terminalsIndex);
    		}
    		
    		
    		
    		if (cTerminals > sqrtT) {//here, sqrtT must contain the original value
    			if (terminalRoot) {
    				cTerminals--;
        			for (int t = 0; t < terminalsIndex.length; t++) {
        				if (terminalsIndex[t] == rootIndex) {
        					terminalsIndex[t] = -1;
        					break;
        				}
        			}
        			local2CheckAdjList.deleteNode(rootIndex);
    			}
    			Arrays.sort(pairDelays, 0, (cTerminals), new S2DDelayComparator());

    			TuAdjl = new AdjList(size);
        		//insert the paths in Tu
    			int closestTerQt = (int)Math.floor(sqrtT);
    			if (terminalRoot)
    				closestTerQt--;
    			for (int c = 0; c < closestTerQt; c++) {
    				int prev = pairDelays[c].getDestination();
    				for (int t = 0; t < terminalsIndex.length; t++) {
    					if (terminalsIndex[t] == prev) {
    						terminalsIndex[t] = -1;
    						break;
    					}
    				}
        			while (P[rootIndex][prev] != rootIndex) {
        				TuAdjl.checkInsertEdge(P[rootIndex][prev], prev, MCPmatrix[P[rootIndex][prev]][prev]);
        				local2CheckAdjList.deleteNode(prev);
        				prev = P[rootIndex][prev];
        			}

        			TuAdjl.checkInsertEdge(P[rootIndex][prev], prev, MCPmatrix[P[rootIndex][prev]][prev]);
        			local2CheckAdjList.deleteNode(prev);
    			}
    			local2CheckAdjList.deleteNode(rootIndex);
    			root[0] = rootIndex;
    			lastPossibleRoot = i;
    			fillMatrixFromAdjList(local2CheckAdjList);
    			minCostPaths();
    			localAdjList = local2CheckAdjList;
    			return TuAdjl;
    		}
    	}   
    	lastPossibleRoot = -1;
    	localAdjList = local2CheckAdjList;
    	return null;
    }
    

    public ALNode getReducedNeibrsFromMCPMatrix (int index) { 
    	ALNode adjl = null;
    	
    	for (int j = 0; j < size; j++) {
    	    if (P[index][j] == index) {
    		adjl = insertInList(adjl, j, MCPmatrix[index][j]);
    	    }
    	}
    	
	return adjl;
    }
    
    public void printSP(int source, int d) {
		int prev = d;
		double sumCost = 0;
		Stack<Integer> stack = new Stack<Integer>();
		if (source != d) {
			stack.push(prev);
			while (P[source][prev] != source) {
				stack.push(P[source][prev]);
				sumCost += MCPmatrix[P[source][prev]][prev];
				prev = P[source][prev];
			}
			stack.push(P[source][prev]);
			sumCost += MCPmatrix[P[source][prev]][prev];
		}
		//System.out.println("----------");
		System.out.println("custo somado de "+source+" para "+d+": "+sumCost+", pela matrix: "+MCPmatrix[source][d]);
		System.out.print("caminho de "+source+" para "+d+": ");
		while (!stack.isEmpty()) {
			System.out.print(stack.pop()+", ");
		}
		System.out.println("");
		//System.out.println("----------");
		
		
    }


    // Methods related to the implementation of Floyd-Warshall's Minimum Cost Paths algorithm

    public void minCostPaths () {
	// Implements the algorithm of Floyd-Warshall (all-pairs minimum cost paths)
	// The matrices MCPmatrix and P contains the result (MCPmatrix contains the costs and
	//        matrix P contains the paths information)

	double value1, value2;
	double[][] prevd;
	int[][] prevp;
        
	MCPmatrix = new double[size][size];
	P = new int[size][size];
	prevd = new double[size][size];
	prevp = new int[size][size];
	
	for (int i = 0; i < size; i++)
	    for (int j = 0; j < size; j++) {
	    	MCPmatrix[i][j] = 0;
		prevd[i][j] = AM[i][j];
		if ((i ==j) || (AM[i][j] == INFINITO))
		    prevp[i][j] = INFINITO;
		else
		    prevp[i][j] = i;
	    }
	 

	for (int k = 0; k < size; k++) {
	    for (int i = 0; i < size; i++)
	        for (int j=0; j < size; j++) {
		    value1 = prevd[i][j];
		    
		    if ((prevd[i][k] == INFINITO) || (prevd[k][j] == INFINITO))
			value2 = INFINITO;
		    else 
			value2 = prevd[i][k] + prevd[k][j];

		    if ((value1 == INFINITO) && (value2 == INFINITO)) {
			MCPmatrix[i][j] = INFINITO;
			//			P[i][j] = INFINITO;
		    } else if (value1 == INFINITO) {
			MCPmatrix[i][j] = value2;
			//			P[i][j] = k;
		    } else if (value2 == INFINITO) {
			MCPmatrix[i][j] = value1;
			//			P[i][j] = prevp[i][j];
		    } else {
			if (value1 < value2) {
			    MCPmatrix[i][j] = value1;
			    //			    P[i][j] = prevp[i][j];
			} else {
			    MCPmatrix[i][j] = value2;
			    //			    P[i][j] = k;
			}
		    }		

		    if (prevd[i][j] == MCPmatrix[i][j]) {
			P[i][j] = prevp[i][j];
		    } else {
			P[i][j] = prevp[k][j];			
		    }
		};

	    if (k < size - 1) {
		for (int p = 0; p < size; p++)
		    for (int q = 0; q < size; q++) {
			prevd[p][q] = MCPmatrix[p][q];
			prevp[p][q] = P[p][q];
		    }
	    }
	}
    }
    
    public void printPMatrix() {
    	System.out.println("printing matrix with path information:");
		for (int p = 0; p < size; p++) {
		    for (int q = 0; q < size; q++) {
			System.out.println("P["+p+"]["+q+"]: "+P[p][q]);
		    }
	    }    	
    }

    public void printMCPmatrix() {
	int i, j;

	System.out.println ("MCP Matrix:");
	for (i = 0; i < size; i++) {
		//if ()
	    System.out.print (i + ": ");
	    for (j = 0; j < size; j++)
		System.out.print ("  [" + MCPmatrix[i][j] + "]");
	    System.out.println();
	}
	///*
	System.out.println ("P Matrix:");
	for (i = 0; i < size; i++) {
	    System.out.print (i + ": ");
	    for (j = 0; j < size; j++)
		System.out.print ("  [" + P[i][j] + "]");
	    System.out.println();
	}
	//*/
    }


    public void plotMCPgraph (PositionTable pt, String fname) {
	// This method plots the result of running the algorithm for finding minimum cost paths
	//        with global information about the graph

	try {
	    PrintWriter out = new PrintWriter(new FileWriter(fname));

	System.out.println ("Plotting MCP graph ...");
	for (int i = 0; i < size; i++) 
	    for (int j = 0; j < size; j++) {
		if (P[i][j] != INFINITO) {
		    out.println (pt.getX(P[i][j]) + " " + pt.getY(P[i][j]));
		    out.println (pt.getX(j) + " " + pt.getY(j));
		    out.println();
		}
	    }
	out.close();
	} catch (IOException e) {
	}
    }

    public void printMinCostPath (int s, int d) {
	System.out.println ("MinCost from " + s + " to " + d);
	int v = P[s][d];
	String path = "" + d;
	while ((v != INFINITO) && (v != s)) {
	    path = v + " " + path;
	    v = P[s][v];
	}
	if (v == INFINITO) 
	    System.out.println ("nao hah caminho entre " + s + " e  " + d);
	else
	    System.out.println (v + " " + path);
    }
	    
}
	   
	
	    
       
