package dataStructure;

import java.io.*;
import java.util.*;

import radios.Radio;
import radios.RadioParameters;

//set for tx levels used in the minimum interference path
import java.util.Set;  
import java.util.HashSet;

import radios.RadioParameters;
import tcAlgorithm.TCOverhearingProtocol;



public class AdjMatrix {
	//AM[i][j] armazena o custo de transmitir de i para j
    double[][] AM;
    int[][] IS;
    
    int size;
    double[][] MCPmatrix;
    int[][] P;
    

    int INFINITO = -1;

    public AdjMatrix (int nverts) {
	AM = new double[nverts][nverts];
	IS = new int[nverts][nverts];
	size = nverts;
	makeEmpty();
    }

    public void makeEmpty() {
	for (int i = 0; i < size; i++)
	    for (int j = 0; j < size; j++)
		if (i == j) {
		    AM[i][j] = 0;
		    IS[i][j] = 0;
		} else {
		    AM[i][j] = INFINITO;
		    IS[i][j] = INFINITO;
		}
    }

    public int getSize() { return size; }
    
    public void fillMatrixFromAdjList (AdjList adjl) {
    	ALNode alnode;

    	//eh importante tornar vazio pois alguns custos entre nos nao vai existir pois nao existe uma relacao de alcance entre estes nos
    	makeEmpty();

    	for (int i = 0; i < adjl.getNumberOfVertices(); i++) {
    	    alnode = adjl.getList(i);
    	    while (alnode != null) {
    		AM[i][alnode.getVid()] = alnode.getWeight();
    		IS[i][alnode.getVid()] = alnode.getIS();
    		alnode = alnode.getProx();
    	    }
    	}
        }    
    
    public void fillMatrixWithISFromAdjList (AdjList adjl) {
    	ALNode alnode;

    	//eh importante tornar vazio pois alguns custos entre nos nao vai existir pois nao existe uma relacao de alcance entre estes nos
    	makeEmpty();

    	for (int i = 0; i < adjl.getNumberOfVertices(); i++) {
    	    alnode = adjl.getList(i);
    	    while (alnode != null) {
    	    	//System.out.println("i: "+i+" j: "+alnode.getVid()+" is: "+alnode.getIS());
	    		AM[i][alnode.getVid()] = alnode.getIS();
	    		alnode = alnode.getProx();
    	    }
    	}
    }

    public void printAdjMatrix() {
	int i, j;

	System.out.println ("Adj Matrix:");
	for (i = 0; i < size; i++) {
	    System.out.print (i + ": ");
	    for (j = 0; j < size; j++)
		System.out.print ("  [" + AM[i][j] + "]");
	    //		System.out.print ("  [" + j + "]");
	    System.out.println();
	}
    }


    private ALNode insertInList (ALNode adjl, int j, double w, double distance, int IS) {
    	ALNode p;
    
		p = new ALNode(j, w, distance);
		p.setIS(IS);
		p.setProx(adjl);
		adjl = p;
	    	return adjl;
    }
     
    public ALNode getReducedNeibrsFromMCPMatrix (int index, Weight w, double power[]) {
    	ALNode adjl = null;
    	double minTx = -100;
    	int qtNeighbors = 0;
    	
    	for (int j = 0; j < size; j++) {
    	    if (P[index][j] == index) {
    	    	double dist = w.distance(index, j);
    	    	double tx = RadioParameters.getTxPower(RadioParameters.getRealDistance(dist));    	    	
    	    	if (tx > minTx) {
    	    		minTx = tx;
    	    		//System.out.println("o min eh: "+minTx);
    	    	}
    	    	
    	    	qtNeighbors++;
    	    	adjl = insertInList(adjl, j, MCPmatrix[index][j], dist, IS[index][j]);
    	    }
    	}
    	
    	if (minTx == -100)
    		minTx = RadioParameters.RADIO_TX_POWER_LEVELS[0];
    	power[0] = minTx;
	return adjl;
    }
    
    public void calcMeanOverhearing(int index, Weight w, AdjList adjl, int farthestNeighbor[], double hops[], double qtKOverhearing[], double qtMaxOverhearing[], double txConsumptionRatio[], double percentMultihop[]) {
    	int qtHops = 0;
    	int qtK = 0;
    	int qtMax = 0;
    	int qtHopsFar, qtKFar, qtMaxFar;
    	int qtAllNeighbors;
    	qtAllNeighbors = 0;
    	qtHopsFar = qtKFar = qtMaxFar = 0;

    	for (int i = 0; i < 4; i++) {
    		hops[i] = 0;
    		qtKOverhearing[i] = 0;
    		qtMaxOverhearing[i] = 0;
    	}
    	
    	qtAllNeighbors = 0;
	    
    	int i = index;
    		
    		int id;
    		int qtNeighbor = 0;
    		int qtMultihopNeighbor = 0;
    		double totalHops = 0;
    		double shortDistances = 0;
    		double totalDistances = 0;
    		double apartDistances = 0;
    		double totalDistanceAB = 0;
    		
    		double maxHopsRatio = percentMultihop[13];
    		//System.out.println("maxHopsRatio: "+maxHopsRatio);
    		double totalHopsRatio = 0;
    		
    		double qtTxLevels = 0;
    		
    		double totalEpsilonPart = 0;
    		double totalElecTxPart = 0;
    		double totalElecRxPart = 0;
    		
    		int minCostPathHops = 0;
    	    ALNode aln;
    	    double allMinCostPathOverhearing, allOneHopPathOverhearing, allMinCostPathHops, allRealMinCostPathOverhearing, allRealOneHopPathOverhearing;
    	    double allMinCostPathTxRatio;
    	    allMinCostPathOverhearing = allOneHopPathOverhearing = allRealMinCostPathOverhearing = allRealOneHopPathOverhearing = allMinCostPathHops = 0;
    	    allMinCostPathTxRatio = 0;
    	    int qtTxRatio = 0;
    	    
    	    for (int t = 2; t < 4; t++) {
    	    	hops[t] = 0;
    	    	qtKOverhearing[t] = 0;
    	    	qtMaxOverhearing[t] = 0;
    	    }
    	    
    	    double multihop = 0;
    	    double inequation = 0;
    	    double c;
    	    double fracK = 0;
    	    double ratio = 0;
    	    double percRecption = 0;
    	    double h = 0;
    	    //int neighbourhood = 0;
    	    

    	    //getting data between a node and each of its directed neighbors
    	    //System.out.println("Para os vizinhos de "+i+", a qt de niveis eh:");
    	    for (aln = adjl.getList(i); aln != null; aln = aln.getProx()) {
    	    	qtNeighbor++;
    	    	
    	    	minCostPathHops = 0;
    	    	id = aln.getVid();
    	    	int j = id;
    	    	/*
    	    	if (i == 23) {
    	    		System.out.println("um dos vizinhos de 23 eh: "+j);
    	    	}
    	    	*/
    	    	int totalOver = 0;
    	    	double totalTxCost = 0;
    	    	double cTotal = 0;
    	    	h = 0;
    	    	
    	    	Set<Double> distances = new HashSet<Double>(RadioParameters.RADIO_TX_POWER_LEVELS.length);
    	    	Set<Double> txLevels = new HashSet<Double>(RadioParameters.RADIO_TX_POWER_LEVELS.length);
    	    	
    	    	double distIJ = w.distance(i, j);
    	    	double realDistIJ = RadioParameters.getRealDistance(distIJ);
    	    	double realTxPowerIJ = RadioParameters.getTxPower(realDistIJ);
    	    	totalDistanceAB += realDistIJ;
    	    	c = RadioParameters.getTxPowerConsumption(realDistIJ);
    	    	//ratio += c/RADIO_RX_POWER_CONSUMPTION; 
    	    	
    	    	double shortestDist = 1000;
    	    	double totalDistance = 0;
    	    	double apart = 0;
    	    	
        		do {
        			int a = P[i][j];
        			minCostPathHops++;
        			allMinCostPathOverhearing += w.nodesInRange(a, j, adjl);
        			double distance = w.distance(a, j);
        			
        			apart += distance;
        			
        			double txRatio = RadioParameters.getTxConsumptionRatio(distance);
        			qtTxRatio++;
        			allMinCostPathTxRatio += txRatio;
        			double realDistance = RadioParameters.getRealDistance(distance);
        			int overhearing = w.nodesInRange(a, j, adjl);
        			h++;
        			double txCost = RadioParameters.getTxPowerConsumption(realDistance)*1.0;
        			totalTxCost += txCost;
        			cTotal += (txCost)/c;
        			totalOver += overhearing;
        			allRealMinCostPathOverhearing += overhearing;
        			
        			if (!distances.contains(realDistance)) {
        				distances.add(realDistance);
        				txLevels.add(RadioParameters.getTxPower(realDistance));
        				if (realDistance < shortestDist)
        					shortestDist = realDistance;
        			}
        			totalDistance += realDistance;
        			        			
        			j = P[i][j];
        		} while (j != i);

        		int over1hop = w.nodesInRange(i, id, adjl);
        		
        		if (h > 1) {
        			qtTxLevels += txLevels.size();
        			
        			shortDistances += shortestDist;
        			totalDistances += totalDistance;
        			apartDistances += apart;
        			qtMultihopNeighbor++;
        			
        			if (h > maxHopsRatio) {
        				maxHopsRatio = h;
        			}
        			
        			totalHops += h;
        			ratio += c/RadioParameters.RADIO_RX_POWER_CONSUMPTION;
        			fracK += cTotal/h;
        			multihop++;
        			j = aln.getVid();
        			//double c = AdjMatrix.getTxPowerConsumption(w.distance(i, j));
        			//int over1hop = w.overhearing(i, adjl, RadioParameters.getRealDistance(w.distance(i, j)));
        			//int over1hop = w.nodesInRange(i, j, adjl);
        			/*
        			if ((c/RadioParameters.RADIO_RX_POWER_CONSUMPTION)*(h-1) < (over1hop-totalOver))
        				inequation++;
        			*/
        			
        			Double []distancesArray = distances.toArray(new Double[distances.size()]);
        			//Double[]distancesArray = (Double [])distances.toArray();
        			Arrays.sort(distancesArray);
        			Double[]txLevelsArray = txLevels.toArray(new Double[distances.size()]);
        			Arrays.sort(txLevelsArray);
        			
        			//double epsilonPart = RadioParameters.epsilon * (Math.pow(shortestDist, RadioParameters.npathLossExp)*h - Math.pow(realDistIJ, RadioParameters.npathLossExp));
        			double epsilonPart =  RadioParameters.epsilon * (distancesArray[distancesArray.length-1] * (h - Math.pow(10, (realTxPowerIJ-txLevelsArray[txLevelsArray.length-1])/10.0)));
        			double elecRxPart = RadioParameters.elecRx * (over1hop - totalOver);
        			totalElecRxPart += elecRxPart;
        			totalEpsilonPart += epsilonPart;
        			double elecTxPart = (h - 1)*RadioParameters.elecTx;
        			totalElecTxPart += elecTxPart;
        			
        			if (epsilonPart + elecTxPart < elecRxPart)
        				inequation++;
        		}
        		percRecption += (1- totalTxCost/(totalOver*RadioParameters.RADIO_RX_POWER_CONSUMPTION));
        		
        		allMinCostPathHops += minCostPathHops;
        		qtHops++;
        		qtK++;

            	j = aln.getVid();
            	qtMax++;
            	allOneHopPathOverhearing += over1hop;
            	allRealOneHopPathOverhearing += over1hop;
    	    }

    	    if (qtNeighbor > 0) {
    	    	percentMultihop[0] += multihop/qtNeighbor;
    	    	percentMultihop[1] += inequation/qtNeighbor;
    	    	if (qtMultihopNeighbor > 0) {
    	    		percentMultihop[2] += fracK/qtMultihopNeighbor;
    	    		percentMultihop[3] += totalHops/qtMultihopNeighbor;
    	    		percentMultihop[4] += ratio/qtMultihopNeighbor;
    	    		percentMultihop[6] += shortDistances/qtMultihopNeighbor;
    	    		percentMultihop[7] += totalDistances/qtMultihopNeighbor;
    	    		percentMultihop[8] += apartDistances/qtMultihopNeighbor;
    	    		percentMultihop[9] += totalDistanceAB/qtMultihopNeighbor;
    	    		percentMultihop[10] += totalEpsilonPart/qtMultihopNeighbor;
    	    		percentMultihop[11] += totalElecTxPart/qtMultihopNeighbor;
    	    		percentMultihop[12] += totalElecRxPart/qtMultihopNeighbor;
    	    		
    	    		percentMultihop[13] = maxHopsRatio;
    	    		
    	    		percentMultihop[18] += qtTxLevels/qtMultihopNeighbor;
    	    	}
    	    	percentMultihop[5] += percRecption/qtNeighbor;
	    	    hops[2] = allMinCostPathHops/qtNeighbor;
	    		hops[3] = allMinCostPathHops/qtNeighbor;
	    		qtKOverhearing[2] = allMinCostPathOverhearing/qtNeighbor;
	    		qtKOverhearing[3] = allRealMinCostPathOverhearing/qtNeighbor;
	    		qtMaxOverhearing[2] = allOneHopPathOverhearing/qtNeighbor;
	    		qtMaxOverhearing[3] = allRealOneHopPathOverhearing/qtNeighbor;
	    		
	    		txConsumptionRatio[1] = allMinCostPathTxRatio/qtNeighbor;
    	    }
    		
    		qtAllNeighbors = qtNeighbor;
    		
    		
    		int hasNeighbor = 0;
    		qtNeighbor = minCostPathHops = 0;
    		qtKOverhearing[0] = qtKOverhearing[1] = 0;
    		txConsumptionRatio[0] = 0;
    		hops[0] = hops[1] = 0;
    		//qtTxRatio = 0;
    		//double minCostPathTxRatio = 0;
    		
    		int j = farthestNeighbor[i];
    		if (j != -1) {
    			hasNeighbor++;
	    		do {
	    			int a = P[i][j];
	    			minCostPathHops++;
	    			qtKOverhearing[0] += w.nodesInRange(a, j, adjl);
	    			double distance = w.distance(a, j);
        			double txRatio = RadioParameters.getTxConsumptionRatio(distance);
        			qtKOverhearing[1] += w.nodesInRange(a, j, adjl);
	    			txConsumptionRatio[0] += txRatio;
	    			j = P[i][j];
	    		} while (j != i);
    		}
    		
    		if (hasNeighbor != 0) {//alguns nos podem nao ter vizinhos
    			hops[0] = minCostPathHops;
    			hops[1] = minCostPathHops;
    			qtHopsFar++;
    			qtKFar++;
    			
    			j = farthestNeighbor[i];
    			qtMaxFar++;
    			qtMaxOverhearing[0] = w.nodesInRange(i, j, adjl);
    			qtMaxOverhearing[1] = w.nodesInRange(i, j, adjl);
    			
    			//txConsumptionRatio[0] = minCostPathTxRatio/qtTxRatio;
    		}
    	
    	hops[4] = qtAllNeighbors;
    	hops[5] = hasNeighbor;
    	qtKOverhearing[4] = qtAllNeighbors;
    	qtKOverhearing[5] = hasNeighbor;
    	qtMaxOverhearing[4] = qtAllNeighbors;
    	qtMaxOverhearing[5] = hasNeighbor;
    }             

    /*
     * Get the PIPS along the minimum cost path between node i and j.
     * Return 0 if node i and node j aren't connected.
     */
    public int getPips(int i, int j) {
    	int mips = 0;
    	if (P[i][j] == INFINITO)
    		return 0;
		do {
			int a = P[i][j];
			mips += MCPmatrix[a][j];
			j = P[i][j];
		} while (j != i);    	
    	return mips;
    }    
    
    /*
     * Return the amount of different tx levels in the minimum interference path between node i and node j
     */
    public int getQtTxLevels(Weight w, int i, int j) {
    	Set<Double> txLevels = new HashSet<Double>(RadioParameters.RADIO_TX_POWER_LEVELS.length);
    	if (P[i][j] == INFINITO)
    		return 0;
		do {
			int a = P[i][j];
			double txLevel = RadioParameters.getTxPower(w.distance(a, j));
			if (!txLevels.contains(txLevel)) {
				txLevels.add(txLevel);
			}
			j = P[i][j];
		} while (j != i);
		return txLevels.size();
    }
    
    int[][] getPathInformation() {
    	return P;
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
				//if ((i == 0) && (j == 3))
				//	System.out.println("achou um caminho entre 0 e 3");
			    MCPmatrix[i][j] = value1;
			    //			    P[i][j] = prevp[i][j];
			} else {
				//if ((i == 0) && (j == 3))
				//	System.out.println("achou um caminho entre 0 e 3");				
			    MCPmatrix[i][j] = value2;
			    //			    P[i][j] = k;
			}
		    }		

		    if (prevd[i][j] == MCPmatrix[i][j])
			P[i][j] = prevp[i][j];
		    else
			P[i][j] = prevp[k][j];
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

    public void printMCPmatrix() {
	int i, j;

	System.out.println ("MCP Matrix:");
	for (i = 0; i < size; i++) {
	    System.out.print (i + ": ");
	    for (j = 0; j < size; j++)
		System.out.print ("  [" + MCPmatrix[i][j] + "]");
	    System.out.println();
	}

	System.out.println ("P Matrix:");
	for (i = 0; i < size; i++) {
	    System.out.print (i + ": ");
	    for (j = 0; j < size; j++)
		System.out.print ("  [" + P[i][j] + "]");
	    System.out.println();
	}
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
	   
	
	    
       