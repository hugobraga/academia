package tcAlgorithm;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.HashSet;
import java.util.Iterator;
import java.util.PriorityQueue;
import java.util.Set;

import dataStructure.ALNode;
import dataStructure.AdjList;
import dataStructure.AdjMatrix;
import dataStructure.PositionTable;
import dataStructure.Weight;

import support.Neighbor;

public abstract class TCProtocol {

	public static final int INFINITO = 100;
	
	protected int size;
	protected PositionTable pt;
	protected double mprange;  // max power range
	protected Weight edgeWeight;

	protected AdjList globalMaxPowerAdjl; 		//global adjacent list with max power information
	protected AdjList globalOptAdjl;	//global adjacent list after TC algorithm been applied
	
	protected AdjList globalOptAssymAdjl;
	
	protected AdjMatrix globalAdjMatrix;
	protected AdjMatrix globalMaxPowerAdjMatrix;
      
	protected int farthestNeighbor[];
    
	protected double PICSRatio = 0;
	protected double maxPICSRatio = 0;
    
	protected double maxQtTxLevels = 0;
	protected double meanQtTxLevels = 0;
    
    TCProtocol (int nsize, PositionTable npt, double nmprange, Weight nedgeWeight) {
    	
	size = nsize;
	pt = npt;
	mprange = nmprange;
	edgeWeight = nedgeWeight;
	
	farthestNeighbor = new int[size];
    }
    
    public double distance (int v1, int v2) {
    	boolean flag = false;
    	double x1, y1, x2, y2;
    	
    	if (v2 == -1) {
    		x2 = 0;
    		y2 = 0;
    	} else {
    		x2 = pt.getX(v2);
    		y2 = pt.getY(v2);
    	}
    	x1 = pt.getX(v1);
    	y1 = pt.getY(v1);
    	
    	if ((x2 < x1) || ((x1 == x2) && (y2 < y1)))
    		flag = true;
    	if (flag)
    		return (Math.sqrt(Math.pow(x1 - x2, 2) + Math.pow(y1 - y2, 2)));
    	else
    		return (Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2)));
    }
    
    public AdjList getMaxSetOfNeighbors() {
		return globalMaxPowerAdjl;
	}    
    
    /*
     * Calculate the PICS spanning factor
     */
    protected void calcPICS() {
    	double totalFrac = 0;
    	int noPath = 0;
    	double max = 0;
    	double maxTxLevels = 0;
    	double totalTxLevels = 0;
    	for (int i = 0; i < size; i++) {
    		for (int j = 0; j < size; j++) {
    			if (i != j) {
    				int subgraphPips = globalAdjMatrix.getPips(i, j);
    				if (subgraphPips == 0) {
    					//System.out.println("para i "+i+" j "+j+" o pips eh 0");
    					noPath++;
    					continue;
    				}
    				
    				int maxPips = globalMaxPowerAdjMatrix.getPips(i, j);
    				//System.out.println("subgraphPips: "+subgraphPips+", maxPips: "+maxPips);
    				double ratio = (subgraphPips*1.0)/maxPips;
    				if (ratio > max)
    					max = ratio;
    				totalFrac += ratio;
    				
    				if (maxQtTxLevels > 0)
    					continue;
    				int qt = globalAdjMatrix.getQtTxLevels(edgeWeight, i, j);
    				totalTxLevels += qt;
    				if (qt > maxTxLevels) {
    					maxTxLevels = qt;
    				}
    			}
    		}
    	}
    	maxPICSRatio = max;
    	PICSRatio = totalFrac/(1.0*size*(size)-(size+noPath));
    	//we decrease (size+noPath) of the amount because there's no PICS between the node and itself (this happens size times) and noPath
    	//represents the number of pairs without connection
    	
    	if (maxQtTxLevels == 0) {
	    	maxQtTxLevels = maxTxLevels;
	    	meanQtTxLevels = totalTxLevels/(1.0*size*(size)-(size+noPath));
    	}
    } 
    
    public double getPICS() {
    	return PICSRatio;
    }
    
    public double getMaxPICS() {
    	return maxPICSRatio;
    }
    
    public double getDiffTxLevels() {
    	return meanQtTxLevels;
    }
    
    public double getMaxDiffTxLevels() {
    	return maxQtTxLevels;
    } 
    
    public void plotMaxPowerGraph (String fname) {
    	ALNode node;

    	try {
    	    PrintWriter out = new PrintWriter(new FileWriter(fname));

    	    System.out.println ("Plotting Max Power Graph ...");
    	    for (int i = 0; i < size; i++) {
    		node = globalMaxPowerAdjl.getList(i);
    		while (node != null) {
    		    out.println (pt.getX(i) + " " + pt.getY(i));
    		    out.println (pt.getX(node.getVid()) + " " + pt.getY(node.getVid()));
    		    out.println();
    		    node = node.getProx();
    		}
    	    }
    	    out.close();
    	} catch (IOException e) {
    	}
        }

        public void plotReducedPowerGraph (String fname) {
    	ALNode node;

    	try {
    	    PrintWriter out = new PrintWriter(new FileWriter(fname));

    	    System.out.println ("Plotting Reduced Power Graph ...");
    	    for (int i = 0; i < size; i++) {
    		node = globalOptAdjl.getList(i);
    		while (node != null) {
    		    out.println (pt.getX(i) + " " + pt.getY(i));
    		    out.println (pt.getX(node.getVid()) + " " + pt.getY(node.getVid()));
    		    out.println();
    		    node = node.getProx();
    		}
    	    }
    	    out.close();
    	} catch (IOException e) {
    	}
        }
        
        public void plotReducedPowerAssymGraph (String fname) {
        	ALNode node;

        	try {
        	    PrintWriter out = new PrintWriter(new FileWriter(fname));

        	    System.out.println ("Plotting Reduced Power Assymmetric Graph ...");
        	    for (int i = 0; i < size; i++) {
        		node = globalOptAssymAdjl.getList(i);
        		while (node != null) {
        		    out.println (pt.getX(i) + " " + pt.getY(i));
        		    out.println (pt.getX(node.getVid()) + " " + pt.getY(node.getVid()));
        		    out.println();
        		    node = node.getProx();
        		}
        	    }
        	    out.close();
        	} catch (IOException e) {
        	}
            }

        public void plotMCP (String fname) {
    	// This method plots the result of finding minimum cost paths from global knowledge
    	//        of the connectivity graph

    	globalMaxPowerAdjl.setWeight(edgeWeight);
    	AdjMatrix adjm = new AdjMatrix(size);
    	adjm.fillMatrixFromAdjList(globalMaxPowerAdjl);
    	adjm.minCostPaths();
    	adjm.plotMCPgraph(pt, fname);
        } 
        
    	protected boolean buildLocalInformation() {
    		// build global adjacent list
        	globalMaxPowerAdjl = new AdjList(size);
        	double maxDist[] = new double[size];
        	//globalOptAdjl = new AdjList(size);
        	
        	
        	//one of the TC algorithms needs the neighborhood list ordered
        	for (int i=0; i < size; i++) {
        		PriorityQueue<Neighbor> orderedNeigh = new PriorityQueue<Neighbor>();
        		maxDist[i] = -1;
        		farthestNeighbor[i] = -1;
        	    for (int j=0; j < size; j++) {
        	    	double distance = distance(i, j);
    	    		if ((i != j) && (distance <= mprange)) {    	    			
    	    			orderedNeigh.add(new Neighbor(j, distance));
    	    			if (distance > maxDist[i]) {
    	    				maxDist[i] = distance;
    	    				farthestNeighbor[i] = j;
    	    			}
    	    		}
        	    }
        	    if (maxDist[i] == -1)
        	    	return false;
        	    while (!orderedNeigh.isEmpty()) {
        	    	Neighbor neigh = (Neighbor)orderedNeigh.poll();
        	    	globalMaxPowerAdjl.insertEdge (i, neigh.getId(), 0, neigh.getDistance(), -1);   // weight is initially set zero
        	    }
        	}
        	
      	  //set the interference based on sender (IS - considers the interference in the assymetric edge)
    	    globalMaxPowerAdjl.setIS(edgeWeight);
    	    //System.exit(0);
        	
    	    return true;
    	}
    	
        public AdjList getOptimizedSetOfNeighbours() {
        	return globalOptAdjl;
        }    	
        
        public abstract boolean execute();
        
        public abstract String getName();
}
