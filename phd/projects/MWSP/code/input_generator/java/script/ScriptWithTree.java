package script;

import java.util.Comparator;
import java.util.PriorityQueue;
import java.util.Queue;

import java.util.*;
import java.util.LinkedList;
import java.util.Queue;
import java.util.Scanner;
import java.util.Random;
import java.lang.*;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.io.File;

import javafx.util.Pair;

public class ScriptWithTree {

    private static final int QT_INST = 20;
    private static final int[] SIZES = {60, 70, 80};
    private static final double[] STR_FACTORS = {3, 4};
    // private static final int[] DEGREE = {4, 8};
    //private static final int[] DEGREE_ind = {0};
    private static final String EDGE_COST[] = {"unit", "small_random", "random"};
    // private static final String ADD_EDGE_TYPE[] = {"random", "waxman"};
    private static final String ADD_EDGE_TYPE[] = {"random"};

    // private static final String DIR = "/home/hugo/Dropbox/USP/git/doutorado/projetos/MWSP/implementacao/gerar_entrada/java/dados/";
    private static final String DIR = "/tmp/entrada3/";
    private static final String TMP_FILE = "input.dat";

    // private static int QT_EDGES[] = {100, 125, 150, 175, 200, 225, 250, 275, 300};

    private static int DEGREES[][] = {
	// {4, 8},
	{4}
    };
    
    private static int QT_EDGES_UNI[] = {100, 125, 150, 175, 200, 225, 250, 275, 300};
    
    private static int QT_EDGES[][] = {
    	{// 50, 75, 100, 
	    100, 120},
    	{// 75, 100, 
	    100, 120}
    };

    private static double DENS[][] = {
	// {0.8, 0.9, 1.0},
	// {0.6, 0.7, 0.8, 0.9, 1.0}
	// {0.5}
	// {0.1, 0.9, 1.0},
	// {0.7}
	{0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0},
	{0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0},
	{0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0}
	// {0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0}
	// {0.4, 0.5},
	// {0.4, 0.5},
	// {0.4, 0.5},
	// {0.4, 0.5},
	// {0.4, 0.5},
	// {0.4, 0.5}

    	// {0.15, 0.2},
	// {0.15, 0.2}
	
    };

    
    private static final int NO_EDGE_CONST = -1;
    private static final int SIDE_LENGTH = 100;
    private static final double MAX_VAL = Double.MAX_VALUE / 2;

    //https://www.sanfoundry.com/java-program-check-whether-undirected-graph-connected-using-bfs/
    private static boolean isConnected(int adjacency_matrix[][], int source) {
	Queue<Integer> queue;

	queue = new LinkedList<Integer>();
	
        int number_of_nodes = adjacency_matrix[source].length;
 
        int[] visited = new int[number_of_nodes];
        int i, element;
        visited[source] = 1;
        queue.add(source);
        while (!queue.isEmpty())
        {
            element = queue.remove();
            i = element;
            while (i < number_of_nodes)
            {
                if (adjacency_matrix[element][i] == 1 && visited[i] == 0)
                {
                    queue.add(i);
                    visited[i] = 1;
                }
                i++;
            }
        }  
        boolean connected = false; 
 
        for (int vertex = 0; vertex < number_of_nodes; vertex++)
        {
            if (visited[vertex] == 1)
            {
                connected = true;
            } else
            {
                connected = false;
                break;
            }
        }
 
        if (connected)
        {
            return true;
        } else
        {
            return false;
        }
    }    


    private static double[][] EdgesUnitCost(int adjMatrix[][]) {
	int size = adjMatrix[0].length;

	double[][] costMatrix = new double[size][size];
	Random rand = new Random();

	for (int i = 0; i < size; i++) {
	    for (int j = (i+1); j < size; j++) {
		double cost = ScriptWithTree.MAX_VAL;

		if (adjMatrix[i][j] == 1) {
		    cost = 1;
		}
		costMatrix[i][j] = cost;
		costMatrix[j][i] = cost;		
	    }
	}

	return costMatrix;	
    }

    private static double[][] EdgesSmallRandomCost(int adjMatrix[][]) {
	// int costs[] = {1, 2, 4, 8, 16};
	int costs[] = {1, 2, 3};	
	int size = adjMatrix[0].length;

	double[][] costMatrix = new double[size][size];
	Random rand = new Random();

	for (int i = 0; i < size; i++) {
	    for (int j = (i+1); j < size; j++) {
		double cost = ScriptWithTree.MAX_VAL;

		if (adjMatrix[i][j] == 1) {

		    float num = rand.nextFloat();
		    if (num < 0.33)
			cost = 1;
		    else if (num < 0.66)
			cost = 2;
		    else cost = 3;
		    // if (num < 0.2)
		    // 	cost = 1;
		    // else if (num < 0.4)
		    // 	cost = 2;
		    // else if (num < 0.6)
		    // 	cost = 4;
		    // else if (num < 0.8)
		    // 	cost = 8;
		    // else cost = 16;		    
		}
		costMatrix[i][j] = cost;
		costMatrix[j][i] = cost;		
	    }
	}

	return costMatrix;	
    }

    
    private static double[][] EdgesRandomCost(int adjMatrix[][]) {
	int costs[] = {1, 2, 4, 8, 16};
	// int costs[] = {1, 2, 3};	
	int size = adjMatrix[0].length;

	double[][] costMatrix = new double[size][size];
	Random rand = new Random();

	for (int i = 0; i < size; i++) {
	    for (int j = (i+1); j < size; j++) {
		double cost = ScriptWithTree.MAX_VAL;

		if (adjMatrix[i][j] == 1) {

		    float num = rand.nextFloat();
		    // if (num < 0.33)
		    // 	cost = 1;
		    // else if (num < 0.66)
		    // 	cost = 2;
		    // else cost = 3;
		    
		    if (num < 0.2)
		    	cost = 1;
		    else if (num < 0.4)
		    	cost = 2;
		    else if (num < 0.6)
		    	cost = 4;
		    else if (num < 0.8)
		    	cost = 8;
		    else cost = 16;		    
		}
		costMatrix[i][j] = cost;
		costMatrix[j][i] = cost;		
	    }
	}

	return costMatrix;	
    }

    private static int[] getArrayOfRandomInt(int size, int low, int high) {
	int[] randInts = new int[size];
	Random r = new Random();
	for (int i = 0; i < size; i++) {
	    randInts[i] = r.nextInt(high-low) + low;
	}
	return randInts;
    }

    private static double[][] EdgesEuclideanCost(int adjMatrix[][]) {
	int size = adjMatrix[0].length;

	int posX[] = getArrayOfRandomInt(size, 0, ScriptWithTree.SIDE_LENGTH);
	int posY[] = getArrayOfRandomInt(size, 0, ScriptWithTree.SIDE_LENGTH);

	double[][] costMatrix = new double[size][size];

	for (int i = 0; i < size; i++) {
	    for (int j = (i+1); j < size; j++) {
		double cost = ScriptWithTree.MAX_VAL;

		if (adjMatrix[i][j] == 1) {
		    cost = Math.sqrt(Math.pow(posX[i] - posX[j], 2) + Math.pow(posY[i] - posY[j], 2));
		}
		costMatrix[i][j] = cost;
		costMatrix[j][i] = cost;		
	    }
	}

	return costMatrix;	
    }

    
    private static int[][] addingEdgesAtRandom(int size, double per) {
	int[][] adjMatrix = new int[size][size];
	Random rand = new Random();
	for (int i = 0; i < size; i++) {
	    for (int j = (i+1); j < size; j++) {
		float num = rand.nextFloat();
		if (num <= per) {
		    adjMatrix[i][j] = 1;
		    adjMatrix[j][i] = 1;
		}
	    }
	}
	
	return adjMatrix;
    }

    /*
https://stackoverflow.com/questions/42614847/floyd-warshall-algorithm-implementation

Me parece que as posições adjMatrix[j][i], j > i, contém valores inválidos
     */
    private static void FloydWarshall(double adjMat[][]) {
	int N = adjMat[0].length;
	
	for(int k = 0; k < N; ++k) {
	    for(int i = 0; i < N; ++i) {
		for(int j = 0; j < N; ++j) {
		    adjMat[i][j] = Math.min(adjMat[i][j], adjMat[i][k] + adjMat[k][j]);
		}
	    }
	}

    }

    private static int countEdges(double costMatrix[][]) {
	int qtEdges = 0;
	int size = costMatrix[0].length;
	for (int i = 0; i < size; i++) {
	    for (int j = (i+1); j < size; j++) {

		if (costMatrix[i][j] < ScriptWithTree.MAX_VAL) {
		    qtEdges++;
		}
	    }
	}
	return qtEdges;
	
    }
    
    private static void insertTreeSolution(double costMatrix[][], int root, double sf) {	
	int size = costMatrix[0].length;

	double[][] distMatrix = new double[size][size];

	//----------------calculando distancias---------------------
	for (int i = 0; i < size; i++) {
	    for (int j = (i+1); j < size; j++) {

		if (costMatrix[i][j] == ScriptWithTree.MAX_VAL) {
		    distMatrix[i][j] = ScriptWithTree.MAX_VAL;
		    distMatrix[j][i] = ScriptWithTree.MAX_VAL;		   
		} else {
		    distMatrix[i][j] = costMatrix[i][j];
		    distMatrix[j][i] = costMatrix[i][j];
		}
	    }
	}
	FloydWarshall(distMatrix);
	//----------------------------------------------------------

	/*
	  ordenando os pares de vértices que representam arestas pelas distâncias
	 */	
	List<Pair<Pair<Integer,Integer>, Double> > pairsDist = new ArrayList<Pair<Pair<Integer,Integer>, Double> >();

	for (int i = 0; i < size; i++) {
	    for (int j = (i+1); j < size; j++) {
		if (costMatrix[i][j] < ScriptWithTree.MAX_VAL) {//representa aresta
		    pairsDist.add(new Pair<>(new Pair<>(i,j), distMatrix[i][j]));
		}
	    }
	}

	Collections.sort(pairsDist, new Comparator<Pair<Pair<Integer,Integer>, Double>>() {
		@Override
		public int compare(final Pair<Pair<Integer,Integer>, Double> o1, final Pair<Pair<Integer,Integer>, Double> o2) {
		    if (o1.getValue() < o2.getValue())
			return -1;
		    else if (o1.getValue() > o2.getValue())
			return 1;
		    else return 0;
		}
	    });
	//-------------------------------------------------------


	int[][] edgesUsedMatrix = new int[size][size];
	for (int i = 0; i < pairsDist.size(); i++) {
	    int u = pairsDist.get(i).getKey().getKey();
	    int v = pairsDist.get(i).getKey().getValue();
	    Double dist = pairsDist.get(i).getValue();

	    // System.out.println("a distancia entre " + u + " e " + v + ": " + dist);

	    if ((u == root) || (v == root)) {
		if (edgesUsedMatrix[u][v] == 0) {
		    costMatrix[u][v] = (sf * dist)/2;
		    edgesUsedMatrix[u][v] = 1;		    
		}

		// System.out.println("o custo da aresta entre " + u + " e " + v + " para sf: " + sf + " e dist: " + dist + " eh de: " + costMatrix[u][v]);
	    } else {
		int source, dest;
		if (u < root) {
		    source = u;
		    dest = root;
		} else {
		    source = root;
		    dest = u;		    
		}
		if (edgesUsedMatrix[source][dest] == 0) {
		    costMatrix[source][dest] = (sf * dist)/2;
		    edgesUsedMatrix[source][dest] = 1;
		    // System.out.println("o custo da aresta entre " + source + " e " + dest + " para sf: " + sf + " e dist: " + dist + " eh de: " + costMatrix[source][dest]);		    
		}		

		if (v < root) {
		    source = v;
		    dest = root;
		} else {
		    source = root;
		    dest = v;		    
		}
		if (edgesUsedMatrix[source][dest] == 0) {
		    costMatrix[source][dest] = (sf * dist)/2;
		    edgesUsedMatrix[source][dest] = 1;
		    // System.out.println("o custo da aresta entre " + source + " e " + dest + " para sf: " + sf + " e dist: " + dist + " eh de: " + costMatrix[source][dest]);
		}
	    }
	}
	//--------------	
	
    }

    private static double[][] genConnGraph(int size, double per, String edgeCost) {
	boolean flag = false;
	int[][] adjMatrix;

	do {
	    adjMatrix = addingEdgesAtRandom(size, per);	    
	    //System.out.println("adicionou arestas aleatoriamente");
	    flag = isConnected(adjMatrix, 0);
	    //System.out.println("verificou se o grafo é conexo");
	} while(!flag);

	double[][] costMatrix;

	String randCostName = new String("random");
	if (edgeCost.equals(randCostName))
	    costMatrix = EdgesRandomCost(adjMatrix);
	else {
	    String smallRandCostName = new String("small_random");

	    if (edgeCost.equals(smallRandCostName))
		costMatrix = EdgesSmallRandomCost(adjMatrix);
	    else {
		String eucCostName = new String("euclidean");
		if (edgeCost.equals(eucCostName))
		    costMatrix = EdgesEuclideanCost(adjMatrix);
		else
		    costMatrix = EdgesUnitCost(adjMatrix);		
	    }	    	    
	}

	return costMatrix;
    }

    private static void WritingGraphEdgeByEdge(String dir, String fileName, double[][] costMatrix, int ne) {

	BufferedWriter bw = null;
	FileWriter fw = null;

	try {
	    fw = new FileWriter(dir + fileName, true);
	    bw = new BufferedWriter(fw);

	    int size = costMatrix[0].length;

	    bw.write(String.valueOf(size + " " + ne));
	    bw.newLine();
	    
	    for (int i = 0; i < size; i++) {
		for (int j = (i+1); j < size; j++) {
		    if (costMatrix[i][j] < ScriptWithTree.MAX_VAL) {
			bw.write(String.valueOf(i + " " + j + " " + costMatrix[i][j]));
			bw.newLine();
		    }
		}
	    }
	    
	} catch (IOException x) {
	} finally {
	    if (bw != null) { try { bw.close(); } catch(Throwable t) { } }
	    if (fw != null) { try { fw.close(); } catch(Throwable t) { } }
	}
	
    }

    private static void orderNotParam(boolean insertTree) {

    	for (int k = 0; k < ScriptWithTree.QT_EDGES_UNI.length; k++) {
    	// for (int i = 0; i < ScriptWithTree.SIZES.length; i++) {
    	    for (int d = 0; d < ScriptWithTree.DENS[k].length; d++) {
    	    // for (int k = 0; k < ScriptWithTree.QT_EDGES_UNI[i].length; k++) {
    		for (int ec = 0; ec < ScriptWithTree.EDGE_COST.length; ec++) {
    		    for (int aet = 0; aet < ScriptWithTree.ADD_EDGE_TYPE.length; aet++) {

			boolean treeFlag = true;
    			for (int sf = 0; (sf < ScriptWithTree.STR_FACTORS.length) && treeFlag; sf++) {
			    if (!insertTree)
				treeFlag = false;

    			    for (int j = 1; j <= ScriptWithTree.QT_INST; j++) {
    				int order = (int)Math.round((1 + Math.sqrt(1 + (8 * ScriptWithTree.QT_EDGES_UNI[k] / ScriptWithTree.DENS[k][d])))/2);
				
    				System.out.println("size: " + order + ", qt_edge: " + ScriptWithTree.QT_EDGES_UNI[k] + ", edge_cost: " + ScriptWithTree.EDGE_COST[ec] + ", add_edge_type: " + ScriptWithTree.ADD_EDGE_TYPE[aet] + ", instance: " + j + ", perc: " + ScriptWithTree.DENS[k][d]);
    				double[][] costMatrix = genConnGraph(order, ScriptWithTree.DENS[k][d], ScriptWithTree.EDGE_COST[ec]);


    				String sfName = new String("");
    				if (insertTree) {
    				    insertTreeSolution(costMatrix, order, ScriptWithTree.STR_FACTORS[sf]);
    				    sfName = String.valueOf("-" + ScriptWithTree.STR_FACTORS[sf]);
    				}

    				int ne = countEdges(costMatrix);
    				System.out.println("ne: " + ne);


    				WritingGraphEdgeByEdge(ScriptWithTree.DIR, String.valueOf(j + sfName + "-" + ScriptWithTree.QT_EDGES_UNI[k] + "-" + (int)Math.round(100*ScriptWithTree.DENS[k][d]) + "-size" + "-" + ScriptWithTree.EDGE_COST[ec] + "-" + ScriptWithTree.ADD_EDGE_TYPE[aet] + "-" + ScriptWithTree.TMP_FILE), costMatrix, ne);
				
    			    }
			    
    			}
			
    		    }
    		}
    	    }
    	}

	
    }

    private static void orderAndDegreeParam(boolean insertTree) {
    	for (int i = 0; i < ScriptWithTree.SIZES.length; i++) {
    	    for (int k = 0; k < ScriptWithTree.DEGREES[i].length; k++) {
    		for (int ec = 0; ec < ScriptWithTree.EDGE_COST.length; ec++) {
    		    for (int d = 0; d < ScriptWithTree.ADD_EDGE_TYPE.length; d++) {

			boolean treeFlag = true;
			
    			for (int sf = 0; (sf < ScriptWithTree.STR_FACTORS.length) && treeFlag; sf++) {
			    if (!insertTree)
				treeFlag = false;

    			    for (int j = 1; j <= ScriptWithTree.QT_INST; j++) {
				double perc = (1.0*ScriptWithTree.DEGREES[i][k])/(ScriptWithTree.SIZES[i]-1);

    				System.out.println("size: " + ScriptWithTree.SIZES[i] + ", grau: " + ScriptWithTree.DEGREES[i][k] + ", edge_cost: " + ScriptWithTree.EDGE_COST[ec] + ", add_edge_type: " + ScriptWithTree.ADD_EDGE_TYPE[d] + ", sf: " + ScriptWithTree.STR_FACTORS[sf] + ", instance: " + j + ", perc: " + perc);
    				double[][] costMatrix = genConnGraph(ScriptWithTree.SIZES[i], perc, ScriptWithTree.EDGE_COST[ec]);

    				String sfName = new String("");
    				if (insertTree) {
    				    insertTreeSolution(costMatrix, ScriptWithTree.SIZES[i]-1, ScriptWithTree.STR_FACTORS[sf]);
    				    sfName = String.valueOf("-" + ScriptWithTree.STR_FACTORS[sf]);
    				}

    				int ne = countEdges(costMatrix);
    				System.out.println("ne: " + ne);
				
    				WritingGraphEdgeByEdge(ScriptWithTree.DIR, String.valueOf(j + sfName + "-" + ScriptWithTree.DEGREES[i][k] + "-" + ScriptWithTree.SIZES[i] + "-order_degree-" + ScriptWithTree.EDGE_COST[ec] + "-" + ScriptWithTree.ADD_EDGE_TYPE[d] + "-" + ScriptWithTree.TMP_FILE), costMatrix, ne);
				
    			    }
			    
    			}
			
    		    }
    		}
    	    }
    	}
	
    }

    
    private static void qtEdgesNotParam(boolean insertTree) {
    	for (int i = 0; i < ScriptWithTree.SIZES.length; i++) {
    	    for (int k = 0; k < ScriptWithTree.DENS[i].length; k++) {
    		for (int ec = 0; ec < ScriptWithTree.EDGE_COST.length; ec++) {
    		    for (int d = 0; d < ScriptWithTree.ADD_EDGE_TYPE.length; d++) {

			boolean treeFlag = true;
			
    			for (int sf = 0; (sf < ScriptWithTree.STR_FACTORS.length) && treeFlag; sf++) {
			    if (!insertTree)
				treeFlag = false;

    			    for (int j = 1; j <= ScriptWithTree.QT_INST; j++) {

    				System.out.println("size: " + ScriptWithTree.SIZES[i] + ", qt_edge: " + ScriptWithTree.DENS[i][k] + ", edge_cost: " + ScriptWithTree.EDGE_COST[ec] + ", add_edge_type: " + ScriptWithTree.ADD_EDGE_TYPE[d] + ", sf: " + ScriptWithTree.STR_FACTORS[sf] + ", instance: " + j + ", perc: " + ScriptWithTree.DENS[i][k]);
    				double[][] costMatrix = genConnGraph(ScriptWithTree.SIZES[i], ScriptWithTree.DENS[i][k], ScriptWithTree.EDGE_COST[ec]);

    				String sfName = new String("");
    				if (insertTree) {
    				    insertTreeSolution(costMatrix, ScriptWithTree.SIZES[i]-1, ScriptWithTree.STR_FACTORS[sf]);
    				    sfName = String.valueOf("-" + ScriptWithTree.STR_FACTORS[sf]);
    				}

    				int ne = countEdges(costMatrix);
    				System.out.println("ne: " + ne);
				
    				WritingGraphEdgeByEdge(ScriptWithTree.DIR, String.valueOf(j + sfName + "-" + (int)Math.round(100*ScriptWithTree.DENS[i][k]) + "-" + ScriptWithTree.SIZES[i] + "-edges-" + ScriptWithTree.EDGE_COST[ec] + "-" + ScriptWithTree.ADD_EDGE_TYPE[d] + "-" + ScriptWithTree.TMP_FILE), costMatrix, ne);
				
    			    }
			    
    			}
			
    		    }
    		}
    	    }
    	}
	
    }
    
    private static void densNotParam(boolean insertTree) {

    	for (int i = 0; i < ScriptWithTree.SIZES.length; i++) {
    	    for (int k = 0; k < ScriptWithTree.QT_EDGES[i].length; k++) {
    		for (int ec = 0; ec < ScriptWithTree.EDGE_COST.length; ec++) {
    		    for (int d = 0; d < ScriptWithTree.ADD_EDGE_TYPE.length; d++) {

			boolean treeFlag = true;
			
    			for (int sf = 0; (sf < ScriptWithTree.STR_FACTORS.length) && treeFlag; sf++) {
			    if (!insertTree)
				treeFlag = false;
			    

    			    for (int j = 1; j <= ScriptWithTree.QT_INST; j++) {
    				double perc = (2.0 * ScriptWithTree.QT_EDGES[i][k])/(ScriptWithTree.SIZES[i] * (ScriptWithTree.SIZES[i]-1));

    				System.out.println("size: " + ScriptWithTree.SIZES[i] + ", qt_edge: " + ScriptWithTree.QT_EDGES[i][k] + ", edge_cost: " + ScriptWithTree.EDGE_COST[ec] + ", add_edge_type: " + ScriptWithTree.ADD_EDGE_TYPE[d] + ", sf: " + ScriptWithTree.STR_FACTORS[sf] + ", instance: " + j + ", perc: " + perc);
    				double[][] costMatrix = genConnGraph(ScriptWithTree.SIZES[i], perc, ScriptWithTree.EDGE_COST[ec]);				
				
    				String sfName = new String("");
    				if (insertTree) {
    				    insertTreeSolution(costMatrix, ScriptWithTree.SIZES[i]-1, ScriptWithTree.STR_FACTORS[sf]);
    				    sfName = String.valueOf("-" + ScriptWithTree.STR_FACTORS[sf]);
    				}

    				int ne = countEdges(costMatrix);
    				System.out.println("ne: " + ne);
				
    				WritingGraphEdgeByEdge(ScriptWithTree.DIR, String.valueOf(j + sfName + "-" + ScriptWithTree.QT_EDGES[i][k] + "-" + ScriptWithTree.SIZES[i] + "-dens-" + ScriptWithTree.EDGE_COST[ec] + "-" + ScriptWithTree.ADD_EDGE_TYPE[d] + "-" + ScriptWithTree.TMP_FILE), costMatrix, ne);
				
    			    }
			    
    			}
			
    		    }
    		}
    	    }
    	}
	
    }
    
    public static void main(String[] args) {
	// ScriptWithTree.orderNotParam(false);
	// ScriptWithTree.densNotParam(false);
	ScriptWithTree.qtEdgesNotParam(false);
	// ScriptWithTree.orderAndDegreeParam(false);
    }
}
