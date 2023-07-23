package dataStructure;

import java.io.*;
import java.util.*;
import java.lang.Math;

public class AdjList {
    
    int nv;
    ALNode[] adjl;
    
    ALNode[] tempAdjl;

    double[] PrimKeys;
    int[] PrimParent;

    int INFINITO = -1;

    public AdjList(int nverts) {
	int i;
	
	tempAdjl = null;

	nv = nverts;
	 adjl = new ALNode[nverts];

	 for (i=0; i<nverts; i++) {
	     adjl[i] = null;
	 }
    }

    public int getNumberOfVertices() { return nv; }
    
    public void setNumberOfVertices(int size) {
    	nv = size;
    }
    
    public int getOutDegree(int vId) {
    	int degree = 0;
    	ALNode aln = adjl[vId];
    	while (aln != null) {
    		degree++;
    		aln = aln.getProx();
    	}
    	return degree;
    }

    public ALNode getList (int lindex) { return adjl[lindex]; }

    public void setList (ALNode aln, int index) {
	adjl[index] = aln;
    }
    
    public void deleteTemporaryNode(int v) {
    	if (tempAdjl == null) {
    		tempAdjl = new ALNode[nv];

    		 for (int i=0; i<nv; i++) {
    			 tempAdjl[i] = null;
    		 }    		
    	}
		copyTemporaryList(v);
		adjl[v] = null;
		
		ALNode nodes, temp;
		ALNode p;
    	for (int i = 0; i < nv; i++) {
    		nodes = adjl[i];
    		if (nodes != null) {//i == v
    			temp = nodes;
        		while(nodes != null) {
        			if (nodes.getVid() == v) {
        				//inserting in temporary list
        				p = new ALNode(nodes.getVid(), nodes.getWeight());
        			    p.setProx(tempAdjl[i]);
        			    tempAdjl[i] = p;            				
        				
        				temp.setProx(nodes.getProx());
        				nodes = null;
        				break;
        			}
        			temp = nodes;
        			nodes = nodes.getProx();
        		}    			
    		}
    	}    	
    }
    
    public void restoreFromTemporaryList() {
    	ALNode p;

    	for (int index = 0; index < nv; index++) {
	    	for (ALNode q = tempAdjl[index]; q != null; q = q.getProx()) {
	    	    p = new ALNode(q.getVid(), q.getWeight());
	    	    p.setProx(adjl[index]);
	    	    adjl[index] = p;
	    	}    	
    	}
    	
    	tempAdjl = null;
    }
    
    private  void copyTemporaryList (int index) {
    	ALNode p;

    	for (ALNode q = getList(index); q != null; q = q.getProx()) {
    	    p = new ALNode(q.getVid(), q.getWeight());
    	    p.setProx(tempAdjl[index]);
    	    tempAdjl[index] = p;
    	}
        }    
    
    public void deleteNode(int v) {
    	ALNode nodes, temp;
    	adjl[v] = null;
    	for (int i = 0; i < nv; i++) {
    		nodes = adjl[i];
    		if (nodes != null) {//i == v
    			temp = nodes;
        		while(nodes != null) {
        			if (nodes.getVid() == v) {
        				temp.setProx(nodes.getProx());
        				nodes = null;
        				break;
        			}
        			temp = nodes;
        			nodes = nodes.getProx();
        		}    			
    		}
    	}
    }
    
    public void insertEdge (int v1, int v2, double weight) {
    	ALNode node = new ALNode (v2,weight);

    	node.setProx(adjl[v1]);
    	adjl[v1] = node;
        }    
    
    public void checkInsertEdge (int v1, int v2, double weight) {
    	ALNode temp = adjl[v1];
    	while (temp != null) {
    		if (temp.getVid() == v2)
    			return;
    		temp = temp.getProx();
    	}
    	insertEdge(v1, v2, weight);
    }    
    
    public void printTree(int root) {
    	ArrayList<Integer> levels = new ArrayList<Integer>();
    	Stack<Integer> stack = new Stack<Integer>();
    	levels.add(root);
    	System.out.println("imprimindo árvore:");
    	while (!levels.isEmpty()) {
    		int index = levels.remove(0);
    		ALNode node = adjl[index];
    		if (node == null)
    			continue;
    		System.out.print(index+": [");
    		while (node != null) {
    			levels.add(node.getVid());
    			System.out.print(node.getVid()+", ");
    			node = node.getProx();
    		}
    		System.out.println("]");
    	}
    }

    public void printAdjList() {
	int i;
	ALNode node;

	System.out.println ("printing Adjacent List:");
	for (i=0; i < nv; i++) {
		if (adjl[i] == null)
			continue;
	    System.out.print ("Vertice: " + i + ": ");
	    node = adjl[i];
	    while (node != null) {
	    	System.out.print(node.getVid()+"/c: "+node.getWeight()+", ");
		node = node.getProx();
	    }
	    System.out.println();
	}
    }

    public void setWeight(Weight w) {
	ALNode node;

	for (int i=0; i < nv; i++) {
	    node = adjl[i];
	    while (node != null) {
		node.setWeight(w.getWeight(i, node.getVid(), this));
		node = node.getProx();
	    }
	}
    }

    public  void checkCopyList (AdjList nadjl, int index) {
    	ALNode p;

    	for (ALNode q = nadjl.getList(index); q != null; q = q.getProx()) {
    		ALNode temp = adjl[index];
    		boolean flag = false;
    		while (temp != null) {
    			if (temp.getVid() == q.getVid()) {
    				flag = true;
    				break;
    			}
    			temp = temp.getProx();
    		}
    		if (flag)
    			continue;
    	    p = new ALNode(q.getVid(), q.getWeight());
    	    p.setProx(adjl[index]);
    	    adjl[index] = p;
    	}
    }

    public  void copyList (AdjList nadjl, int index) {
	ALNode p;

	for (ALNode q = nadjl.getList(index); q != null; q = q.getProx()) {
	    p = new ALNode(q.getVid(), q.getWeight());
	    p.setProx(adjl[index]);
	    adjl[index] = p;
	}
    }


    public void readFromTopologyFile (String fname, PositionTable pt) {
	String s1, s2;

	try {
	    BufferedReader in = new BufferedReader(new FileReader(fname));
	    String aLine1, aLine2;

	    for (int i = 0; i < nv; i++) {
		aLine1 = in.readLine();
		aLine2 = in.readLine();
		StringTokenizer stx1 = new StringTokenizer(aLine1);
		StringTokenizer stx2 = new StringTokenizer(aLine2);

		s1 = stx1.nextToken();    // read word 'Node:'
		s1 = stx1.nextToken();    // read node id
		s1 = stx1.nextToken();    // read word 'neighbours:'

		s2 = stx2.nextToken();    // read word 'Node:'
		s2 = stx2.nextToken();    // read node id
		s2 = stx2.nextToken();    // read word 'edgecosts:'

		while (stx1.hasMoreTokens()) {
		    s1 = stx1.nextToken();    // read neighbour
		    int nb = Integer.parseInt(s1);

		    s2 = stx2.nextToken();    // read neighbour
		    double nbc = Double.parseDouble(s2);

		    insertEdge (i, nb, nbc);
		}

		aLine1 = in.readLine();   // skip line with optimized neighbour set
	    }
	    in.close();
	} catch (IOException e) {
	}
    }
	

    // Methods related to the implementation of Prim's MST algorithm

    public void MSTPrim() {
	int j;
	PrimKeys = new double[nv];
	PrimParent = new int[nv];
	boolean[] alreadySelected = new boolean[nv];
	int nselected, smstindex, neigb;
	double smstkey;
	ALNode node;

	for (int i = 0; i < nv; i++) {
	    PrimKeys[i] = INFINITO;
	    PrimParent[i] = -1;
	    alreadySelected[i] = false;
	}

	PrimKeys[0] = 0;

	nselected = 0;
	while (nselected < nv - 1) {
	    // Find lowest key:

	    for (j =0; alreadySelected[j] == true; j++);
	    smstkey = PrimKeys[j];
	    smstindex = j;
	    for (j=j+1; j < nv; j++) {
		if ((alreadySelected[j] == false) && (PrimKeys[j] != INFINITO)) {
		    if ((smstkey == INFINITO) || (PrimKeys[j]<smstkey)) {
		    smstkey = PrimKeys[j];  
		    smstindex = j;
		    }
		}
	    }

	    alreadySelected[smstindex] = true;
	    nselected++;
	    node = adjl[smstindex];
	    while (node != null) {
		neigb = node.getVid();
		//	System.out.println ("neigb: " + neigb + " - peson: " + PrimKeys[neigb] + " - pesoa: " +
		//	    node.getWeight());
		if ((alreadySelected[neigb] == false) && 
		    ((PrimKeys[neigb]== INFINITO) || (PrimKeys[neigb] > node.getWeight()))) {
		    PrimKeys[neigb] = node.getWeight();
		    PrimParent[neigb] = smstindex;
		}
		node = node.getProx();
	    }
	}
    }

    public void printMST() {
	System.out.println ("MST:");
	for (int i=0; i < nv; i++) {
	    System.out.print ("No: " + i);
	    if (PrimParent[i] == -1)
		System.out.println (" - no raiz");
	    else
		System.out.println (" - pai: " + PrimParent[i]);
	}
    }

    public void plotMST (PositionTable pt, String fname) {
	try {
	    PrintWriter out = new PrintWriter(new FileWriter(fname));
	    
	    for (int i = 0; i < nv; i++) 
		if (PrimParent[i] != -1) {
		    out.println (pt.getX(i) + " " + pt.getY(i));
		    out.println (pt.getX(PrimParent[i]) + " " + pt.getY(PrimParent[i]));
		    out.println();
		}
	    out.close();
	} catch (IOException e) {
	}
    }
}


    

