package dataStructure;

import java.io.*;
import java.util.*;

import support.Edge;
import support.EdgeIDs;


public class AdjList {
    
    int nv;
    ALNode[] adjl;
    ALNode[] correctAdjl;
    
    Map<EdgeIDs, Edge> mstL;
    
    int globalSink;
    
    //OurProtocol tcP;
    ALNode nodesNotToAnswer[];
    
    int maxPhaseId;
    int phaseId[];
    int parentId[];
    int level[];
    
    double power[];
    double dataPower[];
    int order[];
    int centerOrder[];
    PositionTable pt;

    double[] PrimKeys;
    int[] PrimParent;
    int[] PrimLevel;
    boolean[] PrimHasChild;
    
    int INFINITO = -1;

    public AdjList(int nverts) {
	int i;

	nv = nverts;
	 adjl = new ALNode[nverts];
	 correctAdjl = new ALNode[nverts];
	 power = new double[nverts];
	 dataPower = new double[nverts];
	 order = new int[nverts];
	 centerOrder = new int[nverts];
	 phaseId = new int[nverts];
	 parentId = new int[nverts];
	 level = new int[nverts];

	 for (i=0; i<nverts; i++) {
	     adjl[i] = null;
	     correctAdjl[i] = null;
	     
	     phaseId[i] = parentId[i] = level[i] = -1;
	 }
	 
	 mstL = new HashMap<EdgeIDs, Edge>();
	 
	 //tcP = null;
    }
    
    public void setTCProtocol(ALNode nodesNotToAnswer[]) {
    	this.nodesNotToAnswer = new ALNode[nv];
    	for (int i = 0; i < nodesNotToAnswer.length; i++) {
    		ALNode aln = nodesNotToAnswer[i];
    		while (aln != null) {
				ALNode node = new ALNode (aln.getVid(), -1, aln.getDistance());
				node.setProx(this.nodesNotToAnswer[i]);
				this.nodesNotToAnswer[i] = node;
				aln = aln.getProx();
    		}
    	}
    }

    public int getNumberOfVertices() { return nv; }

    public ALNode getList (int lindex) { return adjl[lindex]; }
    
    public ALNode getCorrectList (int lindex) { return correctAdjl[lindex]; }

    public void setList (ALNode aln, int index) {
	adjl[index] = aln;
    }
    
    public void setCorrectList (ALNode aln, int index) {
    	correctAdjl[index] = aln;
        }    

    public void setDataPower(int index, double power) {
    	this.dataPower[index] = power;
    	this.power[index] = power;
    }    
    
    public void setPower(int index, double power) {
    	this.power[index] = power;
    }
    
    public double getPower(int index) {
    	return power[index];
    }
    
    public double getDataPower(int index) {
    	return dataPower[index];
    }    
    
    public void setOrder(int index, int order) {
    	this.order[index] = order;
    }
    
    public int getCenterRelatedOrder(int index) {
    	return centerOrder[index];
    }
    
    public void setCenterRelatedOrder(int index, int order) {
    	this.centerOrder[index] = order;
    	if (order == 0)
    		globalSink = index;
    }
    
    public int getOrder(int index) {
    	return order[index];
    }
    
    public void setPhaseId(int index, int phase) {
    	phaseId[index] = phase;
    }
    
    public int getPhaseId (int index) {
    	return phaseId[index];
    }
    
    public void setMaxPhaseId(int max) {
    	maxPhaseId = max;
    }       
    
    public void setParentId (int index, int id) {
    	parentId[index] = id;
    }
    
    public int getParentId(int index) {
    	return parentId[index];
    } 
    
    public void setLevel(int index, int l) {
    	level[index] = l;
    }
    
    
    public void setPt(PositionTable pt) {
    	this.pt = pt;
    }

    public void insertEdge (int v1, int v2, double weight, double distance, int IS) {
	ALNode node = new ALNode (v2,weight, distance);
	node.setIS(IS);

	node.setProx(adjl[v1]);
	adjl[v1] = node;
    }  

    public void printAdjList() {
	int i;
	ALNode node;

	System.out.println ("Graph:");
	for (i=0; i < nv; i++) {
	    System.out.print ("Vertice: " + i + ": ");
	    node = adjl[i];
	    while (node != null) {
	    	System.out.print ("[" + node.getVid()+"] ");
		node = node.getProx();
	    }
	    System.out.println();
	}
    }

    /* 
     * Get the interference based on sender (IS) in the transmition from node i to node j
     * Observes that we consider the real distance due to radio power levels.
    */    
    private int getIS (int i, int j, Weight w) {
    	int is = w.nodesInRange (i, j, this);
    	//System.out.println("is: "+is);
    	return is;
    }  
    
    /*
     * Sets the coverage, based on the node sender-centric interference (IS), between each node and its neighborhood.
     */    
    public void setIS(Weight w) {
    	ALNode node;

    	for (int i=0; i < nv; i++) {
    	    node = adjl[i];
    	    while (node != null) {
    	    	//System.out.println("vizinho de "+i+": "+node.getVid());
    	    	//System.exit(0);
    	    	int j = node.getVid();
    	    	int is = getIS(i, j, w);
	    		node.setIS(is);
	    		node = node.getProx();
    	    }
    	}
        }    

    public void setWeight(Weight w, AdjList globalOptAdjl) {
    	ALNode node;

    	for (int i=0; i < nv; i++) {
    	    node = adjl[i];
    	    while (node != null) {
    	    	int j = node.getVid();
    		node.setWeight(w.getWeight(i, j, this, globalOptAdjl.getPower(i)));
    		node = node.getProx();
    	    }
    	}
        }    
    
    public void setWeight(Weight w) {
	ALNode node;

	for (int i=0; i < nv; i++) {
	    node = adjl[i];
	    while (node != null) {
	    	int j = node.getVid();
		node.setWeight(w.getWeight(i, j, this));
		node = node.getProx();
	    }
	}
    }


    public  void copyList (AdjList nadjl, int index) {
	ALNode p;

	for (ALNode q = nadjl.getList(index); q != null; q = q.getProx()) {
	    p = new ALNode(q.getVid(), q.getWeight(), q.getDistance());
	    p.setIS(q.getIS());
	    p.setProx(adjl[index]);
	    adjl[index] = p;
	}
    }
    
    public void delList(int index) {
    	ALNode aln = adjl[index];
    	while(aln != null) {
    		ALNode temp = aln;
    		aln = aln.getProx();
    		temp = null;
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

		    insertEdge (i, nb, nbc, -1, -1);
		}

		aLine1 = in.readLine();   // skip line with optimized neighbour set
	    }
	    in.close();
	} catch (IOException e) {
	}
    }
    
    /**
     * Use the Breadth-First Search algorithm to check if node id1 and id2 are connected. 
     */    
    public boolean connected(int id1, int id2) {
    	final int WHITE = 0; //not reached
    	final int GRAY = 1; //reached but your children haven't been reached
    	final int BLACK = 2; //all you children have been reached
    	
    	int color[] = new int[nv];
    	int d[] = new int[nv];
    	int pi[] = new int[nv];
    	for (int i = 0; i < nv; i++) {
    		if (i != id1) {
    			color[i] = WHITE;
    			d[i] = INFINITO;
    			pi[i] = -1;
    		}
    	}
    	color[id1] = GRAY;
    	d[id1] = 0;
    	pi[id1] = -1;
    	
    	Queue<Integer> q = new PriorityQueue<Integer>();
    	q.add(id1);
    	while (!q.isEmpty()) {
    		int v = q.poll();
    		//ALNode aln = covAdjl.getList(v);
    		ALNode aln = adjl[v];
    		while (aln != null) {
    			if (color[aln.getVid()] == WHITE) { //reach this vertex for the first time
    				if (aln.getVid() == id2)
    					return true;
    				color[aln.getVid()] = GRAY;
    				d[aln.getVid()] = d[v] + 1;
    				pi[aln.getVid()] = v;
    				q.add(aln.getVid());
    			}
    			aln = aln.getProx();
    		}
    		color[v] = BLACK;
    	}
    	return false;
    } 
    
    // Methods related to the implementation of Prim's MST algorithm

    public void MSTPrim() {
	int j;
	PrimKeys = new double[nv];
	PrimParent = new int[nv];
	PrimLevel = new int[nv];
	PrimHasChild = new boolean[nv];
	boolean[] alreadySelected = new boolean[nv];
	int nselected, smstindex, neigb;
	double smstkey;
	ALNode node;
	boolean flag = false;

	for (int i = 0; i < nv; i++) {
	    PrimKeys[i] = INFINITO;
	    PrimParent[i] = -1;
	    PrimLevel[i] = -1;
	    PrimHasChild[i] = false;
	    alreadySelected[i] = false;
	}

	PrimKeys[0] = 0;

	nselected = 0;
	while (nselected < nv - 1) {
	    // Find lowest key:
		if (!flag) {
			smstindex = globalSink;
			smstkey = PrimKeys[smstindex];
			PrimLevel[smstindex] = 0;
			flag = true;
		} else {
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
		}
	    

	    alreadySelected[smstindex] = true;
	    nselected++;
	    //node = adjl[smstindex];
	    node = correctAdjl[smstindex];
	    //if (node == null)
	    while (node != null) {
		neigb = node.getVid();
			 //   node.getWeight());
		if ((alreadySelected[neigb] == false) && 
		    ((PrimKeys[neigb]== INFINITO) || (PrimKeys[neigb] > node.getWeight()))) {
		    PrimKeys[neigb] = node.getWeight();
		    PrimParent[neigb] = smstindex;
		    //mstL.put(new EdgeIDs(smstindex, neigb), new Edge())
		    if (PrimLevel[smstindex] == -1) {
		    	//if (neigb == 0)
		    	//globalSink = smstindex;
		    	//PrimLevel[smstindex] = 0;
		    	PrimLevel[neigb] = PrimLevel[smstindex]+2;
		    } else {
		    	PrimLevel[neigb] = PrimLevel[smstindex]+1;		    	
		    }
		    PrimHasChild[smstindex] = true;
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


    
