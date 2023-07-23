package algorithms;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Iterator;
import java.util.Map;

import support.S2DDelay;

import msc.ElkinItem;
import msc.Greedy;
import msc.Group;
import msc.Node;
import msc.PseudoNode;
import msc.Set;

import dataStructure.ALNode;
import dataStructure.AdjList;
import dataStructure.AdjMatrix;
import dataStructure.PositionTable;
import dataStructure.Weight;

public class MSCBasedAlgorithm extends DSMDStAlgorithm {

	AdjMatrix uninformedAdjMatrix;
	
	HashSet informedSet;
	
	ArrayList<Integer> uninformedTerminals;
	ArrayList<Double> uninformedTerminalsDelay;
	
	Greedy greedy;	
	
	ArrayList<ElkinItem> chosenItem;
	HashSet<PseudoNode> chosenPseudo, realChosenPseudo;
	
	AdjList quasiArborescence;
	
	protected void clearVar() {
		super.clearVar();
		quasiArborescence = new AdjList(size);
		
		uninformedAdjMatrix = new AdjMatrix(size);
		
		informedSet = new HashSet<Integer>();		
	}
	
	public MSCBasedAlgorithm(int nsize, double nmprange, int sourceId,
			int[] terminalsId, double[] delaysTerm, PositionTable npt, Weight w) {
		super(nsize, nmprange, sourceId, terminalsId, delaysTerm, npt, w);
		// TODO Auto-generated constructor stub
		
		quasiArborescence = new AdjList(size);
		
		uninformedAdjMatrix = new AdjMatrix(size);
		
		informedSet = new HashSet<Integer>();
	}
	
	//Add the new informed nodes to the informed set.
	protected void updateInformedSet(AdjList arb, int rootIndex) {
		int i;
		ALNode node;

		for (i=0; i < size; i++) {
			if (arb.getList(i) == null)
				continue;
		    node = arb.getList(i);
		    informedSet.add(i);
		    while (node != null) {
		    	informedSet.add(node.getVid());
			node = node.getProx();
		    }
		}
	}
    
    public ArrayList<Group> setMSCGroups(HashSet v1Set, ArrayList<Integer> uncoveredV2Subset, boolean extraCostForSource) {
    	ArrayList<Group> groups = new ArrayList<Group>();
    	
    	HashSet uncoveredTemp = new HashSet();
    	for (int i = 0; i < uninformedTerminals.size(); i++)
    		uncoveredV2Subset.add(uninformedTerminals.get(i));
    	
    	Iterator<Integer> it = v1Set.iterator();
    	while (it.hasNext()) {
    		Group group = new Group();
    		int vId = it.next();
    		uncoveredTemp.addAll(setMSCGroupParametrized(v1Set, group, vId, uninformedTerminals, uninformedTerminalsDelay, extraCostForSource));
    		uncoveredTemp.retainAll(uncoveredV2Subset);
    		uncoveredV2Subset.clear();
    		uncoveredV2Subset.addAll(uncoveredTemp);
    		uncoveredTemp.clear();
    		if (group.size() > 0) {
        		groups.add(group);    			
    		}
    	}
    	
    	return groups;
    }
    
    public ArrayList<Integer> procDistDom(ArrayList<Group> groups, /*HashSet v1Set, */ArrayList<ElkinItem> v2Set) {
    	
    	ArrayList<ElkinItem> v2SetCopy = new ArrayList<ElkinItem>();
    	for (int i = 0; i < v2Set.size(); i++)
    		v2SetCopy.add(v2Set.get(i).copy());
    	
    	ArrayList<Set> D;
    	greedy = new Greedy(v2SetCopy, groups, uninformedAdjl);
    	D = greedy.MSCGreedyBasedSol();
    	
    	//find a dominating set
    	//I can optimize this part
    	chosenItem = new ArrayList<ElkinItem>(); //it seems to be disnecessary this set
    	chosenPseudo = new HashSet<PseudoNode>(); //it seems to be disnecessary this set
    	for (int d = 0; d < D.size(); d++) {
    		Set set = D.get(d);
    		for (int i = 0; i < set.getItems().size(); i++) {
    			ElkinItem item = set.getItems().get(i);
    			int vId = item.getPseudo().getUninformed().getId();
    			
    			chosenItem.add(item);
    			chosenPseudo.add(item.getPseudo());
    		}
    	}
    	
    	return null;
    }
    
    public void addingTreesForDomSet(ArrayList<Integer> terminalsId) {
    	HashSet<ElkinItem> realChosenItem = new HashSet<ElkinItem>();
    	realChosenPseudo = new HashSet<PseudoNode>();
    	
		int [][]P = uninformedAdjMatrix.getP2PPath();
		double [][]MCPmatrix = uninformedAdjMatrix.getP2PCost();    	
    	
    	for (int i = 0; i < terminalsId.size(); i++) {
    		double minCost = -1;
    		Iterator<ElkinItem> itensIt = chosenItem.iterator();
    		ElkinItem minItem = null;
    		while (itensIt.hasNext()) {
    			ElkinItem item = itensIt.next();
    			if (item.getNode().getId() == terminalsId.get(i).intValue()) {
    				double cost = edgeWeight.getWeight(item.getPseudo().getInformed().getId(), item.getPseudo().getUninformed().getId(), globalAdjl) + MCPmatrix[item.getPseudo().getUninformed().getId()][item.getNode().getId()];
    				if ((cost < minCost) || (minCost == -1)) {
    					minItem = item.copy();
    					minCost = cost;
    				}
    			}
    		}
    		realChosenItem.add(minItem);
    		realChosenPseudo.add(new PseudoNode(new Node(minItem.getPseudo().getInformed().getId()), new Node(minItem.getPseudo().getUninformed().getId())));
    	}
    	
    	//independent part
		Iterator<ElkinItem> itensIt = realChosenItem.iterator();
		while (itensIt.hasNext()) {
			ElkinItem item = itensIt.next();
    		int prev = item.getNode().getId();//terminal
    		
    		int s = item.getPseudo().getUninformed().getId();
    		if (s == prev) { //the uninformed node of the pseudo node is a terminal
    			quasiArborescence.checkInsertEdge(item.getPseudo().getInformed().getId(), prev, edgeWeight.getWeight(item.getPseudo().getInformed().getId(), prev, globalAdjl));
    		} else {
        		while (P[s][prev] != s) {
        			quasiArborescence.checkInsertEdge(P[s][prev], prev, MCPmatrix[P[s][prev]][prev]);
        			prev = P[s][prev];
        		}
        		quasiArborescence.checkInsertEdge(P[s][prev], prev, MCPmatrix[P[s][prev]][prev]);
    		}
    		double cost = edgeWeight.getWeight(item.getPseudo().getInformed().getId(), item.getPseudo().getUninformed().getId(), globalAdjl);
    		quasiArborescence.checkInsertEdge(item.getPseudo().getInformed().getId(), item.getPseudo().getUninformed().getId(), cost);
    	}
    }
    
	public ArrayList<Integer> applyDistDomProcedure(HashSet v1Set, ArrayList<ElkinItem> v2Set) {
    	//finding out dominating set D
		ArrayList<Integer> uncovered = new ArrayList<Integer>();
		ArrayList<Group> groups = setMSCGroups(v1Set, uncovered, true);		
    	return procDistDom(groups, v2Set);
	}    
	
	//uninformedTerminals - contera o ultimo subconjunto de Next que nao tinha sido coberto anteriromente. Ou seja, contera os elementos 
	//retornados pela ultima chamada desta funcao
	//vId - id de um no marcado do conjunto I com menor grau que foi selecionado 
	//v1Set - este conjunto contera, a priori, unmarked nos de I e incrementalmente ira englobando nos marked de I
    protected ArrayList<Integer> setMSCGroupParametrized(HashSet v1Set, Group group, int uId, ArrayList<Integer> uninformedTer, ArrayList<Double> uninformedTerSpanQuant, boolean extraCostForSource) {
    	double[][] P2PCosts = uninformedAdjMatrix.getP2PCost();
    	ALNode adjl = globalAdjl.getList(uId);
    	
    	ArrayList<Integer> uncovered = new ArrayList<Integer>();
    	HashSet covered = new HashSet();

    	for (int i = 0; i < uninformedTer.size(); i++) {
    		uncovered.add(uninformedTer.get(i));
    	}
    	
    	while (adjl != null) {
    		PseudoNode pseudo = null;
    		Set sourceSet = new Set();
    		boolean newPseudo = false;
    		
    		if (!informedSet.contains(new Integer(adjl.getVid()))) {
				pseudo = new PseudoNode(new Node(uId), new Node(adjl.getVid()));
				newPseudo = true;				
    		}
    		
    		if (newPseudo) {
    			for (int i = 0; i < uninformedTer.size(); i++) {
    				double extraCost = 0;
    				if (extraCostForSource)
    					extraCost = edgeWeight.getWeight(source, uId, globalAdjl);
    				if (((P2PCosts[adjl.getVid()][uninformedTer.get(i)] != AdjMatrix.INFINITO) && 
    						(Weight.roundTwoDecimals(extraCost + edgeWeight.getWeight(uId, adjl.getVid(), globalAdjl) + P2PCosts[adjl.getVid()][uninformedTer.get(i)]) <= uninformedTerSpanQuant.get(i))) || 
    						((adjl.getVid() == uninformedTer.get(i)) && (Weight.roundTwoDecimals(extraCost + edgeWeight.getWeight(uId, adjl.getVid(), globalAdjl) + P2PCosts[adjl.getVid()][uninformedTer.get(i)]) <= uninformedTerSpanQuant.get(i)))) {//the uninformed node of the pseudo node is a terminal
	    				sourceSet.addItem(new ElkinItem(pseudo, new Node(uninformedTer.get(i))));
	    				covered.add(uninformedTer.get(i));
    				}
    			}
    			
    			if (sourceSet.size() > 0) { //insert set in source group
    				group.addSet(sourceSet);
    			}
    		}
    		adjl = adjl.getProx();
    	}
    	uncovered.removeAll(covered);
    	return uncovered;
    }  	

}
