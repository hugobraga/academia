package algorithms;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.Iterator;
import java.util.Stack;

import support.DegreeComparator;
import support.NodeDegree;
import support.S2DDelay;
import support.S2DDelayComparator;

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

public class SIM extends MSCBasedAlgorithm implements Algorithm {

	ArrayList<S2DDelay> sortedTerminals;
	int[] violatedTerminals;
	boolean flagViolatedTerminals;
	
	boolean finalMatrix;
	double qtViolated;
	
	public SIM(int nsize, double nmprange, int sourceId, int[] terminalsId,
			double[] delaysTerm, PositionTable npt, Weight w) {
		super(nsize, nmprange, sourceId, terminalsId, delaysTerm, npt, w);
		// TODO Auto-generated constructor stub
		
		sortedTerminals = new ArrayList<S2DDelay>(terminals.length);
		flagViolatedTerminals = false;
	}
	
	protected void clearVar() {
		super.clearVar();
		
		finalMatrix = false;
		sortedTerminals = new ArrayList<S2DDelay>(terminals.length);
		flagViolatedTerminals = false;
	}	

	private ArrayList<S2DDelay> getNextSqrtTerm() {
		//get Next sqrt(k) terminals
		int sqrtTerm;
		if (terminals.length == 1) {
			sqrtTerm = 1;
		} else
			sqrtTerm = (int)Math.floor(Math.sqrt(terminals.length));
		ArrayList<S2DDelay> nextList = new ArrayList<S2DDelay>(sqrtTerm);
		for (int i = 0; (i < sqrtTerm) && (sortedTerminals.size() > 0); i++) {
			nextList.add(sortedTerminals.remove(0));
		}
		return nextList;
	}
	
	private void setViolatedTerminals() {
		violatedTerminals = new int[sortedTerminals.size()];
		for (int i = 0; i < sortedTerminals.size(); i++)
			violatedTerminals[i] = sortedTerminals.get(i).getDestination();
	}

	private ArrayList<Group> setV1ToCoverNextTer(HashSet v1Marked, HashSet v1Set, ArrayList<Integer> uncoveredV2) {
		//v1Marked = conjuntos dos nos de I que ja foram marcados
		ArrayList<NodeDegree> sortedMarked = new ArrayList<NodeDegree>();
			Iterator<Integer> it = v1Marked.iterator();
			while(it.hasNext()) {
				int vId = it.next();
				sortedMarked.add(new NodeDegree(vId, quasiArborescence.getOutDegree(vId)));
			}
		//}
		
		Collections.sort(sortedMarked, new DegreeComparator());
		
		ArrayList<Group> groups = new ArrayList<Group>();
		
		ArrayList<Integer> newUncoveredV2 = uncoveredV2;
		
		for (int i = 0; (i < sortedMarked.size()) && (newUncoveredV2.size() > 0); i++) {
			Group group = new Group();
			
			ArrayList<Integer> uncoveredTerminals = new ArrayList<Integer>();
			for (int z = 0; z < newUncoveredV2.size(); z++) {
				int uncId = newUncoveredV2.get(z);
				uncoveredTerminals.add(uncId);
			}
			
			ArrayList<Double> uncoveredTerminalsDelay = new ArrayList<Double>();
			for (int s = 0; s < uncoveredTerminals.size(); s++) {
				for (int t = 0; t < uninformedTerminals.size(); t++) {
					if (uncoveredTerminals.get(s).intValue() == uninformedTerminals.get(t).intValue()) {
						uncoveredTerminalsDelay.add(uninformedTerminalsDelay.get(t));
						break;
					}
				}
			}
			
			newUncoveredV2 = setMSCGroupParametrized(v1Set, group, sortedMarked.get(i).getId(), uncoveredTerminals, uncoveredTerminalsDelay, false);
			if (newUncoveredV2.size() < uncoveredTerminals.size()) {
				groups.add(group);
			}//else - no uncoverd terminal turned to be covered
		}
		return groups;
	}
	
	@Override
	public boolean execute(double perc) {
		clearVar();
		
		preExecute(perc);
    	
		//sorting terminals list
		for (int i = 0; i < terminals.length; i++) {
			sortedTerminals.add(new S2DDelay(source, terminals[i], spTerCosts[i]));
		}
		Collections.sort(sortedTerminals, new S2DDelayComparator());
		HashSet unmarked, marked;
		
		
		unmarked = new HashSet<Integer>();
		marked = new HashSet<Integer>();
		unmarked.add(source);
		informedSet.add(new Integer(source));
		
		uninformedAdjl.deleteNode(source);
    	uninformedAdjMatrix.fillMatrixFromAdjList(uninformedAdjl);
    	uninformedAdjMatrix.minCostPaths();		
		
		while(sortedTerminals.size() > 0) {
			ArrayList<S2DDelay> nextList = getNextSqrtTerm();
			
			uninformedTerminals = new ArrayList<Integer>(nextList.size());
			uninformedTerminalsDelay = new ArrayList<Double>(nextList.size());
			
			ArrayList<ElkinItem> v2Set = new ArrayList<ElkinItem>(nextList.size());
			//building v2Set
			for (int i = 0; i < nextList.size(); i++) {
				v2Set.add(new ElkinItem(null, new Node(nextList.get(i).getDestination())));
				uninformedTerminals.add(nextList.get(i).getDestination());
				uninformedTerminalsDelay.add(nextList.get(i).getDelay());
			}
			
			ArrayList<Integer> uncoveredV2 = new ArrayList<Integer>();
			//unmarked = subconjunto dos nos de I nao-marcados
			//uncoveredV2 = subset de uninformedTerminals que nao possuem arestas para os mesmos
			ArrayList<Group> groups = setMSCGroups(unmarked, uncoveredV2, false);
			if (uncoveredV2.size() > 0) {
				groups.addAll(setV1ToCoverNextTer(marked, informedSet, uncoveredV2));
			}
			
			//22/03 - It seems like this function is only important to set chosenItem
			procDistDom(groups, v2Set);
			
	    	//adding new edges to the arborescence
	    	addingTreesForDomSet(uninformedTerminals);
	    	
	    	updateInformedSet(quasiArborescence, source);
	    	
	    	//updating marked and unmarked set
	    	Iterator<PseudoNode> markedIt = realChosenPseudo.iterator();
	    	while (markedIt.hasNext()) {
	    		PseudoNode ps = markedIt.next();
	    		int value = ps.getInformed().getId();
	    		marked.add(value);
	    	}
	    	unmarked.addAll(informedSet);
	    	unmarked.removeAll(marked);
	    	
	    	//updating sortedTerminals and uninformedAdjl lists by eliminating nodes
	    	for (int j = 0; j < size; j++) {
	    		if ((quasiArborescence.getList(j) != null) //if there is a list for j in quasiArborescence then this node is in the final arborescence and it is not an uninformed terminal anymore  
	    				|| (uninformedTerminals.contains(new Integer(j)))) { //if j was one of the next terminals to be removed, so it is not an uninformed terminal anymore
	    			uninformedAdjl.deleteNode(j);
	    		}
	    	}
	    	//deleting terminals which were covered even they were not members of nextList
	    	deleteCoveredTerminals();
	    	
        	uninformedAdjMatrix.refillMatrix();
        	uninformedAdjMatrix.minCostPaths();	
        	
        	if (!flagViolatedTerminals) {
        		flagViolatedTerminals = true;
        		setViolatedTerminals();
        	}
		}
		
		return true;
	}
	
	public void executeSPT() {
		arborescenceSPT = buildSPT2Terminals(globalAdjMatrix, globalAdjl, terminals);
	}
	
	public int getSPTMaxOutDegree() {
		return getMaxOutDegree(arborescenceSPT);		
	}
	
	public double[] getSPTCosts() {
		return getCosts(globalAdjMatrix, terminals);
	}	
	
	public int getArbMaxOutDegree() {
		return getMaxOutDegree(quasiArborescence);
	}
	
	
	private void setFinalMatrix(AdjList arb) {
		if (!finalMatrix) {
			uninformedAdjMatrix.fillMatrixFromAdjList(arb);
			uninformedAdjMatrix.minCostPaths();	
			finalMatrix = true;
		}
	}	
	
	public double getTotalTerminalsCVR(double minCVR[], double maxCVR[]) {
		setFinalMatrix(quasiArborescence);
		
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
		setFinalMatrix(quasiArborescence);
		
		double terFinalCosts[] = getCosts(uninformedAdjMatrix, violatedTerminals);
		double terSpannerCosts[] = getSpannerQuant(violatedTerminals);
		
		double ratios = 0;
		
		minCVR[0] = 0;
		maxCVR[0] = 0;		
		
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
	
	public double getViolatedTerminalsCVR(double minCVR[], double maxCVR[]) {
		///*
		setFinalMatrix(quasiArborescence);
		
		double terFinalCosts[] = getCosts(uninformedAdjMatrix, violatedTerminals);
		double terSpannerCosts[] = getSpannerQuant(violatedTerminals);
		
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
		else
			return (ratios/qtViolated);
	}
	
	public double getViolatedNodesRatio() {
		return qtViolated/terminals.length;
	}
	
	
	public double[] getArbCosts() {
		uninformedAdjMatrix.fillMatrixFromAdjList(quasiArborescence);
		uninformedAdjMatrix.minCostPaths();
		return getCosts(uninformedAdjMatrix, terminals);
	}
	
	@Override
	public double[] getTerViolatedCosts() {
		return getCosts(uninformedAdjMatrix, violatedTerminals);
	}
	
	public double[] getViolatedTerSpannerQuant() {
		return getSpannerQuant(violatedTerminals);
	}	
	
	
	//Delete terminals from sortedTerminals list which were covered out of order 
	void deleteCoveredTerminals() {
    	ArrayList<Integer> levels = new ArrayList<Integer>();
    	levels.add(source);
    	while (!levels.isEmpty()) {
    		int index = levels.remove(0);
    		for (int i = 0; i < sortedTerminals.size(); i++) {
    			if (sortedTerminals.get(i).getDestination() == index) {    				
    				sortedTerminals.remove(i);
    				break;
    			}
    		}
    		ALNode node = quasiArborescence.getList(index);
    		if (node == null)
    			continue;
    		while (node != null) {
    			levels.add(node.getVid());
    			node = node.getProx();
    		}
    	}		
	}

	@Override
	public String getName() {
		// TODO Auto-generated method stub
		return new String("SIM");
	}

}
