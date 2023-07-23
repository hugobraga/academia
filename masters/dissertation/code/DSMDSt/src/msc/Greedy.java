package msc;

import java.util.ArrayList;
import java.util.Iterator;

import dataStructure.AdjList;

/*
 * Multiple-Set Cover approximation solution by Chekuri2004
 * We use nomenclature presented in Chekuri2004 and Elkin2006
 * */
public class Greedy {
	ArrayList<Set> finalSets;
	ArrayList<ElkinItem> X;
	ArrayList<Group> G;
	AdjList list;
	
	public Greedy(ArrayList<ElkinItem> X, ArrayList<Group> G, AdjList adjl) {
		list = adjl;
		finalSets = new ArrayList<Set>();
		this.X = X;
		this.G = G;
	}
	
	// Considering the alfa = 1
	private Set oracle(Group G, ArrayList<ElkinItem> XSubset) {
		Set maxSet = null;
		int max = 0;
		Iterator<Set> it = G.iterator();
		
    	while (it.hasNext()) {
    		Set s = it.next();
    		int qt;
    		if ((qt = s.intersection(XSubset/*, max*/)) > max) {
    			maxSet = s;
    			max = qt;
    		}
    	}
		return maxSet;
	}
	
	private ArrayList<Set> greedy(ArrayList<ElkinItem> XSet, ArrayList<Group> groups, int k, int []groupK) {
		boolean emptyXSet = false;
		ArrayList<Set> sets = new ArrayList<Set>();
		int qtSelec[] = new int[groups.size()];
		for (int i = 0; i < groups.size(); i++) {
			qtSelec[i] = 0;
		}
		int l = groups.size();
		Set[] A = new Set[l];
		for (int j = 0; (j < k) && (!emptyXSet); j++) {
			for (int i = 0; i < l; i++) {
				Group Gi = groups.get(i);
				if (Gi.marked() == 0) {
					A[i] = oracle(Gi, XSet);
				} else {
					A[i] = null;
				}
			}
			int r = argmax(A);
			if ((r == -1)) {
				emptyXSet = true;
				continue;
			}

			
			sets.add(A[r]);
			Iterator<ElkinItem> it = A[r].iterator();
			while(it.hasNext()) {
				ElkinItem item = it.next();
				XSet.remove(item);//removing element from v2Set
			}
			
			qtSelec[r]++;
			if (qtSelec[r] == groupK[r])
				groups.get(r).mark();
			
			if (XSet.isEmpty()) {
				emptyXSet = true;
			}
		}
		for (int g = 0; g < groups.size(); g++)
			groups.get(g).unMark();		
		return sets;
	}
	
	public ArrayList<Set> MSCGreedyBasedSol() {
		int k, bound;
		int groupK[] = new int[G.size()];
		if (X.size() == 1) {
			k = 1;
			bound = 1;
		} else {
			k = 1;
			bound = X.size();			
		}
		
		int qtIter2CoverXSet = (((int)Math.ceil(Math.log(X.size())))+1);
		
		for (; k <= bound; k++) {//k is being guessing
			for (int i = 0; i < G.size(); i++) {
				groupK[i] = k;
			}
			ArrayList<ElkinItem> XCopy = new ArrayList<ElkinItem>();
			for (int i = 0; i < X.size(); i++) {
				XCopy.add(X.get(i).copy());
			}
			finalSets.clear();
			
			/* v2 represents number of iterations to cover all elements of X.
			 * See the section: A logn + 1 approximation for SCG in Chekuri2004
			 */
			for (int v2 = 0; v2 < qtIter2CoverXSet; v2++) { 
				ArrayList<Set> sets;

				sets = greedy(XCopy, G, k, groupK);//elements of XCopy are being removed inside the function
				for (int s = 0; s < sets.size(); s++) {
					finalSets.remove(sets.get(s));
				}
				finalSets.addAll(sets);
				if (XCopy.size() == 0) {
					return finalSets;	
				}
			}
		}
		return null;
	}
	
	private int argmax(Set[] g) {
		int max = 0;
		int maxId = -1;
		for (int i = 0; i < g.length; i++) {
			if (g[i] == null)
				continue;
			if (g[i].size() > max) {
				max = g[i].size();
				maxId = i;
			}
		}
		return maxId;
	}
}
