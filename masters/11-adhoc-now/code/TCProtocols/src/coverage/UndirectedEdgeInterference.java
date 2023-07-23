package coverage;

import java.util.HashSet;
import java.util.Iterator;
import java.util.Set;

import dataStructure.ALNode;
import dataStructure.AdjList;

public class UndirectedEdgeInterference implements ListCoverage {

	@Override
	//seting the traditional (link-based) coverage
	public void setListCoverage(AdjList adjl, Coverage coverage) {
		int nv = adjl.getNumberOfVertices();
    	ALNode node;
    	Set<Integer> covs[][] = new HashSet[nv][nv]; //the coverage for all edges, considering the first index for the transmitter and the second for the receiver 
    	
    	for (int i = 0; i < nv; i++)
    		for (int j = 0; j < nv; j++)
    			covs[i][j] = null;

    	for (int i=0; i < nv; i++) {
    	    node = adjl.getList(i);
    	    while (node != null) {
    	    	int j = node.getVid();
    	    	Set<Integer> cov = coverage.getCoverage(i, j, adjl);
    	    	
    	    	node = node.getProx();
    	    	
    	    	int menor, maior;
    	    	if (j < i) {
    	    		menor = j;
    	    		maior = i;
    	    	} else {
    	    		menor = i;
    	    		maior = j;
    	    	}
    	    	
    		    //checking the converage set existence for an edge
    		    if (covs[menor][maior] == null) {//este par nao existe
    		    	covs[menor][maior] = cov;
    		    } else {
    		    	Set<Integer> value = covs[menor][maior];
    		    	Iterator<Integer> it = cov.iterator();
    		    	while (it.hasNext()) {
    		    		value.add(it.next());
    		    	}
    		    	covs[menor][maior] = value;
    		    }
    	    }
    	} 
    	
    	//setting the coverage in the ALNode structure
    	for (int i=0; i < nv; i++) {
    	    node = adjl.getList(i);
    	    while (node != null) {
    	    	int j = node.getVid();
    	    	int menor, maior;
    	    	if (j < i) {
    	    		menor = j;
    	    		maior = i;
    	    	} else {
    	    		menor = i;
    	    		maior = j;
    	    	}
    	    	node.setCov((Set<Integer>)covs[menor][maior]);
    	    	node = node.getProx();
    	    }
    	}   
	}

}
