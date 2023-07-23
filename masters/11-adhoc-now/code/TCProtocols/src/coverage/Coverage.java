package coverage;

import java.util.HashSet;
import java.util.Set;

import radios.RadioParameters;

import dataStructure.ALNode;
import dataStructure.AdjList;
import dataStructure.Weight;


public abstract class Coverage {
	protected ListCoverage listCov;
	protected Weight weight;
	public Coverage(Weight w) {
		weight = w;
	}
	
	//este metodo eh chamado por setGeneric(Assymetric)Coverage
	//when this method is called, there is no coverage set to node (ALNode)
	public abstract Set<Integer> getCoverage(int i, int j, AdjList adjl);
	
	public abstract void setCoverage(AdjList adjl);
	
	public static Set<Integer> getAssymetricBuckhartCoverage(int i, int j, AdjList adjl, Weight w) {
    	double dist = RadioParameters.getRealDistance(w.distance(i, j));
    	Set<Integer> cov = new HashSet<Integer>(adjl.getList(i).getSubListLength());
    	for (ALNode aln = adjl.getList(i); aln != null; aln = aln.getProx()) {
    	    if (w.distance(i, aln.getVid()) <= dist) {
    	    	cov.add(aln.getVid());
    	    }
    	}
    	return cov;
	}
	
	public static Set<Integer> getBuckhartCoverage(int i, int j, AdjList adjl, Weight w) {
    	double dist = RadioParameters.getRealDistance(w.distance(i, j));
    	Set<Integer> cov = new HashSet<Integer>(adjl.getList(i).getSubListLength());
    	for (ALNode aln = adjl.getList(i); aln != null; aln = aln.getProx()) {
    	    if (w.distance(i, aln.getVid()) <= dist) {
    	    	cov.add(aln.getVid());
    	    }
    	}
    	for (ALNode aln = adjl.getList(j); aln != null; aln = aln.getProx()) {
    	    if (w.distance(j, aln.getVid()) <= dist) {
    	    	cov.add(aln.getVid());
    	    }
    	}    	
    	return cov;
	}	
	
	public Weight getWeight() {
		return weight;
	}
}
