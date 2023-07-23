package dataStructure;

import java.util.HashSet;
import java.util.Iterator;
import java.util.Set;


public class ALNode {
	int subListLength;
	
    int vid;
    double weight;
    double dist;
    int coverage;
    int IS;
    Set<Integer> cov; //it will contain the coverage between  the source node and vid (this) node
    Set<Integer> buckhartCov;
    ALNode prox;
    
    double distance;

    public ALNode (int vertid, double w, double distance) {
	vid = vertid;
	weight = w;
	dist = distance;
	subListLength = 0;
    }

    public int getVid() { return vid; }
    
    public double getDistance() {return dist;}

    public double getWeight() { return weight; }

    public ALNode getProx() { return prox; }
    
    public void setCoverage(int cov) {
    	coverage = cov;
    }
    
    public void setCov(Set<Integer> s) {
    	cov = new HashSet<Integer>(s.size());
    	Iterator<Integer> it = s.iterator();
    	while (it.hasNext()) {
    		cov.add(it.next());
    	}
    }
    
    public void setBuckhartCov(Set<Integer> s) {
    	buckhartCov = new HashSet<Integer>(s.size());
    	Iterator<Integer> it = s.iterator();
    	while (it.hasNext()) {
    		buckhartCov.add(it.next());
    	}    	
    }
    
    public void setIS(int is) {
    	this.IS = is;
    }
    
    public int getIS() {
    	return IS;
    }
    
    //generic coverage (could be even the buckhart coverage)
    public int getCoverage(){
    	int size = cov.size();
    	cov.clear();
    	return size;
    }
    
    public int getBuckhartCoverage(){
    	if (buckhartCov == null)
    		return 0;
    	else {
	    	int size = buckhartCov.size();
	    	buckhartCov.clear();
	    	return size;
    	}
    }

    public void setWeight (double w) {
	weight = w;
    }
    
    public boolean checkVidExistence(int id) {
    	//System.out.println("checking existence");
    	ALNode aln = this;
    	while (aln.getVid() != id) {
    		aln = aln.getProx();
    		if (aln == null)
    			return false;
    	}
    	return true;
    }
    
    public int getSubListLength() {
    	return subListLength;
    }

    public void setProx (ALNode p) {
	prox = p;
	if (p != null)
		subListLength = p.getSubListLength() + 1;
    }
    
    public  void copyList (ALNode aln, int index) {
    	ALNode p;

    	for (ALNode q = aln; q != null; q = q.getProx()) {
    	    p = new ALNode(q.getVid(), q.getWeight(), q.getDistance());
    	    p.setProx(aln);
    	    aln = p;
    	}
        }
}
	
    