package dataStructure;

public class ALNode {

    int vid;
    double weight;
    ALNode prox;

    ALNode (int vertid, double w) {
	vid = vertid;
	weight = w;
    }

    public int getVid() { return vid; }

    public double getWeight() { return weight; }

    public ALNode getProx() { return prox; }

    public void setWeight (double w) {
	weight = w;
    }

    public void setProx (ALNode p) {
	prox = p;
    }
}
