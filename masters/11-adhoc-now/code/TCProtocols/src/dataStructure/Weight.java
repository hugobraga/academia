package dataStructure;

import radios.RadioParameters;
  

public class Weight {

    PositionTable pt;
    double elecTx, elecRx, epsilon, pathLossExp;

    public Weight (double nelecTx, double nelecRx, double nepsilon, double npathLossExp, PositionTable npt) {
	elecTx = nelecTx;
	elecRx = nelecRx;
	epsilon = nepsilon;
	pathLossExp = npathLossExp;
	pt = npt;
    }

    public double distance (int v1, int v2) {
	return (Math.sqrt(Math.pow(pt.getX(v1) - pt.getX(v2), 2) + Math.pow(pt.getY(v1) - pt.getY(v2), 2)));
    }
    
    public int nodesInRange (int i, int j, AdjList adjl) {
		double distanceIJ = RadioParameters.getRealDistance(distance(i, j));
		//System.out.println("distanceIJ: "+distanceIJ);
		return nodesInRange(i, j, adjl, distanceIJ);
    }
    
    public int nodesInRange (int i, int j, AdjList adjl, double distanceIJ) {
    	int n = 0;                
    	
    	for (ALNode aln = adjl.getList(i); aln != null; aln = aln.getProx()) {
    		double dist = distance(i, aln.getVid());
    	    if (dist <= distanceIJ) {
    		n++;
    	    }
    	}
    	return n;
        }     
    
    ///*
    public double getWeight(int i, int j, AdjList adjl, double txLevel) {
    	double dist = RadioParameters.getDistance(txLevel);
    	//return (elecTx + epsilon * Math.pow(dist, pathLossExp) + elecRx * nodesInRange (i, j, adjl, dist));
    	return (elecTx + epsilon * Math.pow(dist, RadioParameters.npathLossExp) + elecRx * nodesInRange (i, j, adjl, dist));
    }
		
    public double getWeight (int i, int j, AdjList adjl) {
    	double dist = RadioParameters.getRealDistance(distance(i, j));
    	//double weight = (elecTx + epsilon * Math.pow(dist, pathLossExp) + elecRx * nodesInRange (i, j, adjl));
    	double weight = (elecTx + epsilon * Math.pow(dist, RadioParameters.npathLossExp) + elecRx * nodesInRange (i, j, adjl));
	return weight;
    }
    //*/
    /*desregarding overhearing
    public double getWeight(int i, int j, AdjList adjl, double txLevel) {
    	double dist = RadioParameters.getDistance(txLevel);
    	return (elecTx + epsilon * Math.pow(dist, pathLossExp) + elecRx);
    }
		
    public double getWeight (int i, int j, AdjList adjl) {
    	double dist = RadioParameters.getRealDistance(distance(i, j));
    	double weight = (elecTx + epsilon * Math.pow(dist, pathLossExp) + elecRx);
	return weight;
    }
    */   
}