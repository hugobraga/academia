package dataStructure;

import java.math.BigDecimal;
import java.text.DecimalFormat;


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
    
    public static double round(double unrounded, int precision, int roundingMode)
    {
        BigDecimal bd = new BigDecimal(unrounded);
        BigDecimal rounded = bd.setScale(precision, roundingMode);
        return rounded.doubleValue();
    }


    public static double roundTwoDecimals(double d) {
    	double result = round(d, 2, BigDecimal.ROUND_HALF_UP); 
    	return result;
}    
    
    double distance (int v1, int v2) {
    	//System.out.println("x1: "+pt.getX(v1)+", y1: "+pt.getY(v1)+", x2: "+pt.getX(v2)+", y2: "+pt.getY(v2)+", dist: "+Math.sqrt(Math.pow(pt.getX(v1) - pt.getX(v2), 2) + Math.pow(pt.getY(v1) - pt.getY(v2), 2)));
	return roundTwoDecimals(Math.sqrt(Math.pow(pt.getX(v1) - pt.getX(v2), 2) + Math.pow(pt.getY(v1) - pt.getY(v2), 2)));
    }

    int nodesInRange (int i, int j, AdjList adjl) {
	int n = 0;                
	
	double distanceIJ = distance (i, j);
	for (ALNode aln = adjl.getList(i); aln != null; aln = aln.getProx()) {
	    if (distance(i, aln.getVid()) <= distanceIJ) {
		n++;
	    }
	}
	return n;
    }
		
    public double getWeight (int i, int j, AdjList adjl) {
    	return distance(i, j);
    }

}


