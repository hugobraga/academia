package algorithms;

public interface Algorithm {
	public boolean pseudoConstructor();
	
    public abstract boolean execute(double perc);
    
    public double[] getArbCosts();
    
    public double[] getTerViolatedCosts();
    
    public int getArbMaxOutDegree();
    
    public abstract String getName();

	public abstract double[] getSPCosts();
	
	public double[] getSpannerQuant();
	
	public double[] getViolatedTerSpannerQuant();
	
	public double getTotalTerminalsCVR(double minCVR[], double maxCVR[]);
	
	public double getSecondPhaseTerminalsCVR(double minCVR[], double maxCVR[]);
	
	public double getViolatedTerminalsCVR(double minCVR[], double maxCVR[]);
	
	public double getViolatedNodesRatio();

	public abstract double getTermCoverFstPhRatio();
	
}
