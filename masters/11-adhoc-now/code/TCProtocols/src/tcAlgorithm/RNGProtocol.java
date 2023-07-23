package tcAlgorithm;

import coverage.RNGCoverage;
import coverage.RNGTCAlgorithm;
import dataStructure.PositionTable;
import dataStructure.Weight;

public class RNGProtocol extends UndirectedGraphSimpleTCProtocol {

	public RNGProtocol(int nsize, PositionTable npt, double nmprange,
			Weight nedgeWeight) {
		super(nsize, npt, nmprange, nedgeWeight);
		// TODO Auto-generated constructor stub
	}

	@Override
	public boolean execute() {
		// TODO Auto-generated method stub
		return executeTCAlgorithm(new RNGCoverage(edgeWeight), new RNGTCAlgorithm(edgeWeight, size));
	}
	
	public String getName() {
		return new String("RNG");
	}	

}
