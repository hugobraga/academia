package tcAlgorithm;

import coverage.GGCoverage;
import coverage.GGTCAlgorithm;
import dataStructure.PositionTable;
import dataStructure.Weight;

public class GGProtocol extends UndirectedGraphSimpleTCProtocol {

	public GGProtocol(int nsize, PositionTable npt, double nmprange, Weight nedgeWeight) {
		super(nsize, npt, nmprange, nedgeWeight);
		// TODO Auto-generated constructor stub
	}

	@Override
	public boolean execute() {
		// TODO Auto-generated method stub
		return executeTCAlgorithm(new GGCoverage(edgeWeight), new GGTCAlgorithm(edgeWeight, size));
	}
	
	public String getName() {
		return new String("GG");
	}	

}
