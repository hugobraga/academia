package tcAlgorithm;

import coverage.LifeCoverage;
import coverage.LifeTCAlgorithm;
import dataStructure.PositionTable;
import dataStructure.Weight;

public class LifeProtocol extends UndirectedGraphSimpleTCProtocol {

	public LifeProtocol(int nsize, PositionTable npt, double nmprange,
			Weight nedgeWeight) {
		super(nsize, npt, nmprange, nedgeWeight);
		// TODO Auto-generated constructor stub
	}
	
	public boolean execute() {
		return executeTCAlgorithm(new LifeCoverage(edgeWeight), new LifeTCAlgorithm(edgeWeight, size));

	}
	
	public String getName() {
		return new String("LIFE");
	}	

}
