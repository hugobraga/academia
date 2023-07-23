package tcAlgorithm;

import coverage.IRNGCoverage;
import coverage.IRNGTCAlgorithm;
import dataStructure.PositionTable;
import dataStructure.Weight;

public class IRNGProtocol extends DirectedGraphSimpleTCProtocol {

	public IRNGProtocol(int nsize, PositionTable npt, double nmprange, Weight nedgeWeight) {
		super(nsize, npt, nmprange, nedgeWeight);
		// TODO Auto-generated constructor stub
	}

	@Override
	public boolean execute() {
		// TODO Auto-generated method stub
		return executeTCAlgorithm(new IRNGCoverage(edgeWeight), new IRNGTCAlgorithm(edgeWeight, size));
	}

	@Override
	public String getName() {
		// TODO Auto-generated method stub
		return new String("IRNG");
	}

}
