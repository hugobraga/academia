package tcAlgorithm;

import coverage.XTCCoverage;
import coverage.XTCTCAlgorithm;
import dataStructure.PositionTable;
import dataStructure.Weight;

public class XTCProtocol extends DirectedGraphSimpleTCProtocol {

	public XTCProtocol(int nsize, PositionTable npt, double nmprange,
			Weight nedgeWeight) {
		super(nsize, npt, nmprange, nedgeWeight);
		// TODO Auto-generated constructor stub
	}
	
	public boolean execute() {
		return executeTCAlgorithm(new XTCCoverage(edgeWeight), new XTCTCAlgorithm(edgeWeight, size));

	}
	
	public String getName() {
		return new String("XTC");
	}	

}
