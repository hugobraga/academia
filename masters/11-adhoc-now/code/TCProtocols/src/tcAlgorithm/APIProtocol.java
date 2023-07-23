package tcAlgorithm;

import coverage.APICoverage;
import coverage.APITCAlgorithm;
import dataStructure.PositionTable;
import dataStructure.Weight;

public class APIProtocol extends DirectedGraphSimpleTCProtocol {

	public APIProtocol(int nsize, PositionTable npt, double nmprange, Weight nedgeWeight) {
		super(nsize, npt, nmprange, nedgeWeight);
		// TODO Auto-generated constructor stub
	}

	public boolean execute() {
		return executeTCAlgorithm(new APICoverage(edgeWeight), new APITCAlgorithm(edgeWeight, size));

	}
	
	public String getName() {
		return new String("API");
	}
}
