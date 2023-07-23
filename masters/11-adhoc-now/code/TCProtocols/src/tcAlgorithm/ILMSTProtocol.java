package tcAlgorithm;

import coverage.ILMSTCoverage;
import coverage.ILMSTTCAlgorithm;
import coverage.IRNGCoverage;
import coverage.IRNGTCAlgorithm;
import dataStructure.AdjList;
import dataStructure.PositionTable;
import dataStructure.Weight;

public class ILMSTProtocol extends DirectedGraphSimpleTCProtocol {

	ILMSTProtocol(int nsize, PositionTable npt, double nmprange, Weight nedgeWeight) {
		super(nsize, npt, nmprange, nedgeWeight);
		// TODO Auto-generated constructor stub
	}

	@Override
	public boolean execute() {
		for (int i=0; i < size; i++) {
			AdjList localAdjl = new AdjList(size);
			localAdjl.copyList(globalMaxPowerAdjl, i);
		}
		// TODO Auto-generated method stub
		return executeTCAlgorithm(new ILMSTCoverage(edgeWeight), new ILMSTTCAlgorithm(edgeWeight, size));
	}

	@Override
	public String getName() {
		return new String("ILMST");
		// TODO Auto-generated method stub
	}

}
