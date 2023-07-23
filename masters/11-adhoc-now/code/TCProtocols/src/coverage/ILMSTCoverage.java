package coverage;

import java.util.Set;

import dataStructure.AdjList;
import dataStructure.Weight;

public class ILMSTCoverage extends Coverage {

	public ILMSTCoverage(Weight w) {
		super(w);
		listCov = new UndirectedEdgeInterference();
	}

	@Override
	public Set<Integer> getCoverage(int i, int j, AdjList adjl) {
		// TODO Auto-generated method stub
		return Coverage.getBuckhartCoverage(i, j, adjl, weight);
	}

	@Override
	public void setCoverage(AdjList adjl) {
		listCov.setListCoverage(adjl, new ILMSTCoverage(weight));
	}

}
