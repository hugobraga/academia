package coverage;

import java.util.Set;

import dataStructure.AdjList;
import dataStructure.Weight;


public class XTCCoverage extends Coverage {

	public XTCCoverage(Weight w) {
		super(w);
	}

	@Override
	public Set<Integer> getCoverage(int i, int j, AdjList adjl) {
		return null;
	}

	@Override
	public void setCoverage(AdjList adjl) {		
	}
}
