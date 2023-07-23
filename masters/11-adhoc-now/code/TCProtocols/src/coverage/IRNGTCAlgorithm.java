package coverage;

import support.Edge;
import dataStructure.ALNode;
import dataStructure.AdjMatrix;
import dataStructure.Weight;

public class IRNGTCAlgorithm extends DirectedGraphTCAlgorithm {

	public IRNGTCAlgorithm(Weight w, int size) {
		super(w, size);
		// TODO Auto-generated constructor stub
	}

	@Override
	protected void execute(AdjMatrix globalAdjMatrix) {
		for (int u = 0;  u < globalAdjl.getNumberOfVertices(); u++) {
			for (ALNode vAln = globalAdjl.getList(u); vAln != null; vAln = vAln.getProx()) {
				if (vAln.getCoverage() == 0)
					globalOptAdjl.insertEdge (u, vAln.getVid(), 0, 0, vAln.getIS());
			}
		}
	}

}
