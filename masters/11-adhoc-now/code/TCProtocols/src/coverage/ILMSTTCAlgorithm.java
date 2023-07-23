package coverage;

import dataStructure.ALNode;
import dataStructure.AdjList;
import dataStructure.AdjMatrix;
import dataStructure.Weight;

public class ILMSTTCAlgorithm extends DirectedGraphTCAlgorithm {

	public ILMSTTCAlgorithm(Weight w, int size) {
		super(w, size);
		// TODO Auto-generated constructor stub
	}

	@Override
	protected void execute(AdjMatrix globalAdjMatrix) {
		int size = globalAdjl.getNumberOfVertices();
		AdjList lAdjlArray[] = new AdjList[size]; 
		for (int i=0; i < size; i++) {
			AdjList localAdjl = new AdjList(size);
			localAdjl.copyList(globalAdjl, i);
			
			for (ALNode v1Aln = globalAdjl.getList(i); v1Aln != null; v1Aln = v1Aln.getProx()) {
				for (ALNode v2Aln = globalAdjl.getList(v1Aln.getVid()); v2Aln != null; v2Aln = v1Aln.getProx()) {
					if (localAdjl.getList(i).checkVidExistence(v2Aln.getVid())) {
						localAdjl.insertEdge(v1Aln.getVid(), v2Aln.getVid(), v2Aln.getCoverage(), v2Aln.getDistance(), v2Aln.getIS());
						
						localAdjl.MSTPrim();
						lAdjlArray[i] = localAdjl;
					}
				}
			}
		}
		
		for (int i = 0; i < size; i++) {
			AdjList localAdjl = lAdjlArray[i];
			
			
		}
	}

}
