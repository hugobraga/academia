package coverage;

import dataStructure.ALNode;
import dataStructure.AdjList;
import dataStructure.AdjMatrix;
import dataStructure.Weight;

public abstract class TCAlgorithm {
	Weight weight;
	AdjList globalOptAdjl; //global adjacent list after topology control been applied 
	
	public TCAlgorithm(Weight w, int size) {
		globalOptAdjl = new AdjList(size);
		weight = w;
	}
	
	public void executeAndFillMatrixWithIS(AdjMatrix globalAdjMatrix) {
		//dealing only with interference
		int size = globalAdjMatrix.getSize();
		execute(globalAdjMatrix);
		globalAdjMatrix.fillMatrixWithISFromAdjList(globalOptAdjl);
	}
	
    public AdjList getOptimizedSetOfNeighbours() {
    	int size = globalOptAdjl.getNumberOfVertices();
    	
		//getting the optimized set of neighbors
		AdjMatrix globalLocalAdjMatrix = new AdjMatrix(size);
		globalLocalAdjMatrix.fillMatrixFromAdjList(globalOptAdjl);
		double power[] = new double[1];
		ALNode[] reducedNeibrsSet = new ALNode[size];
		for (int i = 0; i < size; i++) {
			globalOptAdjl.setDataPower(i, power[0]);
		}    	
    	return globalOptAdjl;
    }	
	
	protected abstract void execute(AdjMatrix globalAdjMatrix);
}
