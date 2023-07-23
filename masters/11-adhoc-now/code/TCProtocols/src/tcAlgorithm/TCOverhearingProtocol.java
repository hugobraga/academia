package tcAlgorithm;

import java.util.Arrays;

import main.RunTCAlgorithm;

import dataStructure.ALNode;
import dataStructure.AdjList;
import dataStructure.AdjMatrix;
import dataStructure.Position;
import dataStructure.PositionTable;
import dataStructure.Weight;

import radios.RadioParameters;
import support.PositionComparator;

public class TCOverhearingProtocol extends TCProtocol {
	public static final int ASSYMMETRIC_APPROACH_A = 0;
	public static final int SYMMETRIC_APPROACH_A = 1;
	public static final int SYMMETRIC_APPROACH_B = 2;
	public static final int SYMMETRIC_APPROACH_A_NO_ANSWER = 3;
	public static final int TOTAL_SYMMETRIC_APPROACH = 4;
	
	//protected AdjList globaTCOverlAdjl; //global adjacent list after TCOverhearing been applied
	
	protected AdjList localAdjl;
	
	protected AdjMatrix localAdjMatrix;
	
	protected Position[] positions;
	protected Position positions2Center[];	
	
	protected AdjList allLocalAdjl[];
    
	protected ALNode nodesToNotAnswer[];	
	
	protected int symmetricApproach;	
	
	protected double qtKOverhearing[];
	protected double qtMaxOverhearing[];
	protected double hops[];
	protected double txConsumptionRatio[];
	protected double meanKOverhearing[], meanMaxOverhearing[], meanHops[], meanTxConsumptionRatio[];
	protected double percentMultihop[];	

	public TCOverhearingProtocol(int nsize, PositionTable npt, double nmprange,
			Weight nedgeWeight) {
		super(nsize, npt, nmprange, nedgeWeight);
		// TODO Auto-generated constructor stub
		
		symmetricApproach = TCOverhearingProtocol.SYMMETRIC_APPROACH_B;
		
		qtKOverhearing = new double[10];
		qtMaxOverhearing = new double[10];
		hops = new double[10];
		txConsumptionRatio = new double[4];
		
		percentMultihop = new double[19];
		for (int i = 0; i < 19; i++)
			percentMultihop[i] = 0;
		
		meanKOverhearing = new double[10];
		meanMaxOverhearing = new double[10];
		meanHops = new double[10];
		meanTxConsumptionRatio = new double[4];	
		
		positions = new Position[size];
		
		nodesToNotAnswer = new ALNode[size];
		
		allLocalAdjl = new AdjList[size];		
		
		initVars(0);
	}
	
    public double[] getMeanQtKOverhearing() {
    	return meanKOverhearing;
    }
    
    public double[] getMeanQtMaxOverhearing() {
    	return meanMaxOverhearing;
    }
    
    public double[] getMeanHops() {
    	return meanHops;
    }
    
    public double[] getMeanTxConsumptionRatio() {
    	return txConsumptionRatio;
    }
    
    public double[] getPercentMultihop() {
    	return percentMultihop;
    }	
	
    protected void initVars(int init) {
    	for (int i = 0; i < 10; i++) {
    		qtKOverhearing[i] = 0;
    		qtKOverhearing[i] = 0;
    		qtMaxOverhearing[i] = 0;
    		
    		meanKOverhearing[((i%4)+init)] = 0;
    		meanMaxOverhearing[((i%4)+init)] = 0;
    		meanHops[((i%4)+init)] = 0;
    		
    		int temp = init/2;
    		txConsumptionRatio[((i%2)+temp)] = 0;
    		meanTxConsumptionRatio[((i%2)+temp)] = 0;
    	}    	
    }	
	
    public double distance2Center(int v) {
    	double x1, y1, x2, y2;
    	boolean flag = false;
    	
    	x1 = pt.getX(v);
    	y1 = pt.getY(v);
    	x2 = RunTCAlgorithm.FIELD_X/2;
    	y2 = RunTCAlgorithm.FIELD_Y/2;
    	
    	if ((x2 < x1) || ((x1 == x2) && (y2 < y1)))
    		flag = true;
    	if (flag)
    		return (Math.sqrt(Math.pow(x1 - x2, 2) + Math.pow(y1 - y2, 2)));
    	else
    		return (Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2)));    	
    }	
    
    public ALNode[] getNodesNotToAnswer() {
    	return nodesToNotAnswer;
    }    
	
    private void adjustReducedNeighbors(int i, ALNode[] reducedNeibrsSet) {
		if (reducedNeibrsSet[i] != null) {
			double newReachDist = RadioParameters.getDistance(globalOptAdjl.getPower(i));
			ALNode aln = globalMaxPowerAdjl.getList(i);
			while (aln != null) {
				double dist = distance(i, aln.getVid());
				if (dist <= newReachDist) {//out of reach				
					if (!reducedNeibrsSet[i].checkVidExistence(aln.getVid())) {
						//a informacao do peso nao corresponde a nova realidade
						ALNode node = new ALNode (aln.getVid(), aln.getWeight(), aln.getDistance());
						node.setProx(reducedNeibrsSet[i]);
						reducedNeibrsSet[i] = node;			
					}
				}
				aln = aln.getProx();
			}
		}
    }
    
    private void setNodesToNotAnswer(int i, ALNode[] reducedNeibrsSet) {
    	double newReachDist = RadioParameters.getDistance(globalOptAdjl.getPower(i));
		if (reducedNeibrsSet[i] != null) {
			ALNode aln = globalMaxPowerAdjl.getList(i);
			while (aln != null) {
				double dist = distance(i, aln.getVid());
				if (dist <= newReachDist) {//able to be reached				
					if (!reducedNeibrsSet[i].checkVidExistence(aln.getVid())) {
						//a informacao do peso nao corresponde a nova realidade
						//ALNode node = new ALNode (aln.getVid(), -1);
						ALNode node = new ALNode (i, -1, dist);
						//node.setProx(nodesToNotAnswer[i]);
						node.setProx(nodesToNotAnswer[aln.getVid()]);
						
						//nodesToNotAnswer[i] = node;
						nodesToNotAnswer[aln.getVid()] = node;//2
					}
				}
				aln = aln.getProx();
			}
		}    	
    } 	

    @Override
    public boolean execute() {    	
		if (!buildLocalInformation())
			return false;
		
		globalMaxPowerAdjl.setWeight(edgeWeight);

    	ALNode[] reducedNeibrsSet = new ALNode[size];
    	int RSNLenght[], NewRSNLenght[];
    	RSNLenght = new int[size];
    	NewRSNLenght = new int[size];	

    	globalOptAdjl = new AdjList(size);
    	globalOptAssymAdjl = new AdjList(size);
    	
    	int qtStatisticalNodes = 0;
    	
    	double minReachedDist = INFINITO;
    	double maxReachedDist = -INFINITO;
    	
    	for (int i=0; i < size; i++) {
    	    //System.out.println ("Solving for node " + i + " ...");
    	    // build local adjacent list:
    	    localAdjl = new AdjList(size);
    	    localAdjl.copyList(globalMaxPowerAdjl, i);
    	    //localAdjl contem a topologia (vizinhanca) do no i e dos vizinhos do no i
    	    ALNode aln = globalMaxPowerAdjl.getList(i);
    	    while (aln != null) {
    		localAdjl.copyList(globalMaxPowerAdjl, aln.getVid());
    		aln = aln.getProx();
    	    }  	    
    	    
    	    allLocalAdjl[i] = localAdjl;
    	
    	    // build local adjacent matrix:
    	    localAdjMatrix = new AdjMatrix(size);
    	    localAdjMatrix.fillMatrixFromAdjList(localAdjl);
    	    
    	    //localAdjl.delList(i);
    	    //System.gc();
    	    //localAdjl = null;

    	    // calculate minimum cost paths (locally at node i):
    	    localAdjMatrix.minCostPaths();
    	    
    	    //calc perMultiHop, perInequation, qtHops
    	    ///*
    	    //System.out.println("antes de chamar calc, perMultihop[13]: "+percentMultihop[13]);
    	    localAdjMatrix.calcMeanOverhearing(i, edgeWeight, globalMaxPowerAdjl, farthestNeighbor, hops, qtKOverhearing, qtMaxOverhearing, txConsumptionRatio, percentMultihop);
    	    
    	    if (hops[4] > 0) {//existe vizinho
    	    	qtStatisticalNodes++;
    	    	txConsumptionRatio[2] += txConsumptionRatio[0];
    	    	txConsumptionRatio[3] += txConsumptionRatio[1];
    	    }
    	    for (int s = 0; s < 4; s++) {
    	    	hops[s+6] += hops[s];
    	    	qtKOverhearing[s+6] += qtKOverhearing[s];
    	    	if (qtMaxOverhearing[4] > 0)
    	    		qtMaxOverhearing[s+6] += qtMaxOverhearing[s];
    	    	else
    	    		qtMaxOverhearing[s+6] += 0;
    	    }
    	    //*/

    	    double power[] = new double[1];
    	    reducedNeibrsSet[i] = localAdjMatrix.getReducedNeibrsFromMCPMatrix(i, edgeWeight, power);
    	    
    	    if (reducedNeibrsSet[i] != null)
    	    	NewRSNLenght[i] = reducedNeibrsSet[i].getSubListLength() + 1;
    	    else
    	    	NewRSNLenght[i] = 0;
    	    
    	    ///*
    	    ALNode alnNew = reducedNeibrsSet[i];
    	    while (alnNew != null) {
    	    	globalOptAdjl.insertEdge(i, alnNew.getVid(), alnNew.getWeight(), alnNew.getDistance(), alnNew.getIS());
    	    	alnNew = alnNew.getProx();
    	    }
    	    globalOptAdjl.setDataPower(i, power[0]);
    	    
    	    double dist = RadioParameters.getDistance(power[0]);
    	    if (dist < minReachedDist)
    	    	minReachedDist = dist;
    	    else if (dist > maxReachedDist) {
    	    	maxReachedDist = dist;
    	    }
    	    percentMultihop[17] += dist;
    	    
    	    //globaTCOverlAdjl.setCoverage(edgeWeight);
    	    //globaTCOverlAdjl.setIS(edgeWeight);	    
    	     //*/
    	    
    	    
    	    //globalOptAdjl.setList(reducedNeibrsSet[i], i);
    	    //globalOptAdjl.setList(localAdjMatrix.getReducedNeibrsFromMCPMatrix(i, this, power), i);
    	    //globalOptAdjl.setDataPower(i, power[0]);
    	    //*/
    	    positions[i] = new Position(i, distance(i, -1));
    	    positions[i].setDistance2Center(distance2Center(i));

    	    if (i < size -1) {
    		localAdjMatrix.makeEmpty();
    	    }
    	}
    	
    	/**
    	 *now we are filling the global matrix with IS considering with and without the topology control being applied in order
    	 * to calculate the PICS spanning factor
    	 */
    	globalAdjMatrix = new AdjMatrix(size);
    	globalAdjMatrix.fillMatrixWithISFromAdjList(globalOptAdjl);	
    	
    	globalMaxPowerAdjMatrix = new AdjMatrix(size);
    	globalMaxPowerAdjMatrix.fillMatrixWithISFromAdjList(globalMaxPowerAdjl);
    	
    	globalMaxPowerAdjMatrix.minCostPaths();
    	globalAdjMatrix.minCostPaths();
    	calcPICS();
    	
    	/**
    	 * Now we are filling the global matrix with standard transmission cost information in order to calculate how 
    	 */
    	
    	//System.out.println("perMulti: "+percentMultihop[0]+", perIneq: "+percentMultihop[1]+", fracK: "+ percentMultihop[2]);
    	percentMultihop[0] = percentMultihop[0]/qtStatisticalNodes;
    	percentMultihop[1] = percentMultihop[1]/qtStatisticalNodes;
    	percentMultihop[2] = percentMultihop[2]/qtStatisticalNodes;
    	percentMultihop[3] = percentMultihop[3]/qtStatisticalNodes;
    	percentMultihop[4] = percentMultihop[4]/qtStatisticalNodes;
    	percentMultihop[5] = percentMultihop[5]/qtStatisticalNodes;
    	percentMultihop[6] = percentMultihop[6]/qtStatisticalNodes;
    	percentMultihop[7] = percentMultihop[7]/qtStatisticalNodes;
    	percentMultihop[8] = percentMultihop[8]/qtStatisticalNodes;
    	percentMultihop[9] = percentMultihop[9]/qtStatisticalNodes;
    	percentMultihop[10] = percentMultihop[10]/qtStatisticalNodes;
    	percentMultihop[11] = percentMultihop[11]/qtStatisticalNodes;    	
    	percentMultihop[12] = percentMultihop[12]/qtStatisticalNodes;
    	percentMultihop[13] = percentMultihop[13];
    	//percentMultihop[14] = percentMultihop[14]/qtStatisticalNodes;
    	percentMultihop[15] = minReachedDist;
    	percentMultihop[16] = maxReachedDist;
    	percentMultihop[17] = percentMultihop[17]/qtStatisticalNodes;
    	percentMultihop[18] = percentMultihop[18]/qtStatisticalNodes;
    	//System.out.println("perMulti: "+percentMultihop[0]+", perIneq: "+percentMultihop[1]+", fracK: "+ percentMultihop[2]);
    	
        for (int s = 6; s < 10; s++) {
        	hops[s] = hops[s]/qtStatisticalNodes;
        	qtKOverhearing[s] = qtKOverhearing[s]/qtStatisticalNodes;
        	qtMaxOverhearing[s] = qtMaxOverhearing[s]/qtStatisticalNodes;
        	//System.out.println("valor final de hops[s]: "+hops[s]);
        }
        txConsumptionRatio[2] = txConsumptionRatio[2]/qtStatisticalNodes;
        txConsumptionRatio[3] = txConsumptionRatio[3]/qtStatisticalNodes;
        
        
    	for (int t = 0; t < 4; t++) {
    		meanKOverhearing[t] = qtKOverhearing[t+6];
    		meanMaxOverhearing[t] = qtMaxOverhearing[t+6];
    		meanHops[t] = hops[t+6];
    		//System.out.println("valor de meanHops[t]: "+meanHops[t]);
    	}
    	meanTxConsumptionRatio[0] = txConsumptionRatio[2];
    	meanTxConsumptionRatio[1] = txConsumptionRatio[3];
    	
    	//System.out.println("kOver: "+meanKOverhearing[3]+", maxOver: "+meanMaxOverhearing[3]);
    	
    	//System.out.println("available memory: "+Runtime.getRuntime().freeMemory());
    	
    	if (1 == 1)
    		return true;	
    	
    	positions2Center = positions.clone();
    	Arrays.sort(positions, new PositionComparator(false));
    	Arrays.sort(positions2Center, new PositionComparator(true));
    	
    	for (int i = 0; i < size; i++) {
    		globalOptAdjl.setList(reducedNeibrsSet[i], i);
    		//System.out.println("para i: "+i+", index: "+positions[i].getIndex());
    		//globalOptAdjl.setOrder(positions[i].getIndex(), globalOptAdjl.getOrder(positions[i].getIndex()));
    		globalOptAdjl.setOrder(positions[i].getIndex(), i);
    		globalOptAdjl.setCenterRelatedOrder(positions2Center[i].getIndex(), i);
    		
    		globalMaxPowerAdjl.setOrder(positions[i].getIndex(), i);
    		globalMaxPowerAdjl.setCenterRelatedOrder(positions2Center[i].getIndex(), i);
    	}
    	
    	globalOptAdjl.setPt(pt);
    	globalMaxPowerAdjl.setPt(pt);
    	

    	//globalOptAdjl.setTCProtocol(nodesToNotAnswer);
    	//ALNode teste = globalOptAdjl.getCorrectList(0);
    	//if (teste == null)
    	//	System.out.println("eh nulo");
    	if (1 == 1)
    	return true;
    	
    	/*
        for (int s = 6; s < 10; s++) {
        	hops[s] = hops[s]/qtStatisticalNodes;
        	qtKOverhearing[s] = qtKOverhearing[s]/qtStatisticalNodes;
        	qtMaxOverhearing[s] = qtMaxOverhearing[s]/qtStatisticalNodes;
        	//System.out.println("valor final de hops[s]: "+hops[s]);
        }
        txConsumptionRatio[2] = txConsumptionRatio[2]/qtStatisticalNodes;
        txConsumptionRatio[3] = txConsumptionRatio[3]/qtStatisticalNodes;
        
        
    	for (int t = 0; t < 4; t++) {
    		meanKOverhearing[t] = qtKOverhearing[t+6];
    		meanMaxOverhearing[t] = qtMaxOverhearing[t+6];
    		meanHops[t] = hops[t+6];
    		//System.out.println("valor de meanHops[t]: "+meanHops[t]);
    	}
    	meanTxConsumptionRatio[0] = txConsumptionRatio[2];
    	meanTxConsumptionRatio[1] = txConsumptionRatio[3];
    	*/    
    	
    	
    	//this vector is important to know the real communication graph. Through this vector you can plot the reduced graph
    	/*
    	for (int i = 0; i < size; i++) {
    		ALNode p, alnList = null;
        	for (ALNode q = reducedNeibrsSet[i]; q != null; q = q.getProx()) {
        	    p = new ALNode(q.getVid(), q.getWeight());
        	    p.setProx(alnList);
        	    alnList = p;
        	}		
    		globalOptAssymAdjl.setList(alnList, i);
    	}	
    	 */
    	
    	///*
    	//due to discrete tx levels, some nodes can be reached and they weren't added to to ReducedNeibrsSet list. So they must be added
    	//Now I think this is useless because the tx levels are considered in the edge cost
    	//we need to do this because the routing tree can be constructed in any way. So if a node becomes a son of another node it has to be able to reach its father
    	//if (symmetricApproach != OurProtocol.SYMMETRIC_APPROACH_A)
    	for (int i=0; i < size; i++) {
    		adjustReducedNeighbors(i, reducedNeibrsSet);
    	}
    	
    	//globalOptAdjl.printAdjList();
    	
    	/*
    	//assymmetric graph
    	for (int i = 0; i < size; i++) {
    		ALNode p, alnList = null;
        	for (ALNode q = reducedNeibrsSet[i]; q != null; q = q.getProx()) {
        	    p = new ALNode(q.getVid(), q.getWeight());
        	    p.setProx(alnList);
        	    alnList = p;
        	}		
    		globalOptAssymAdjl.setList(alnList, i);
    	}	
    	*/
    	
    	//globalOptAssymAdjl.printAdjList();

    	if ((symmetricApproach == TCOverhearingProtocol.SYMMETRIC_APPROACH_B) || (symmetricApproach == TCOverhearingProtocol.SYMMETRIC_APPROACH_A)) {
    		double maxTxPower[] = new double[size];
    		
    		for (int t = 0; t < size; t++)
    			maxTxPower[t] = globalOptAdjl.getPower(t);		
    		
    		double maxDistance[] = new double[size];
    		for (int t = 0; t < size; t++)
    			maxDistance[t] = 0;
    		
    		for (int i=0; i < size; i++) {
    			//System.out.println("no "+i);
    			if (reducedNeibrsSet[i] != null) {
    				ALNode aln = reducedNeibrsSet[i];
    				while (aln != null) {
    					/*
    					if ((aln.getVid() == 48) && (i == 2))
    						System.out.println("estou analisando 48 e seu vizinho agora eh: 2");
    						*/
    						
    						if (!reducedNeibrsSet[aln.getVid()].checkVidExistence(i)) {
    							//System.out.println("do no "+aln.getVid()+" para o no "+i+" foi inserido aresta");
    							double dist = distance(aln.getVid(), i);
    							if (dist > maxDistance[aln.getVid()])
    								maxDistance[aln.getVid()] = dist;
    							
    							//a informacao do peso nao corresponde a nova realidade
    							ALNode node = new ALNode (i, -1, dist);
    							node.setProx(reducedNeibrsSet[aln.getVid()]);
    							reducedNeibrsSet[aln.getVid()] = node;
    							
    							double newTxPower = RadioParameters.getTxPower(dist);
    							if (newTxPower > maxTxPower[aln.getVid()]) {
    								maxTxPower[aln.getVid()] = newTxPower;
    							}
    						}
    					aln = aln.getProx();
    				}
    			}
    			//globalOptAdjl.setList(reducedNeibrsSet[i], i);
    		}
    		
    		for (int j = 0; j < size; j++) {
    			//The ReducedNeibrs list was increased and some nodes now can be reached (recursive). Probably some nodes don't know (new assymmetric links). So the algorithm can't stop
    				globalOptAdjl.setDataPower(j, maxTxPower[j]);
    				if (symmetricApproach == TCOverhearingProtocol.SYMMETRIC_APPROACH_B)
    					adjustReducedNeighbors(j, reducedNeibrsSet);
    				else {//SYMMETRIC_APPROACH_A
    					setNodesToNotAnswer(j, reducedNeibrsSet);
    				}
    				//break;
    		}
    		
    	}
    	
    	for (int i = 0; i < size; i++) {
    		ALNode alnTemp;
    		ALNode list = null;
    		for (alnTemp = reducedNeibrsSet[i]; alnTemp != null; alnTemp = alnTemp.getProx()) {
    			ALNode temp = new ALNode(alnTemp.getVid(), alnTemp.getWeight(), alnTemp.getDistance());
    			temp.setProx(list);
    			list = temp;
    		}
    		//if (list == null)
    		//	System.out.println("eh nulo");
    		globalOptAdjl.setCorrectList(list, i);
    	}
    	
    	symmetricApproach = TCOverhearingProtocol.TOTAL_SYMMETRIC_APPROACH;
    	
    	if (symmetricApproach == TCOverhearingProtocol.TOTAL_SYMMETRIC_APPROACH) {
    	//extend algorithm to implement symmetric links
    	boolean reducedNeighborhoodExtended;
    	double maxTxPower[] = new double[size];
    	
    	for (int t = 0; t < size; t++)
    		maxTxPower[t] = globalOptAdjl.getPower(t);
    	
    	do {
    		//System.out.println("passou uma vez");
    		//System.out.println("dentro do do/while");
    		for (int i=0; i < size; i++) {
    			//System.out.println("Analisando o no "+i);
    			RSNLenght[i] = NewRSNLenght[i];
    			ALNode aln = globalMaxPowerAdjl.getList(i);
    			while(aln != null) {
    				//System.out.println("verificando o vizinho " + aln.getVid());
    				double newReachDist = RadioParameters.getDistance(globalOptAdjl.getPower(aln.getVid()));
    				double dist = distance(aln.getVid(), i);
    				//System.out.println("power: "+globalOptAdjl.getPower(i)+", newReachDist: "+newReachDist+", dist: "+dist);
    				if (dist <= newReachDist) {//out of reach
    					//System.out.println("alcancavel");
    					if (!reducedNeibrsSet[i].checkVidExistence(aln.getVid())) {
    						//System.out.println(aln.getVid()+ " vai inserir o no " + i+" como vizinho");
    					//A can reach B but B can't reach A. So in this case node B must add A in the ReducedNeibrs list and increase txPower (if necessary) to be able to reach A
    						//ALNode node = new ALNode (reducedNeibrsSet[i].getVid(), reducedNeibrsSet[i].getWeight());
    						ALNode node = new ALNode (aln.getVid(), reducedNeibrsSet[aln.getVid()].getWeight(), dist);
    						node.setProx(reducedNeibrsSet[i]);
    						reducedNeibrsSet[i] = node;
    						NewRSNLenght[i]++;
    						double newTxPower = RadioParameters.getTxPower(dist);
    						if (newTxPower > maxTxPower[i]) {
    							maxTxPower[i] = newTxPower;
    						}
    					}
    				}
    				aln = aln.getProx();
    			}
    		}
    		
    		
    		reducedNeighborhoodExtended = false;
    		for (int j = 0; j < size; j++)
    			if (NewRSNLenght[j] > RSNLenght[j]) {
    			//The ReducedNeibrs list was increased and some nodes now can be reached (recursive). Probably some nodes don't know (new assymmetric links). So the algorithm can't stop
    				reducedNeighborhoodExtended = true;
    				globalOptAdjl.setPower(j, maxTxPower[j]);
    				adjustReducedNeighbors(j, reducedNeibrsSet);
    				//break;
    			}
    	} while (reducedNeighborhoodExtended);
    	//*/
    	}
    	
    	qtStatisticalNodes = 0;
    	initVars(4);
    	//now we have the new ReducedNeibrs list
    	for (int i = 0; i < size; i++) {
    		globalOptAdjl.setList(reducedNeibrsSet[i], i);
    		
    		/*
    		localAdjl = allLocalAdjl[i];
    		
    	    localAdjl.setWeight(edgeWeight, globalOptAdjl);     // set the weight of edges
    	
    	    // build local adjacent matrix:
    	    localAdjMatrix = new AdjMatrix(size);
    	    localAdjMatrix.fillMatrixFromAdjList(localAdjl);		
    		
    	    localAdjMatrix.minCostPaths();
    		
    		localAdjMatrix.calcMeanOverhearing(i, edgeWeight, globalAdjl, farthestNeighbor, hops, qtKOverhearing, qtMaxOverhearing, txConsumptionRatio);
    		
    	    if (hops[4] > 0) {
    	    	qtStatisticalNodes++;
    	    	txConsumptionRatio[2] += txConsumptionRatio[0];
    	    	txConsumptionRatio[3] += txConsumptionRatio[1];
    	    }
    	    for (int s = 0; s < 4; s++) {
    	    	hops[s+6] += hops[s];
    	    	qtKOverhearing[s+6] += qtKOverhearing[s];
    	    	if (qtMaxOverhearing[4] > 0)
    	    		qtMaxOverhearing[s+6] += qtMaxOverhearing[s];
    	    	else
    	    		qtMaxOverhearing[s+6] += 0;
    	    }
    	    
    	    if (i < size -1) {
    			localAdjMatrix.makeEmpty();
    		}
    	    */
    	}
    	
    	/*
        for (int s = 6; s < 10; s++) {
        	hops[s] = hops[s]/qtStatisticalNodes;
        	qtKOverhearing[s] = qtKOverhearing[s]/qtStatisticalNodes;
        	qtMaxOverhearing[s] = qtMaxOverhearing[s]/qtStatisticalNodes;
        }
        txConsumptionRatio[2] = txConsumptionRatio[2]/qtStatisticalNodes;
        txConsumptionRatio[3] = txConsumptionRatio[3]/qtStatisticalNodes;
        
        
    	for (int t = 0; t < 4; t++) {
    		meanKOverhearing[t+4] = qtKOverhearing[t+6];
    		meanMaxOverhearing[t+4] = qtMaxOverhearing[t+6];
    		meanHops[t+4] = hops[t+6];
    	}
    	meanTxConsumptionRatio[2] = txConsumptionRatio[2];
    	meanTxConsumptionRatio[3] = txConsumptionRatio[3];
    	*/  	
    	
    	
    	positions2Center = positions.clone();
    	Arrays.sort(positions, new PositionComparator(false));
    	Arrays.sort(positions2Center, new PositionComparator(true));
    	
    	for (int i = 0; i < size; i++) {
    		//System.out.println("para i: "+i+", index: "+positions[i].getIndex());
    		//globalOptAdjl.setOrder(positions[i].getIndex(), globalOptAdjl.getOrder(positions[i].getIndex()));
    		globalOptAdjl.setOrder(positions[i].getIndex(), i);
    		globalOptAdjl.setCenterRelatedOrder(positions2Center[i].getIndex(), i);
    		
    		globalMaxPowerAdjl.setOrder(positions[i].getIndex(), i);
    		globalMaxPowerAdjl.setCenterRelatedOrder(positions2Center[i].getIndex(), i);
    	}
    	
    	globalOptAdjl.setPt(pt);
    	globalMaxPowerAdjl.setPt(pt);
    	

    	//globalOptAdjl.setTCProtocol(nodesToNotAnswer);
    	//ALNode teste = globalOptAdjl.getCorrectList(0);
    	//if (teste == null)
    	//	System.out.println("eh nulo");
    	return true;
    }
    
	public String getName() {
		return new String("TCOverhearing");
	}    
}
