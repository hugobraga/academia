package main;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Date;
import java.util.Random;

import algorithms.Algorithm;
import algorithms.ApproximationAlgorithm;
import algorithms.SIM;
import algorithms.SPT;
import dataStructure.AdjList;
import dataStructure.PositionTable;
import dataStructure.Weight;

public class Graph {
	public static final double[] SPANNER_FACTORS = {1, 1.25, 1.5, 1.75, 2};
	public static final int FIRST_NET_SIZE = 60;
	public static final int FIRST_TERMINAL_SIZE = 10;
	public static final int INC_NET_SIZE = 40;
	public static final int INC_TERMINAL_SIZE = 10;
	public static final int NUMBER_OF_NET_SIZES = 7;
	public static final int NUMBER_OF_TERMINAL_SET_SIZES = 5;
	public static final int SOURCE_ID = 5;
	public static final int NUMBER_OF_RUNS_SIZES = 30;
	public static final int FIELD_X = 500;
	public static final int FIELD_Y = 500;
	
	
	public static int[] getNetSizes() {
		int size, sizes[] = new int[Graph.NUMBER_OF_NET_SIZES];
		size = Graph.FIRST_NET_SIZE;
		for (int i = 0; i < Graph.NUMBER_OF_NET_SIZES; i++) {
			sizes[i] = size;
			size += Graph.INC_NET_SIZE;
		}
		return sizes;
	}
	
	public static int[] getSetSizes(int qtSizes, int firstSize, int incSize) {
		int size, sizes[] = new int[qtSizes];
		size = firstSize;
		for (int i = 0; i < qtSizes; i++) {
			sizes[i] = size;
			size += incSize;
		}
		return sizes;
	}
	
	public static double getStandardDeviation(double values[], int qt, double mean) {
		int qtValues = qt;
		
		double sum = 0;
		//System.out.println("media: "+mean);
		for (int i = 0; i < qtValues; i++) {
			sum += Math.pow((values[i] - mean), 2);
			//System.out.println("valor: "+values[i]);
		}
		//System.out.println("SD: "+Math.sqrt(sum/qtValues));
		return Math.sqrt(sum/qtValues);
	}
	
    /**
     * @param args
     */
    public static void main (String[] args) {
    	Date start = new Date();
    	int netSizes[] = Graph.getSetSizes(Graph.NUMBER_OF_NET_SIZES, Graph.FIRST_NET_SIZE, Graph.INC_NET_SIZE);
		int terminalSizes[] = Graph.getSetSizes(Graph.NUMBER_OF_TERMINAL_SET_SIZES, Graph.FIRST_TERMINAL_SIZE, Graph.INC_TERMINAL_SIZE);
		String dir = "";
		
		String degreeSDFile = "grauSD.dat";
		String cvrSDFile = "cvrSD.dat";
		String pvtSDFile = "pvtSD.dat";
		String pvrSDFile = "pvrSD.dat";
		
		String degreeFile = "degree.dat";
		String cvrFile = "CVR.dat";
		String pvtFile = "PVT.dat";
		String pvrFile = "PVR.dat";
		String percCovTerFile = "percCovTer.dat";
		//String algoName = "ApproxAlgorithm-";
		String algoName = "SIM-";
		
		Date d = new Date();
		String tempFile = "tempFile-"+d.getTime()+".dat";
		Random random = new Random();    	
    	
    	
    	int sourceId = Graph.SOURCE_ID;
    	
    	int terminalsId[];// = new int[Graph.NUMBER_OF_TERMINALS];
    	double delaysTerm[];// = new double[Graph.NUMBER_OF_TERMINALS];
    	
    	double degreeSD[][][][] = new double[netSizes.length][terminalSizes.length][Graph.SPANNER_FACTORS.length][Graph.NUMBER_OF_RUNS_SIZES];
    	double cvrSD[][][][] = new double[netSizes.length][terminalSizes.length][Graph.SPANNER_FACTORS.length][Graph.NUMBER_OF_RUNS_SIZES];
    	//double pvrSD[][][][] = new double[netSizes.length][terminalSizes.length][Graph.SPANNER_FACTORS.length][Graph.NUMBER_OF_RUNS_SIZES];
    	double pvtSD[][][][] = new double[netSizes.length][terminalSizes.length][Graph.SPANNER_FACTORS.length][Graph.NUMBER_OF_RUNS_SIZES];
    	int degreeSDRunsVector[][][] = new int[netSizes.length][terminalSizes.length][Graph.SPANNER_FACTORS.length];
    	int cvrSDRunsVector[][][] = new int[netSizes.length][terminalSizes.length][Graph.SPANNER_FACTORS.length];
    	//int pvrSDRunsVector[][][] = new int[netSizes.length][terminalSizes.length][Graph.SPANNER_FACTORS.length];
    	int pvtSDRunsVector[][][] = new int[netSizes.length][terminalSizes.length][Graph.SPANNER_FACTORS.length];
    	
    	double degree[][][] = new double[netSizes.length][terminalSizes.length][Graph.SPANNER_FACTORS.length];
    	double cvr[][][] = new double[netSizes.length][terminalSizes.length][Graph.SPANNER_FACTORS.length];
    	double minOfMeanViolatedTerminalsCVR[][][] = new double[netSizes.length][terminalSizes.length][Graph.SPANNER_FACTORS.length];
    	double maxOfMeanViolatedTerminalsCVR[][][] = new double[netSizes.length][terminalSizes.length][Graph.SPANNER_FACTORS.length];
    	double maxOfMaximumViolatedTerminalsCVR[][][] = new double[netSizes.length][terminalSizes.length][Graph.SPANNER_FACTORS.length];
    	double meanOfMaximumViolatedTerminalsCVR[][][] = new double[netSizes.length][terminalSizes.length][Graph.SPANNER_FACTORS.length];
    	int cvrRunsVector[][][] = new int[netSizes.length][terminalSizes.length][Graph.SPANNER_FACTORS.length];
    	double spCosts[][][] = new double[netSizes.length][terminalSizes.length][Graph.SPANNER_FACTORS.length];
    	double terCoveredRatio[][][] = new double[netSizes.length][terminalSizes.length][Graph.SPANNER_FACTORS.length];
    	int terCoveredRatioRUNSVector[][][] = new int[netSizes.length][terminalSizes.length][Graph.SPANNER_FACTORS.length];
    	double pvt[][][] = new double[netSizes.length][terminalSizes.length][Graph.SPANNER_FACTORS.length];
    	int pvtRunsVector[][][] = new int[netSizes.length][terminalSizes.length][Graph.SPANNER_FACTORS.length];
    	double pvr[][][] = new double[netSizes.length][terminalSizes.length][Graph.SPANNER_FACTORS.length];
    	double pvrRunsVector[][][] = new double[netSizes.length][terminalSizes.length][Graph.SPANNER_FACTORS.length];
    	
    	double degreeSPT[][] = new double[netSizes.length][terminalSizes.length];
    	double costsSPT[][] = new double[netSizes.length][terminalSizes.length];
		
		try {
			
			PrintWriter degreeSDOut = new PrintWriter(new BufferedWriter(new FileWriter(dir+algoName+degreeSDFile)));
			PrintWriter cvrSDOut = new PrintWriter(new BufferedWriter(new FileWriter(dir+algoName+cvrSDFile)));
			PrintWriter pvtSDOut = new PrintWriter(new BufferedWriter(new FileWriter(dir+algoName+pvtSDFile)));
			//PrintWriter pvrSDOut = new PrintWriter(new BufferedWriter(new FileWriter(dir+algoName+pvrSDFile)));			
			
			PrintWriter degreeOut = new PrintWriter(new BufferedWriter(new FileWriter(dir+algoName+degreeFile)));
			PrintWriter cvrOut = new PrintWriter(new BufferedWriter(new FileWriter(dir+algoName+cvrFile)));
			PrintWriter pvtOut = new PrintWriter(new BufferedWriter(new FileWriter(dir+algoName+pvtFile)));
			PrintWriter pvrOut = new PrintWriter(new BufferedWriter(new FileWriter(dir+algoName+pvrFile)));
			PrintWriter percCovTerOut = new PrintWriter(new BufferedWriter(new FileWriter(dir+algoName+percCovTerFile)));			
			
			for (int i = 0; i < netSizes.length; i++) {
				for (int t = 0; t < terminalSizes.length; t++) {
					terminalsId = new int[terminalSizes[t]];
					delaysTerm = new double[terminalSizes[t]];
					int qtTerm = 0;
					ArrayList<Integer> listTerm = new ArrayList<Integer>();
					for (int id = 0; qtTerm < terminalSizes[t]; id++) {
						random.setSeed(id);
						int terId = random.nextInt(netSizes[i]);
						if ((terId != sourceId) && (!listTerm.contains(terId))) {
							terminalsId[qtTerm] = terId;
							delaysTerm[qtTerm] = 300 + random.nextInt(100);
							qtTerm++;
							listTerm.add(terId);
						}
					}
					
					for (int p = 0; p < Graph.SPANNER_FACTORS.length; p++) {
						degree[i][t][p] = 0;
						cvr[i][t][p] = 0;
						minOfMeanViolatedTerminalsCVR[i][t][p] = 0;
						maxOfMeanViolatedTerminalsCVR[i][t][p] = 0;
						maxOfMaximumViolatedTerminalsCVR[i][t][p] = 0;
						meanOfMaximumViolatedTerminalsCVR[i][t][p] = 0;
						spCosts[i][t][p] = 0;
						terCoveredRatio[i][t][p] = 0;
						pvt[i][t][p] = 0;
						pvr[i][t][p] = 0;
						
						pvrRunsVector[i][t][p] = 0;
						cvrRunsVector[i][t][p] = 0;
						pvtRunsVector[i][t][p] = 0;
						terCoveredRatioRUNSVector[i][t][p] = 0;
						
						//pvrSDRunsVector[i][t][p] = 0;
						//cvrSDRunsVector[i][t][p] = 0;
						//pvtSDRunsVector[i][t][p] = 0;						
					}
					
					degreeSPT[i][t] = 0;
					costsSPT[i][t] = 0;					
					
				int seed = 0;
				for (int j = 0; j < Graph.NUMBER_OF_RUNS_SIZES; j++, seed++) {
					System.out.println("size: "+netSizes[i]+", ter: "+terminalSizes[t]+", run: "+j);
					random.setSeed(seed);
					///*
					PrintWriter topo = null;

							topo = new PrintWriter(new BufferedWriter(new FileWriter(dir+tempFile)));
						for (int a = 0; a < netSizes[i]; a++) {
							double x = random.nextDouble()*Graph.FIELD_X;
							topo.println(a+" set X_ "+x);
							double y = random.nextDouble()*Graph.FIELD_Y;
							topo.println(a+" set Y_ "+y);
							topo.println("--");
							topo.println("--");
							topo.println();
						}
						topo.close();

												
						PositionTable pt = new PositionTable(netSizes[i]);
						pt.readFromTopologyFile(dir+tempFile);
						Weight w = new Weight (50, 50, 0.1, 2, pt);
						//Algorithm algo = new ApproximationAlgorithm(netSizes[i], 125.6, sourceId, terminalsId, delaysTerm, pt, w);
						Algorithm algo = new SIM(netSizes[i], 125.6, sourceId, terminalsId, delaysTerm, pt, w);
						if (algo.pseudoConstructor()) {
							
							for (int p = 0; p < Graph.SPANNER_FACTORS.length; p++) {
								
								algo.execute(Graph.SPANNER_FACTORS[p]);
								
								double minCVR[] = new double[2];
								double maxCVR[] = new double[2];
								
								double val = algo.getArbMaxOutDegree();								
								degree[i][t][p] += val;
								degreeSD[i][t][p][j] = val;
								
								val = algo.getViolatedTerminalsCVR(minCVR, maxCVR);
								if (val > 0) {
									cvr[i][t][p] += val;
									cvrSD[i][t][p][cvrRunsVector[i][t][p]] = val;
									cvrRunsVector[i][t][p]++;

									if (val > maxOfMeanViolatedTerminalsCVR[i][t][p])
										maxOfMeanViolatedTerminalsCVR[i][t][p] = val;
									if ((val < minOfMeanViolatedTerminalsCVR[i][t][p]) || (minOfMeanViolatedTerminalsCVR[i][t][p] == 0))
										minOfMeanViolatedTerminalsCVR[i][t][p] = val;

									if (maxCVR[0] > maxOfMaximumViolatedTerminalsCVR[i][t][p])
										maxOfMaximumViolatedTerminalsCVR[i][t][p] = maxCVR[0];
									meanOfMaximumViolatedTerminalsCVR[i][t][p] += maxCVR[0];
									
								}
								
								if (val > 0) {
									pvrRunsVector[i][t][p]++;
								}
								
								val = algo.getTermCoverFstPhRatio();
								if (val > 0) {
									terCoveredRatio[i][t][p] += val;
									terCoveredRatioRUNSVector[i][t][p]++;
								}
								
								val = algo.getViolatedNodesRatio();
								if (val > 0) {
									pvt[i][t][p] += val;
									pvtSD[i][t][p][pvtRunsVector[i][t][p]] = val*100;
									pvtRunsVector[i][t][p]++;
								}
							}						
						} else
							j--;
				}
				
				for (int p = 0; p < Graph.SPANNER_FACTORS.length; p++) {
					degree[i][t][p] = degree[i][t][p]/Graph.NUMBER_OF_RUNS_SIZES;
					degreeSD[i][t][p][0] = Graph.getStandardDeviation(degreeSD[i][t][p], Graph.NUMBER_OF_RUNS_SIZES, degree[i][t][p]);
					
					if (cvrRunsVector[i][t][p] == 0) {
						cvr[i][t][p] = 0;
						meanOfMaximumViolatedTerminalsCVR[i][t][p] = 0;
						cvrSD[i][t][p][0] = -1;
					} else {
						//cvr[i][t][p] = (cvr[i][t][p]+(Graph.NUMBER_OF_RUNS_SIZES-cvrRunsVector[i][t][p]))/Graph.NUMBER_OF_RUNS_SIZES;
						cvr[i][t][p] = (cvr[i][t][p]/cvrRunsVector[i][t][p]);
						meanOfMaximumViolatedTerminalsCVR[i][t][p] = (meanOfMaximumViolatedTerminalsCVR[i][t][p]+(Graph.NUMBER_OF_RUNS_SIZES-cvrRunsVector[i][t][p]))/Graph.NUMBER_OF_RUNS_SIZES;
						//System.out.println("-----vai pegar o SD de CVR----");
						cvrSD[i][t][p][0] = Graph.getStandardDeviation(cvrSD[i][t][p], cvrRunsVector[i][t][p], cvr[i][t][p]);
						//System.out.println("-----pegou-----");
					}

					if (terCoveredRatioRUNSVector[i][t][p] == 0)
						terCoveredRatio[i][t][p] = 0;
					else
						terCoveredRatio[i][t][p] = (terCoveredRatio[i][t][p]/terCoveredRatioRUNSVector[i][t][p])*100;
					if (pvtRunsVector[i][t][p] == 0) {
						pvt[i][t][p] = 0;
						pvtSD[i][t][p][0] = -1;
					} else {
						//pvt[i][t][p] = (pvt[i][t][p]/Graph.NUMBER_OF_RUNS_SIZES)*100;
						pvt[i][t][p] = (pvt[i][t][p]/pvtRunsVector[i][t][p])*100;
						pvtSD[i][t][p][0] = Graph.getStandardDeviation(pvtSD[i][t][p], pvtRunsVector[i][t][p], pvt[i][t][p]);
					}
					
					pvr[i][t][p] = (pvrRunsVector[i][t][p]/Graph.NUMBER_OF_RUNS_SIZES)*100;
					
					degreeOut.println(netSizes[i]+" "+terminalSizes[t]+" "+Graph.SPANNER_FACTORS[p]+" "+degree[i][t][p]);
					cvrOut.println(netSizes[i]+" "+terminalSizes[t]+" "+Graph.SPANNER_FACTORS[p]+" "+cvr[i][t][p]+" "+maxOfMeanViolatedTerminalsCVR[i][t][p]+" "+maxOfMaximumViolatedTerminalsCVR[i][t][p]+" "+meanOfMaximumViolatedTerminalsCVR[i][t][p]);
					//percCovTerOut.println(netSizes[i]+" "+terminalSizes[t]+" "+Graph.SPANNER_FACTORS[p]+" "+terCoveredRatio[i][t][p]);
					pvtOut.println(netSizes[i]+" "+terminalSizes[t]+" "+Graph.SPANNER_FACTORS[p]+" "+pvt[i][t][p]);
					pvrOut.println(netSizes[i]+" "+terminalSizes[t]+" "+Graph.SPANNER_FACTORS[p]+" "+pvr[i][t][p]);
					
					degreeSDOut.println(netSizes[i]+" "+terminalSizes[t]+" "+Graph.SPANNER_FACTORS[p]+" "+degreeSD[i][t][p]);
					cvrSDOut.println(netSizes[i]+" "+terminalSizes[t]+" "+Graph.SPANNER_FACTORS[p]+" "+cvrSD[i][t][p]);
					pvtSDOut.println(netSizes[i]+" "+terminalSizes[t]+" "+Graph.SPANNER_FACTORS[p]+" "+pvtSD[i][t][p]);
					
					
					System.out.println(netSizes[i]+" "+terminalSizes[t]+" "+Graph.SPANNER_FACTORS[p]+" "+degree[i][t][p]);
					System.out.println(netSizes[i]+" "+terminalSizes[t]+" "+Graph.SPANNER_FACTORS[p]+" "+cvr[i][t][p]);
					//System.out.println(netSizes[i]+" "+terminalSizes[t]+" "+Graph.SPANNER_FACTORS[p]+" "+terCoveredRatio[i][t][p]); -> unecessary					
					System.out.println(netSizes[i]+" "+terminalSizes[t]+" "+Graph.SPANNER_FACTORS[p]+" "+pvt[i][t][p]);
					System.out.println(netSizes[i]+" "+terminalSizes[t]+" "+Graph.SPANNER_FACTORS[p]+" "+pvr[i][t][p]);
					System.out.println();
					
					System.out.println(netSizes[i]+" "+terminalSizes[t]+" "+Graph.SPANNER_FACTORS[p]+" "+degreeSD[i][t][p][0]);
					System.out.println(netSizes[i]+" "+terminalSizes[t]+" "+Graph.SPANNER_FACTORS[p]+" "+cvrSD[i][t][p][0]);					
					System.out.println(netSizes[i]+" "+terminalSizes[t]+" "+Graph.SPANNER_FACTORS[p]+" "+pvtSD[i][t][p][0]);
					System.out.println();
				}
				
				degreeOut.println();
				cvrOut.println();
				percCovTerOut.println();
				pvtOut.println();
				pvrOut.println();
				
				degreeSDOut.println();
				cvrSDOut.println();
				pvtSDOut.println();				
			}
			}
			
			degreeOut.close();
			cvrOut.close();
			percCovTerOut.close();
			pvtOut.close();
			pvrOut.close();
			
			degreeSDOut.close();
			cvrSDOut.close();
			pvtSDOut.close();		
			
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			
			for (int p = 0; p < Graph.SPANNER_FACTORS.length; p++) {
				System.out.println("Spanning factor of "+Graph.SPANNER_FACTORS[p]);
				for (int i = 0; i < netSizes.length; i++)
					for (int j = 0; j < terminalSizes.length; j++) {
						System.out.println("size "+netSizes[i]+" ter "+terminalSizes[j]+" degree "+degree[i][j][p]);
					}
				
				for (int i = 0; i < netSizes.length; i++)
					for (int j = 0; j < terminalSizes.length; j++) {
						System.out.println("size "+netSizes[i]+" ter "+terminalSizes[j]+" violatedTerminalsCVR "+cvr[i][j][p]);
					}					
				
				for (int i = 0; i < netSizes.length; i++)
					for (int j = 0; j < terminalSizes.length; j++) {
						System.out.println("size "+netSizes[i]+" ter "+terminalSizes[j]+" terCoveredRatio "+terCoveredRatio[i][j][p]);
					}
				
				for (int i = 0; i < netSizes.length; i++)
					for (int j = 0; j < terminalSizes.length; j++) {
						System.out.println("size "+netSizes[i]+" ter "+terminalSizes[j]+" violatedNodesRatio "+pvt[i][j][p]);
					}			
				
				for (int i = 0; i < netSizes.length; i++)
					for (int j = 0; j < terminalSizes.length; j++) {
						System.out.println("tam "+netSizes[i]+" ter "+terminalSizes[j]+" violatedRunsRatio "+pvr[i][j][p]);
					}				
			}
		
			Date stop = new Date();
			long l1 = start.getTime();
			long l2 = stop.getTime();
			long diff = l2 - l1;

			long secondInMillis = 1000;
			long minuteInMillis = secondInMillis * 60;
			long hourInMillis = minuteInMillis * 60;
			long dayInMillis = hourInMillis * 24;
			long yearInMillis = dayInMillis * 365;

			long elapsedYears = diff / yearInMillis;
			diff = diff % yearInMillis;
			long elapsedDays = diff / dayInMillis;
			diff = diff % dayInMillis;
			long elapsedHours = diff / hourInMillis;
			diff = diff % hourInMillis;
			long elapsedMinutes = diff / minuteInMillis;
			diff = diff % minuteInMillis;
			long elapsedSeconds = diff / secondInMillis;
			
			System.out.println(elapsedHours+"h:"+elapsedMinutes+"m:"+elapsedSeconds+"s");


    }
}
