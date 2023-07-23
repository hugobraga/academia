package main;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Date;
import java.util.Random;

import dataStructure.PositionTable;
import dataStructure.Weight;

import radios.RadioParameters;
import tcAlgorithm.APIProtocol;
import tcAlgorithm.GGProtocol;
import tcAlgorithm.IRNGProtocol;
import tcAlgorithm.LifeProtocol;
import tcAlgorithm.RNGProtocol;
import tcAlgorithm.SimpleTCProtocol;
import tcAlgorithm.TCOverhearingProtocol;
import tcAlgorithm.TCProtocol;
import tcAlgorithm.XTCProtocol;

public class RunTCAlgorithm {
	public static final int FIELD_X = 200;
	public static final int FIELD_Y = 200;
	
	public static final int FIRST_NET_SIZE = 300;
	//public static final int FIRST_NET_SIZE = 150;
	public static final int INC_NET_SIZE = 30;
	public static final int NUMBER_OF_NET_SIZES = 1;
	//public static final int NUMBER_OF_NET_SIZES = 1;
	public static final int NUMBER_OF_RUNS_SIZES = 30;	

	public static int[] getNetSizes() {
		int size, sizes[] = new int[RunTCAlgorithm.NUMBER_OF_NET_SIZES];
		size = RunTCAlgorithm.FIRST_NET_SIZE;
		for (int i = 0; i < RunTCAlgorithm.NUMBER_OF_NET_SIZES; i++) {
			sizes[i] = size;
			size += RunTCAlgorithm.INC_NET_SIZE;
		}
		return sizes;
	}
	
	public static void concatFile(PrintWriter out, String file) {
		try {
			BufferedReader in = new BufferedReader(new FileReader(file));
			String line;
			while ((line = in.readLine()) != null) {
				out.println(line);
			}
			in.close();
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}	
	
	public static void getSimpleTCProtocolData(int sizes[], String dir, String outFile, String tempFile, String radioName, Random random) {
		try {
			double fator[] = new double[sizes.length];
			double maxFator[] = new double[sizes.length];
			
			//boolean flag = false;			
			PrintWriter out = new PrintWriter(new BufferedWriter(new FileWriter(dir+outFile)));	
			//PrintWriter outMax = new PrintWriter(new BufferedWriter(new FileWriter(dir+outMaxFile)));
			
			TCProtocol tcProtocol = null;
			
			for (int i = 0; i < sizes.length; i++) {
				
				fator[i] = 0;
				maxFator[i] = 0;
								//if (flag) {
				out.println(sizes[i]);
				out.println("SN.numNodes="+sizes[i]);
				//}
				
				//outMax.println(sizes[i]);
				//outMax.println("SN.numNodes="+sizes[i]);
				//int faultRuns = 0;
				int seed = 0;
				for (int j = 0; j < RunTCAlgorithm.NUMBER_OF_RUNS_SIZES; j++, seed++) {
					System.out.println("tam: "+sizes[i]+", run: "+j);
					random.setSeed(seed);
					///*
					PrintWriter topo = null;
						//if (flag)
							topo = new PrintWriter(new BufferedWriter(new FileWriter(dir+tempFile)));
						for (int a = 0; a < sizes[i]; a++) {
							double x = random.nextDouble()*RunTCAlgorithm.FIELD_X;
							topo.println(a+" set X_ "+x);
							double y = random.nextDouble()*RunTCAlgorithm.FIELD_Y;
							topo.println(a+" set Y_ "+y);
							topo.println("--");
							topo.println("--");
							topo.println();
						}
						topo.close();
						//*/
						
						PositionTable pt = new PositionTable(sizes[i]);
						pt.readFromTopologyFile(dir+tempFile);
						pt.print();
						Weight w = new Weight (RadioParameters.elecTx, RadioParameters.elecRx, RadioParameters.epsilon, RadioParameters.npathLossExp, pt);
						
						//here we choose the simple TC protocol
						tcProtocol = new XTCProtocol(sizes[i], pt, RadioParameters.MAXIMUM_DISTANCE_REACHED, w);
						
						if (tcProtocol.execute()) {
							fator[i] += tcProtocol.getPICS();
							maxFator[i] += tcProtocol.getMaxPICS();
						} else {
							j--;
						}
				}				
				fator[i] = fator[i]/(RunTCAlgorithm.NUMBER_OF_RUNS_SIZES);
				maxFator[i] = maxFator[i]/(RunTCAlgorithm.NUMBER_OF_RUNS_SIZES);
				System.out.println("Em "+i+" o fator eh: "+fator[i]+" e o maxFator eh: "+maxFator[i]);
			}
			
			String fatorfileStr = radioName+"-"+tcProtocol.getName()+"-fator.info";
			String maxFatorfileStr = radioName+"-"+tcProtocol.getName()+"-maxFator.info";			
						
			PrintWriter fatorOut = new PrintWriter(new BufferedWriter(new FileWriter(dir+fatorfileStr)));
			PrintWriter maxFatorOut = new PrintWriter(new BufferedWriter(new FileWriter(dir+maxFatorfileStr)));
			
			for (int t = 0; t < sizes.length; t++) {
				fatorOut.println(sizes[t]+" "+fator[t]);
				maxFatorOut.println(sizes[t]+" "+maxFator[t]);
			}						
			fatorOut.close();
			maxFatorOut.close();
			
			out.close();
			//outMax.close();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	public static void getTCOverhearingData(int sizes[], String dir, String outFile, String tempFile, String radioName, Random random) {
		try {
			double fator[] = new double[sizes.length];
			double maxFator[] = new double[sizes.length];
			
			double diffTxLevels[] = new double[sizes.length];
			double maxDiffTxLevels[] = new double[sizes.length];			
			
			//boolean flag = false;
			double perMultihop[] = new double[sizes.length];
			double perInequation[] = new double[sizes.length];
			double fracK[] = new double[sizes.length];
			double totalHops[] = new double[sizes.length];
			double ratio[] = new double[sizes.length];
			double percRecp[] = new double[sizes.length];
			double distances[] = new double[sizes.length];
			double totalDistances[] = new double[sizes.length];
			double apartDistances[] = new double[sizes.length];
			double totalDistancesAB[] = new double[sizes.length];
			
			double totalEpsilonPart[] = new double[sizes.length];
			double totalElecTxPart[] = new double[sizes.length];
			double totalElecRxPart[] = new double[sizes.length];
			
			double maxHopsRatio[] = new double[sizes.length];
			double meanHopsRatio[] = new double[sizes.length];
			
			double minReachedDist[] = new double[sizes.length];			
			double maxReachedDist[] = new double[sizes.length];
			double meanReachedDist[] = new double[sizes.length];
			
			double meanTxLevels[] = new double[sizes.length];
			
			double hops[] = new double[sizes.length];
			double allHops[] = new double[sizes.length];
			double kOverhearing[] = new double[sizes.length];
			double kRealOverhearing[] = new double[sizes.length];
			double allKOverhearing[] = new double[sizes.length];
			double allKRealOverhearing[] = new double[sizes.length];			
			double maxOverhearing[] = new double[sizes.length];
			double maxRealOverhearing[] = new double[sizes.length];
			double allMaxOverhearing[] = new double[sizes.length];
			double allMaxRealOverhearing[] = new double[sizes.length];
			double txConsumptionRatio[] = new double[sizes.length];
			double allTxConsumptionRatio[] = new double[sizes.length];
			
			double afterHops[] = new double[sizes.length];
			double afterAllHops[] = new double[sizes.length];
			double afterKOverhearing[] = new double[sizes.length];
			double afterKRealOverhearing[] = new double[sizes.length];
			double afterAllKOverhearing[] = new double[sizes.length];
			double afterAllKRealOverhearing[] = new double[sizes.length];			
			double afterMaxOverhearing[] = new double[sizes.length];
			double afterMaxRealOverhearing[] = new double[sizes.length];
			double afterAllMaxOverhearing[] = new double[sizes.length];
			double afterAllMaxRealOverhearing[] = new double[sizes.length];
			double afterTxConsumptionRatio[] = new double[sizes.length];
			double afterAllTxConsumptionRatio[] = new double[sizes.length];			
			
			PrintWriter out = new PrintWriter(new BufferedWriter(new FileWriter(dir+outFile)));	
			//PrintWriter outMax = new PrintWriter(new BufferedWriter(new FileWriter(dir+outMaxFile)));
			
			String fracKfileStr = radioName+"-fracK.info";
			String ratiofileStr = radioName+"-ratio.info";
			
			String perMultihopfileStr = radioName+"-perMultihop.info";
			String perInequationfileStr = radioName+"-perInequation.info";			
			String totalHopsfileStr = radioName+"-totalHops.info";
			String percRecpfileStr = radioName+"-percRecp.info";
			String distancesfileStr = radioName+"-distances.info";
			String totalDistancesfileStr = radioName+"-totalDistances.info";
			String apartDistancesfileStr = radioName+"-apartDistances.info";
			String totalDistancesABfileStr = radioName+"-totalDistancesAB.info";
			
			String totalEpsilonPartfileStr = radioName+"-totalEpsilonPart.info";
			String totalElecTxPartfileStr = radioName+"-totalElecTxPart.info";
			String totalElecRxPartfileStr = radioName+"-totalElecRxPart.info";
			
			String maxHopsRatiofileStr = radioName+"-maxHopsRatio.info";
			String meanHopsRatiofileStr = radioName+"-meanHopsRatio.info";
			
			String minReachedDistfileStr = radioName+"-minReachedDist.info";			
			String maxReachedDistfileStr = radioName+"-maxReachedDist.info";
			String meanReachedDistfileStr = radioName+"-minReachedDist.info";
			
			String meanTxLevelsfileStr = radioName+"-meanTxLevels.info";
			
			String allKRealOverfileStr = radioName+"all-real-k-OverMean.info";
			String allMaxRealOverfileStr = radioName+"all-real-max-OverMean.info";
			
			/*
			String hopsfileStr = radioName+"hopsMean.info";
			String allHopsfileStr = radioName+"all-hopsMean.info";
			String kOverfileStr = radioName+"k-OverMean.info";
			String kRealOverfileStr = radioName+"real-k-OverMean.info";
			String allKOverfileStr = radioName+"all-k-OverMean.info";
			String allKRealOverfileStr = radioName+"all-real-k-OverMean.info";
			String maxOverfileStr = radioName+"max-OverMean.info";
			String maxRealOverfileStr = radioName+"real-max-OverMean.info";
			String allMaxOverfileStr = radioName+"all-max-OverMean.info";
			String allMaxRealOverfileStr = radioName+"all-real-max-OverMean.info";
			String txConsumptionRatiofileStr = radioName+"txConsumptionRatio.info";
			String allTxConsumptionRatiofileStr = radioName+"all-txConsumptionRatio.info";
			
			String afterHopsfileStr = radioName+"after-hopsMean.info";
			String afterAllHopsfileStr = radioName+"after-all-hopsMean.info";
			String afterKOverfileStr = radioName+"after-k-OverMean.info";
			String afterKRealOverfileStr = radioName+"after-real-k-OverMean.info";
			String afterAllKOverfileStr = radioName+"after-all-k-OverMean.info";
			String afterAllKRealOverfileStr = radioName+"after-all-real-k-OverMean.info";
			String afterMaxOverfileStr = radioName+"after-max-OverMean.info";
			String afterMaxRealOverfileStr = radioName+"after-real-max-OverMean.info";
			String afterAllMaxOverfileStr = radioName+"after-all-max-OverMean.info";
			String afterAllMaxRealOverfileStr = radioName+"after-all-real-max-OverMean.info";
			String afterTxConsumptionRatiofileStr = radioName+"after-txConsumptionRatio.info";
			String afterAllTxConsumptionRatiofileStr = radioName+"after-all-txConsumptionRatio.info";
			*/			
			
			TCProtocol tcProtocol = null;			
			
			for (int i = 0; i < sizes.length; i++) {
				/*
				fracK[i] = 0;
				ratio[i] = 0;
				*/
				perMultihop[i] = 0;
				perInequation[i] = 0;				
				totalHops[i] = 0;
				percRecp[i] = 0;
				fator[i] = 0;
				maxFator[i] = 0;
				distances[i] = 0;
				totalDistances[i] = 0;
				apartDistances[i] = 0;
				totalDistancesAB[i] = 0;
				
				totalEpsilonPart[i] = 0;
				totalElecTxPart[i] = 0;
				totalElecRxPart[i] = 0;
				
				maxHopsRatio[i] = 0;
				meanHopsRatio[i] = 0;
				
				minReachedDist[i] = 0;				
				maxReachedDist[i] = 0;
				meanReachedDist[i] = 0;
				
				meanTxLevels[i] = 0;
				
				diffTxLevels[i] = 0;
				maxDiffTxLevels[i] = 0;				
				
				allKRealOverhearing[i] = 0;
				allMaxRealOverhearing[i] = 0;
				
				/*
				hops[i] = 0;
				allHops[i] = 0;
				kOverhearing[i] = 0;
				kRealOverhearing[i] = 0;
				maxOverhearing[i] = 0;
				maxRealOverhearing[i] = 0;
				allKOverhearing[i] = 0;
				allKRealOverhearing[i] = 0;
				allMaxOverhearing[i] = 0;
				allMaxRealOverhearing[i] = 0;
				txConsumptionRatio[i] = 0;
				allTxConsumptionRatio[i] = 0;
				
				afterHops[i] = 0;
				afterAllHops[i] = 0;
				afterKOverhearing[i] = 0;
				afterKRealOverhearing[i] = 0;
				afterMaxOverhearing[i] = 0;
				afterMaxRealOverhearing[i] = 0;
				afterAllKOverhearing[i] = 0;
				afterAllKRealOverhearing[i] = 0;
				afterAllMaxOverhearing[i] = 0;
				afterAllMaxRealOverhearing[i] = 0;
				afterTxConsumptionRatio[i] = 0;
				afterAllTxConsumptionRatio[i] = 0;
				*/				
				//if (flag) {
				out.println(sizes[i]);
				out.println("SN.numNodes="+sizes[i]);
				//}
				
				//outMax.println(sizes[i]);
				//outMax.println("SN.numNodes="+sizes[i]);
				
				
				int seed = 0;
				for (int j = 0; j < RunTCAlgorithm.NUMBER_OF_RUNS_SIZES; j++, seed++) {
					System.out.println("tam: "+sizes[i]+", run: "+j);
					random.setSeed(seed);
					///*
					PrintWriter topo = null;
						//if (flag)
							topo = new PrintWriter(new BufferedWriter(new FileWriter(dir+tempFile)));
						for (int a = 0; a < sizes[i]; a++) {
							double x = random.nextDouble()*RunTCAlgorithm.FIELD_X;
							topo.println(a+" set X_ "+x);
							double y = random.nextDouble()*RunTCAlgorithm.FIELD_Y;
							//if ((i < GerarPosicaoPotencia.NUMBER_OF_RUNS_SIZES) && (sizes[i] <= 300) & (j < 18))
							//	continue;
							//flag = true;
							topo.println(a+" set Y_ "+y);
							topo.println("--");
							topo.println("--");
							topo.println();
						}
						topo.close();
						//*/
						
						//if (flag) {
						
						PositionTable pt = new PositionTable(sizes[i]);
						pt.readFromTopologyFile(dir+tempFile);
						pt.print();
						Weight w = new Weight (RadioParameters.elecTx, RadioParameters.elecRx, RadioParameters.epsilon, RadioParameters.npathLossExp, pt);
						tcProtocol = new TCOverhearingProtocol (sizes[i], pt, RadioParameters.MAXIMUM_DISTANCE_REACHED, w);
						if (tcProtocol.execute()) {
							//op.LifeOutput();
							/*
							fracK[i] += temp[2];
							ratio[i] += temp[4];
							*/
							
							double temp[] = new double[19];
							TCOverhearingProtocol tcOverhearing = (TCOverhearingProtocol)tcProtocol; 
							temp = tcOverhearing.getPercentMultihop();
							perMultihop[i] += temp[0];
							perInequation[i] += temp[1];							
							totalHops[i] += temp[3];
							percRecp[i] += temp[5];
							fator[i] += tcProtocol.getPICS();
							maxFator[i] += tcProtocol.getMaxPICS();
							System.out.println("Em "+sizes[i]+", run: "+j+", fator: "+tcProtocol.getPICS()+" maxFator: "+tcProtocol.getMaxPICS());
							distances[i] += temp[6];
							totalDistances[i] += temp[7];
							apartDistances[i] += temp[8];
							totalDistancesAB[i] += temp[9];
							
							totalEpsilonPart[i] += temp[10];
							totalElecTxPart[i] += temp[11];
							totalElecRxPart[i] += temp[12];
							
				    		maxHopsRatio[i] += temp[13];
				    		//meanHopsRatio[i] += temp[14];
				    		
				    		minReachedDist[i] += temp[15];
				    		maxReachedDist[i] += temp[16];
				    		meanReachedDist[i] += temp[17];
				    		
				    		meanTxLevels[i] += temp[18];
							
							double kOverInfo[] = new double[4];
							kOverInfo = tcOverhearing.getMeanQtKOverhearing();
							allKRealOverhearing[i] += kOverInfo[3];
							
							double maxOverInfo[] = new double[4];
							maxOverInfo = tcOverhearing.getMeanQtMaxOverhearing();
							allMaxRealOverhearing[i] += maxOverInfo[3];							
							
							diffTxLevels[i] += tcProtocol.getDiffTxLevels();
							maxDiffTxLevels[i] += tcProtocol.getMaxDiffTxLevels();							
						} else
							j--;

				}
				/*
				fracK[i] = fracK[i]/GerarPosicaoPotencia.NUMBER_OF_RUNS_SIZES;
				ratio[i] = ratio[i]/GerarPosicaoPotencia.NUMBER_OF_RUNS_SIZES;
				*/
				
				perMultihop[i] = perMultihop[i]/RunTCAlgorithm.NUMBER_OF_RUNS_SIZES;
				perInequation[i] = perInequation[i]/RunTCAlgorithm.NUMBER_OF_RUNS_SIZES;				
				totalHops[i] = totalHops[i]/RunTCAlgorithm.NUMBER_OF_RUNS_SIZES;
				percRecp[i] = percRecp[i]/RunTCAlgorithm.NUMBER_OF_RUNS_SIZES;
				fator[i] = fator[i]/(RunTCAlgorithm.NUMBER_OF_RUNS_SIZES);	
				maxFator[i] = maxFator[i]/(RunTCAlgorithm.NUMBER_OF_RUNS_SIZES);
				distances[i] = distances[i]/RunTCAlgorithm.NUMBER_OF_RUNS_SIZES;
				totalDistances[i] = totalDistances[i]/RunTCAlgorithm.NUMBER_OF_RUNS_SIZES;
				apartDistances[i] = apartDistances[i]/RunTCAlgorithm.NUMBER_OF_RUNS_SIZES;
				totalDistancesAB[i] = totalDistancesAB[i]/RunTCAlgorithm.NUMBER_OF_RUNS_SIZES;
				
				totalEpsilonPart[i] = totalEpsilonPart[i]/RunTCAlgorithm.NUMBER_OF_RUNS_SIZES;
				totalElecTxPart[i] = totalElecTxPart[i]/RunTCAlgorithm.NUMBER_OF_RUNS_SIZES;
				totalElecRxPart[i] = totalElecRxPart[i]/RunTCAlgorithm.NUMBER_OF_RUNS_SIZES;
				
				maxHopsRatio[i] = maxHopsRatio[i]/RunTCAlgorithm.NUMBER_OF_RUNS_SIZES;
				//meanHopsRatio[i] = meanHopsRatio[i]/RunTCAlgorithm.NUMBER_OF_RUNS_SIZES;
				
				minReachedDist[i] = minReachedDist[i]/RunTCAlgorithm.NUMBER_OF_RUNS_SIZES;				
				maxReachedDist[i] = maxReachedDist[i]/RunTCAlgorithm.NUMBER_OF_RUNS_SIZES;
				meanReachedDist[i] = meanReachedDist[i]/RunTCAlgorithm.NUMBER_OF_RUNS_SIZES;
				
				meanTxLevels[i] = meanTxLevels[i]/RunTCAlgorithm.NUMBER_OF_RUNS_SIZES;
				
				allKRealOverhearing[i] = allKRealOverhearing[i]/(RunTCAlgorithm.NUMBER_OF_RUNS_SIZES);	
				allMaxRealOverhearing[i] = allMaxRealOverhearing[i]/(RunTCAlgorithm.NUMBER_OF_RUNS_SIZES);				

				System.out.println("Em "+sizes[i]+", fator: "+fator[i]+" maxFator: "+maxFator[i]);
				//System.out.println(RadioParameters.radioName+" "+sizes[i]+" "+RadioParameters.elecTx+" "+RadioParameters.epsilon+" "+RadioParameters.elecRx+" "+totalEpsilonPart[i]+" "+totalElecTxPart[i]+" "+totalElecRxPart[i]+" "+fator[i]+" "+maxFator[i]+" "+totalHops[i]+" "+maxHopsRatio[i]+" "+perMultihop[i]+" "+perInequation[i]+" "+distances[i]+" "+totalDistances[i]+" "+apartDistances[i]+" "+totalDistancesAB[i]+" "+allMaxRealOverhearing[i]+" "+allKRealOverhearing[i]+" "+minReachedDist[i]+" "+maxReachedDist[i]+" "+meanReachedDist[i]+" "+meanTxLevels[i]);
				//System.out.println("Radio: "+RadioParameters.radioName+" - Em "+sizes[i]+", fator: "+fator[i]+" maxFator: "+maxFator[i]+" totalHops: "+totalHops[i]+" perMultihop: "+perMultihop[i]+" perInequation: "+perInequation[i]+" distance: "+distances[i]+" totalDistance: "+totalDistances[i]+" apartDistance: "+apartDistances[i]+" totalDistanceAB: "+totalDistancesAB[i]+" totalEpsilonPart: "+totalEpsilonPart[i]+" totalElecTxPart: "+totalElecTxPart[i]+" totalElecRxPart: "+totalElecRxPart[i]);
				//System.out.println("Em "+sizes[i]+", fator: "+fator[i]+" maxFator: "+maxFator[i]+" diffTxLevels: "+diffTxLevels[i]+" maxDiffTxLevels: "+maxDiffTxLevels[i]);
				//System.out.println(fator[i]+" "+maxFator[i]+" "+totalHops[i]+" "+perMultihop[i]+" "+perInequation[i]);
			}
			/*
			PrintWriter fracKOut = new PrintWriter(new BufferedWriter(new FileWriter(dir+fracKfileStr)));
			PrintWriter ratioOut = new PrintWriter(new BufferedWriter(new FileWriter(dir+ratiofileStr)));
			*/
			
			String fatorfileStr = radioName+"-"+tcProtocol.getName()+"-fator.info";
			String maxFatorfileStr = radioName+"-"+tcProtocol.getName()+"-maxFator.info";			
			
			PrintWriter perMultihopOut = new PrintWriter(new BufferedWriter(new FileWriter(dir+perMultihopfileStr)));
			PrintWriter perInequationOut = new PrintWriter(new BufferedWriter(new FileWriter(dir+perInequationfileStr)));			
			PrintWriter totalHopsOut = new PrintWriter(new BufferedWriter(new FileWriter(dir+totalHopsfileStr)));
			PrintWriter percRecpOut = new PrintWriter(new BufferedWriter(new FileWriter(dir+percRecpfileStr)));
			PrintWriter fatorOut = new PrintWriter(new BufferedWriter(new FileWriter(dir+fatorfileStr)));
			PrintWriter maxFatorOut = new PrintWriter(new BufferedWriter(new FileWriter(dir+maxFatorfileStr)));
			PrintWriter distancesOut = new PrintWriter(new BufferedWriter(new FileWriter(dir+distancesfileStr)));
			PrintWriter totalDistancesOut = new PrintWriter(new BufferedWriter(new FileWriter(dir+totalDistancesfileStr)));
			PrintWriter apartDistancesOut = new PrintWriter(new BufferedWriter(new FileWriter(dir+apartDistancesfileStr)));
			PrintWriter totalDistancesABOut = new PrintWriter(new BufferedWriter(new FileWriter(dir+totalDistancesABfileStr)));
			PrintWriter totalEpsilonPartOut = new PrintWriter(new BufferedWriter(new FileWriter(dir+totalEpsilonPartfileStr)));
			PrintWriter totalElecTxPartOut = new PrintWriter(new BufferedWriter(new FileWriter(dir+totalElecTxPartfileStr)));
			PrintWriter totalElecRxPartOut = new PrintWriter(new BufferedWriter(new FileWriter(dir+totalElecRxPartfileStr)));
			
			PrintWriter maxHopsRatioOut = new PrintWriter(new BufferedWriter(new FileWriter(dir+maxHopsRatiofileStr)));
			//PrintWriter meanHopsRatioOut = new PrintWriter(new BufferedWriter(new FileWriter(dir+meanHopsRatiofileStr)));
			
			PrintWriter minReachedDistOut = new PrintWriter(new BufferedWriter(new FileWriter(dir+minReachedDistfileStr)));
			PrintWriter maxReachedDistOut = new PrintWriter(new BufferedWriter(new FileWriter(dir+maxReachedDistfileStr)));
			PrintWriter meanReachedDistOut = new PrintWriter(new BufferedWriter(new FileWriter(dir+meanReachedDistfileStr)));
			
			PrintWriter meanTxLevelsOut = new PrintWriter(new BufferedWriter(new FileWriter(dir+meanTxLevelsfileStr)));
			
			PrintWriter allKRealOverhearingOut = new PrintWriter(new BufferedWriter(new FileWriter(dir+allKRealOverfileStr)));
			PrintWriter allMaxRealOverhearingOut = new PrintWriter(new BufferedWriter(new FileWriter(dir+allMaxRealOverfileStr)));			
			
			for (int t = 0; t < sizes.length; t++) {
				/*
				fracKOut.println(sizes[t]+" "+fracK[t]*100);
				ratioOut.println(sizes[t]+" "+ratio[t]*100);
				*/
				
				perMultihopOut.println(sizes[t]+" "+perMultihop[t]*100);
				perInequationOut.println(sizes[t]+" "+perInequation[t]*100);				
				totalHopsOut.println(sizes[t]+" "+totalHops[t]);
				percRecpOut.println(sizes[t]+" "+percRecp[t]*100);
				fatorOut.println(sizes[t]+" "+fator[t]);
				maxFatorOut.println(sizes[t]+" "+maxFator[t]);
				distancesOut.println(sizes[t]+" "+distances[t]);
				totalDistancesOut.println(sizes[t]+" "+totalDistances[t]);
				apartDistancesOut.println(sizes[t]+" "+apartDistances[t]);
				totalDistancesABOut.println(sizes[t]+" "+totalDistancesAB[t]);
				
				totalEpsilonPartOut.println(sizes[t]+" "+totalEpsilonPart[t]);
				totalElecTxPartOut.println(sizes[t]+" "+totalElecTxPart[t]);
				totalElecRxPartOut.println(sizes[t]+" "+totalElecRxPart[t]);
				
				maxHopsRatioOut.println(sizes[t]+" "+maxHopsRatio[t]);
				//meanHopsRatioOut.println(sizes[t]+" "+meanHopsRatio[t]);
				
				minReachedDistOut.println(sizes[t]+" "+minReachedDist[t]);				
				maxReachedDistOut.println(sizes[t]+" "+maxReachedDist[t]);
				meanReachedDistOut.println(sizes[t]+" "+meanReachedDist[t]);
				
				meanTxLevelsOut.println(sizes[t]+" "+meanTxLevels[t]);
				
				allKRealOverhearingOut.println(sizes[t]+" "+allKRealOverhearing[t]);
				allMaxRealOverhearingOut.println(sizes[t]+" "+allMaxRealOverhearing[t]);				
			}
			/*
			fracKOut.close();
			ratioOut.close();
			*/
			
			perMultihopOut.close();
			perInequationOut.close();			
			totalHopsOut.close();
			percRecpOut.close();
			fatorOut.close();
			maxFatorOut.close();
			distancesOut.close();
			totalDistancesOut.close();
			apartDistancesOut.close();
			totalDistancesABOut.close();
			totalEpsilonPartOut.close();
			totalElecTxPartOut.close();
			totalElecRxPartOut.close();
			
			maxHopsRatioOut.close();
			//meanHopsRatioOut.close();
			
			minReachedDistOut.close();
			maxReachedDistOut.close();
			meanReachedDistOut.close();
			
			meanTxLevelsOut.close();
			
			allKRealOverhearingOut.close();
			allMaxRealOverhearingOut.close();			
			
			out.close();
			//outMax.close();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}		
	}
	
	/**
	 * @param args
	 */
	public static void main(String[] args) {
		RadioParameters radioParameters = new RadioParameters();
		int sizes[] = RunTCAlgorithm.getNetSizes();
		//String dir = "/home/hugo/desenvolvimento/aplicativos/simulacao/omnetpp-4.1b3/samples/TCAlgorithm/";
		String dir = "/home/hugo/flavio/";
		String outFile = "posicao_potencia_TotalSymmetric-CC2420OverhearingInfoAfterControlPowerMST.dat";
		//String radioName = RadioParameters.radioName+"-600-";
		String radioName = RadioParameters.radioName+"-alfa"+RadioParameters.npathLossExp+"-size-"+FIELD_X;
		Date d = new Date();
		String tempFile = "tempFile-"+d.getTime();
		Random random = new Random();
		
		//here we choose the TC protocol
		getTCOverhearingData(sizes, dir, outFile, tempFile, radioName, random);
		//getSimpleTCProtocolData(sizes, dir, outFile, tempFile, radioName, random);
	}
}