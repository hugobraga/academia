package script;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.io.File;

//import main.Support;
/*vai chamar algoritmo que implementa UB para o caso unitário. Mais 
especificadamente, se para uma determinada instância admitir um bounded 
diameter spanning tree, o algoritmo encontra uma solução ótima sem 
precisar executar o cplex.

Assim que percebemos que é possível encontrar uma solução ótima, nós não a 
calculamos. Além disso, asim que vemos para um determinado fator t já 
encontramos uma solução, não executamos o algoritmo para esta instância 
para fatores maiores, pois já sabemos que a solução é a mesma de t.
 */

public class NewStatistics {
    private static final int DEBUG = 2;
    private static final boolean TREE_VERSION = true;//eh um problema de árvore?
    private static final boolean GET_MODEL_STATS = false;//estou interessado nos dados das formulações lineares?
    private static final boolean CG_ALGORITHM = true;//está sendo utilizado a implementação baseada em geração de colunas?

    /*valores das constantes para quando o pré-processamento, no caso unitário, é/não é utilizado no algoritmo baseado em GC
     */
    private static final int UNIT_BOUND_USED = -2;
    private static final int UNIT_BOUND_UNUSED = -3;

    /*constantes para quando o limite de tempo é excedido e quando é comprovando 
que a instância não admite solução, respectivamente*/
    private static final int TIME_LIMIT_CONST = 0;
    private static final int NO_SOLUTION_CONST = -1;

    //quando não foi possível achar/calcular o gap de integralidade.
    private static final double NO_INTEGRALITY_GAP = -1;
    
    
    // private static final int QT_EDGES[][] = {
    // 	{50, 75, 100, 125, 150, 175, 200},
    // 	{75, 100, 125, 150, 175, 200},
    // 	{100, 125, 150, 175, 200}
    // };
    
    private static int DEGREES[][] = {
	// {4, 8},
	{4}
    };
    
    private static int QT_EDGES_UNI[] = {100, 200, 300};
    
    private static int QT_EDGES[][] = {
    	{// 50, 75, 100, 
	    100, 120},
    	{// 75, 100, 
	    100, 120}
    };

    
    private static double DENS[][] = {
	// {0.7}
    	// {0.1},
	// {0.1, 0.2},
	// {0.1, 0.2}
	
	// {0.4, 0.5},
	// {0.3},
	// {0.3}
	
	{0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0},
    	{0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0},	
	{0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9}
	// {0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0}
	
	// {0.8, 0.9},
	// {0.7}	
	
	// {0.5, 0.6, 0.7},
    	// {0.4, 0.5, 0.6},
	// {0.3}

	// {0.4}
	
	// {0.1, 0.2, 0.3, 0.4},
    	// {0.1, 0.2},	
	// {0.1, 0.2}
	
	// {0.4}
	
    	// {0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0},	
	// {0.2, 0.3, 0.4, 0.5},
	// {0.2, 0.3, 0.4}
	
    	// {0.1, 0.2, 0.3},	
	// {0.1, 0.2},
	// {0.1, 0.2}
	
	// {0.2, 0.3, 0.4},
    	// {0.2, 0.3, 0.4},	
	// {0.1, 0.2, 0.3}	
    	// {0.8, 0.9, 1.0},	
	// {0.5, 0.6},
	// {0.4, 0.5}
	
	// {0.2, 0.3, 0.4},
    	// {0.2, 0.3, 0.4},	
	// {0.1, 0.2, 0.3}	
	// {0.1, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0},
    	// {0.1, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0},	
	// {0.4, 0.5},
	// {0.1, 0.2, 0.3, 0.4}
	
	// {0.7}
    	// {0.8, 0.9, 1.0},	
	// {0.5, 0.6},
	// {0.4, 0.5}	
	// {0.1, 0.2},
	// {0.1, 0.9, 1.0}
	// {0.2, 0.3, 0.4},
    	// {0.2, 0.3, 0.4},	
	// {0.1, 0.2, 0.3}	
    	// {0.6, 0.7},	
	// {0.4},
	// {0.3}	
    	// {0.2, 0.3, 0.4, 0.5, 0.6, 0.7},	
	// {0.2, 0.3, 0.4, 0.5, 0.6},
	// {0.2, 0.3, 0.4, 0.5}
	// {0.2, 0.3},
	// {0.2}	
	// {0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0},
    	// {0.8, 0.9, 1.0},	
	// {0.7, 0.8, 0.9, 1.0},
	// {0.6, 0.7, 0.8, 0.9, 1.0}
    	// {0.1, 0.2, 0.3, 0.4, 0.5},
	// {0.1, 0.2, 0.3, 0.4, 0.5},
	// {0.1, 0.2, 0.3, 0.4, 0.5}
	// {0.4, 0.5},
	// {0.4, 0.5},
	// {0.4, 0.5},
	// {0.5, 0.6, 0.7},
	// {0.4, 0.5, 0.6},
	// {0.3, 0.4, 0.5}
    	// {0.15, 0.2},
	// {0.15, 0.2}
	
    };


    private static final String NOME_PARAM = "edge";
    private static final int QT_INST = 10;
    private static final int[] SIZES = {20, 30, 40};
    // private static final double[] STR_FACTORS = {1.1, 2.0, 3.0, 4.0};
    private static final double[] STR_FACTORS = {3.0, 4.0};
    // private static final int[] QT_EDGES = {50, 75, 100, 150, 200, 250, 300};
    // private static final int[] QT_EDGES = {50, 75, 100, 125, 150, 175, 200};
    private static final int[] EDGES_PERC = {20, 40, 60, 80};
    // private static final String EDGE_COST[] = {"unit", "random", "euclidean"};
    private static final String EDGE_COST[] = {"random"};
    private static final String ADD_EDGE_TYPE[] = {"random"};
    private static final String ALG[] = {"tree_spanner_cg_30"};
    // private static final String ALG[] = {"tree_spanner_cg_unit"};
    // private static final String ALG[] = {"tree_labels_dist_spanner", "tree_labels_dist_spanner"};
    // private static final String ALG[] = {"tree_find_dist_spanner", "tree_find_dist_spanner"};
    private static final String OUT_FILE_COMP = "";
    private static final String IN_DIR = "/home/hugo/Dropbox/USP/git/doutorado/tese/experimentos/dados_random/";
    private static final String OUT_DIR = "/tmp/average_data/";
    private static final String TIME_STATISTICS_OUT_FILE = "TreeCGStatistics.dat";
    private static final String TIME_OUT_FILE = NewStatistics.OUT_FILE_COMP+"time_output.dat";
    private static final String COUNT_TIME_FILE = NewStatistics.OUT_FILE_COMP+"count_time.dat";
    private static final String WEIGHTED = NewStatistics.OUT_FILE_COMP+"count_time.dat";
    //private static final String EDGE_OUT_FILE = NewStatistics.OUT_FILE_COMP+"edge_output.dat";
    //private static final String OPT_OUT_FILE = NewStatistics.OUT_FILE_COMP+"opt_output.dat";

    private static final int NULL_CONST = -2;

    private static void readSpannerCGInput(String fileName, int[] count, double[] time, double[] opt, double[] gap) {

	File file = null;
	FileReader fileReader = null;
	BufferedReader bufferedReader = null;
	
	try {
	    System.out.println("nome arquivo: " + fileName);
	    file = new File(fileName);
	    fileReader = new FileReader(file);
	    bufferedReader = new BufferedReader(fileReader);

	    String line;

	    // System.out.println("---------------");
	    // System.out.println(fileName);
	    // System.out.println("---------------");

	    double solvedOptSum = 0;
	    double solvedTimeSum = 0;
	    //double solvedTreeTimeSum = 0;
	    double solvedWPrepTimeSum = 0;
	    double solvedWILPTimeSum = 0;
	    double integralityGapSum = 0;
	    int gapCount = 0;
	    int solvedCount = 0;
	    //int solvedTreeCount = 0;
	    int solvedWPrepCount = 0;
	    int solvedWILPCount = 0;
	    for (int z = 0; z < NewStatistics.QT_INST; z++) {
	    // while ((line = bufferedReader.readLine()) != null) {
		line = bufferedReader.readLine();
		// System.out.println("line: "+ line);
		String[] splitArray = line.split("\\s+");
		int instance = Integer.parseInt(splitArray[0]);
		int instanceFlag = Integer.parseInt(splitArray[1]);
		double val = Double.parseDouble(splitArray[2]);

		if (instanceFlag != NewStatistics.TIME_LIMIT_CONST) {//instance solved
		// if (val > 0) { //instance solved
		    solvedCount++;
		    double optVal = Double.parseDouble(splitArray[1]);
		    solvedOptSum += optVal;
		    solvedTimeSum += val;
		}
		else { //not solved
		    if (val != NO_INTEGRALITY_GAP) {
			gapCount++;
			integralityGapSum += val;
		    }
		}
	    }

	    count[0] = solvedCount;
	    if (gapCount > 0) {
		gap[0] = integralityGapSum / gapCount;
		gap[1] = gapCount;
	    // if (count[0] < NewStatistics.QT_INST) {	       
		// gap[0] = integralityGapSum / (NewStatistics.QT_INST - count[0]);
	    } else {
		gap[0] = 0;
		gap[1] = 0;
	    }	    
	    if (count[0] > 0){
		time[0] = solvedTimeSum/solvedCount;
		opt[0] = solvedOptSum/solvedCount;
	    } else {
		time[0] = 0;
		opt[0] = 0;
	    }
	} catch (IOException x) {
	    //result = false;
	} finally {
	    //if (file != null) { try { file.close(); } catch(Throwable t) { } }
	    if (bufferedReader != null) { try { bufferedReader.close(); } catch(Throwable t) { } }
	    if (fileReader != null) { try { fileReader.close(); } catch(Throwable t) { } }
	}
	
    }

    private static void readTreeUnitInput(String fileName, int[] count, double[] time, double[] gap) {
	File file = null;
	FileReader fileReader = null;
	BufferedReader bufferedReader = null;
	
	try {
	    if (DEBUG == 1)
		System.out.println("nome arquivo: " + fileName);
	    file = new File(fileName);
	    fileReader = new FileReader(file);
	    bufferedReader = new BufferedReader(fileReader);

	    String line;

	    // System.out.println("---------------");
	    // System.out.println(fileName);
	    // System.out.println("---------------");

	    double solvedTimeSum = 0;
	    double solvedTreeTimeSum = 0;
	    double solvedWPrepTimeSum = 0;
	    double solvedWILPTimeSum = 0;
	    double integralityGapSum = 0;
	    int gapCount = 0;
	    int solvedCount = 0;
	    int solvedTreeCount = 0;
	    int solvedWPrepCount = 0;
	    int solvedWILPCount = 0;

	    for (int z = 0; z < NewStatistics.QT_INST; z++) {
	    // while ((line = bufferedReader.readLine()) != null) {
		line = bufferedReader.readLine();
		// System.out.println("line: "+ line);
		String[] splitArray = line.split("\\s+");
		int instance = Integer.parseInt(splitArray[0]);
		int instanceFlag = Integer.parseInt(splitArray[1]);
		double val = Double.parseDouble(splitArray[2]);		
		
		if (instanceFlag != NewStatistics.TIME_LIMIT_CONST) {//instance solved
		    // int flag = Integer.parseInt(splitArray[1]);
		    
		    solvedCount++;		    
		    solvedTimeSum += val;
		    
		    if (instanceFlag != NewStatistics.NO_SOLUTION_CONST) {
			solvedTreeCount++;
			solvedTreeTimeSum += val;

			if (!NewStatistics.CG_ALGORITHM) {

			    if (instanceFlag == NewStatistics.UNIT_BOUND_USED) {//solved w preprocessing
				solvedWPrepCount++;
				solvedWPrepTimeSum += val;
			    } else {
				solvedWILPCount++;
				solvedWILPTimeSum += val;
			    }
			    
			}
			
		    } else {//do not admit solution
			solvedWILPCount++;
			solvedWILPTimeSum += val;
		    }
		    // System.out.println("inst: " + instance + ", val: " + val);

		} else { //not solved
		    if (val != NO_INTEGRALITY_GAP) {
			gapCount++;
			integralityGapSum += val;
		    }		    
		}
	    }

	    count[0] = solvedCount;
	    if (!NewStatistics.CG_ALGORITHM)
		count[2] = solvedWILPCount;
	    // count[1] = solvedWPrepCount;
	    count[1] = solvedTreeCount;
	    if (gapCount > 0) {
		gap[0] = integralityGapSum / gapCount;
		gap[1] = gapCount;
	    // if (count[0] < NewStatistics.QT_INST) {	       
		// gap[0] = integralityGapSum / (NewStatistics.QT_INST - count[0]);
	    } else {
		gap[0] = 0;
		gap[1] = 0;
	    }
	    if (count[0] > 0)
		time[0] = solvedTimeSum/solvedCount;
	    else
		time[0] = 0;
	    if (!NewStatistics.CG_ALGORITHM) {

		if (solvedWILPCount > 0)
		    time[2] = solvedWILPTimeSum/solvedWILPCount;
		else
		    time[2] = 0;
		
	    }
	    if (solvedTreeCount > 0)
		time[1] = solvedTreeTimeSum/solvedTreeCount;
	    // if (solvedWPrepCount > 0)
		// time[2] = solvedWPrepTimeSum/solvedWPrepCount;
	    else
		time[1] = 0;

	    if (DEBUG == 1) {
		System.out.println("count[0]: " + count[0] + ", count[1]: " + count[1] + ", count[2]: " + count[2]);
		System.out.println("time[0]: " + time[0] + ", time[1]: " + time[1] + ", time[2]: " + time[2]);
		System.out.println("gap[0]: " + gap[0] + ", gap[1]: " + gap[1]);
	    }	    
			    
	} catch (IOException x) {
	    //result = false;
	} finally {
	    //if (file != null) { try { file.close(); } catch(Throwable t) { } }
	    if (bufferedReader != null) { try { bufferedReader.close(); } catch(Throwable t) { } }
	    if (fileReader != null) { try { fileReader.close(); } catch(Throwable t) { } }
	}
    }

    
    private static void readTreeUnitInput(String fileName, int[] count, double[] time, double[] gap, int[] qtColumns, int[] qtRows, int[] qtNZC) {
	File file = null;
	FileReader fileReader = null;
	BufferedReader bufferedReader = null;
	
	try {
	    if (DEBUG == 1)
		System.out.println("nome arquivo: " + fileName);
	    file = new File(fileName);
	    fileReader = new FileReader(file);
	    bufferedReader = new BufferedReader(fileReader);

	    String line;

	    // System.out.println("---------------");
	    // System.out.println(fileName);
	    // System.out.println("---------------");

	    double solvedTimeSum = 0;
	    double solvedTreeTimeSum = 0;
	    double solvedWPrepTimeSum = 0;
	    double solvedWILPTimeSum = 0;
	    double integralityGapSum = 0;
	    int gapCount = 0;
	    int solvedCount = 0;
	    int solvedTreeCount = 0;
	    int solvedWPrepCount = 0;
	    int solvedWILPCount = 0;

	    int qtModelColumns = 0;
	    int qtModelRows = 0;
	    int qtModelNZC = 0;
	    for (int z = 0; z < NewStatistics.QT_INST; z++) {
	    // while ((line = bufferedReader.readLine()) != null) {
		line = bufferedReader.readLine();
		// System.out.println("line: "+ line);
		String[] splitArray = line.split("\\s+");
		int instance = Integer.parseInt(splitArray[0]);
		int instanceFlag = Integer.parseInt(splitArray[1]);
		double val = Double.parseDouble(splitArray[2]);

		qtModelColumns += Integer.parseInt(splitArray[3]);
		qtModelRows += Integer.parseInt(splitArray[4]);
		qtModelNZC += Integer.parseInt(splitArray[5]);
		
		
		if (instanceFlag != NewStatistics.TIME_LIMIT_CONST) {//instance solved
		    // int flag = Integer.parseInt(splitArray[1]);
		    
		    solvedCount++;		    
		    solvedTimeSum += val;
		    
		    if (instanceFlag != NewStatistics.NO_SOLUTION_CONST) {
			solvedTreeCount++;
			solvedTreeTimeSum += val;

			if (!NewStatistics.CG_ALGORITHM) {

			    if (instanceFlag == NewStatistics.UNIT_BOUND_USED) {//solved w preprocessing
				solvedWPrepCount++;
				solvedWPrepTimeSum += val;
			    } else {
				solvedWILPCount++;
				solvedWILPTimeSum += val;
			    }
			    
			}
		    } else {//do not admit solution
			solvedWILPCount++;
			solvedWILPTimeSum += val;
		    }
		    // System.out.println("inst: " + instance + ", val: " + val);

		} else { //not solved
		    if (val != NO_INTEGRALITY_GAP) {
			gapCount++;
			integralityGapSum += val;
		    }		    
		}
	    }

	    count[0] = solvedCount;
	    if (!NewStatistics.CG_ALGORITHM)
		count[2] = solvedWILPCount;
	    // count[1] = solvedWPrepCount;
	    count[1] = solvedTreeCount;
	    if (gapCount > 0) {
		gap[0] = integralityGapSum / gapCount;
		gap[1] = gapCount;
	    // if (count[0] < NewStatistics.QT_INST) {	       
		// gap[0] = integralityGapSum / (NewStatistics.QT_INST - count[0]);
	    } else {
		gap[0] = 0;
		gap[1] = 0;
	    }
	    if (count[0] > 0)
		time[0] = solvedTimeSum/solvedCount;
	    else
		time[0] = 0;
	    if (!NewStatistics.CG_ALGORITHM) {
		if (solvedWILPCount > 0)
		    time[2] = solvedWILPTimeSum/solvedWILPCount;
		else
		    time[2] = 0;		
	    }
	    if (solvedTreeCount > 0)
		time[1] = solvedTreeTimeSum/solvedTreeCount;
	    // if (solvedWPrepCount > 0)
		// time[2] = solvedWPrepTimeSum/solvedWPrepCount;
	    else
		time[1] = 0;

	    qtColumns[0] = (int)Math.round((1.0 * qtModelColumns) / NewStatistics.QT_INST);
	    qtRows[0] = (int)Math.round((1.0 * qtModelColumns) / NewStatistics.QT_INST);
	    qtNZC[0] = (int)Math.round((1.0 * qtModelNZC) / NewStatistics.QT_INST);
	    if (DEBUG == 1) {
		System.out.println("count[0]: " + count[0] + ", count[1]: " + count[1] + ", count[2]: " + count[2]);
		System.out.println("time[0]: " + time[0] + ", time[1]: " + time[1] + ", time[2]: " + time[2]);
		System.out.println("gap[0]: " + gap[0] + ", gap[1]: " + gap[1]);
	    }	    
			    
	} catch (IOException x) {
	    //result = false;
	} finally {
	    //if (file != null) { try { file.close(); } catch(Throwable t) { } }
	    if (bufferedReader != null) { try { bufferedReader.close(); } catch(Throwable t) { } }
	    if (fileReader != null) { try { fileReader.close(); } catch(Throwable t) { } }
	}
    }

    private static void readTreeWeightedInput(String fileName, int[] count, double[] time, double[] opt, double[] gap) {
	File file = null;
	FileReader fileReader = null;
	BufferedReader bufferedReader = null;
	
	try {
	    System.out.println("readTreeWeightedInput, nome arquivo: " + fileName);
	    file = new File(fileName);
	    fileReader = new FileReader(file);
	    bufferedReader = new BufferedReader(fileReader);

	    String line;

	    // System.out.println("---------------");
	    // System.out.println(fileName);
	    // System.out.println("---------------");

	    double solvedOptSum = 0;
	    double solvedTimeSum = 0;
	    double solvedTreeTimeSum = 0;
	    double integralityGapSum = 0;
	    int gapCount = 0;
	    //double solvedWOPrepTimeSum = 0;
	    int solvedCount = 0;
	    int solvedTreeCount = 0;
	    //int solvedWOPrepCount = 0;

	    for (int z = 0; z < NewStatistics.QT_INST; z++) {
	    // while ((line = bufferedReader.readLine()) != null) {
		line = bufferedReader.readLine();
		System.out.println("linha do arquivo: " + line);
		// System.out.println("line: "+ line);
		String[] splitArray = line.split("\\s+");
		int instance = Integer.parseInt(splitArray[0]);
		double term1 = Double.parseDouble(splitArray[1]);
		double val = Double.parseDouble(splitArray[2]);
		
		if (term1 != NewStatistics.TIME_LIMIT_CONST) {//instance solved
		    
		    solvedCount++;		    
		    solvedTimeSum += val;

		    if (term1 != NewStatistics.NO_SOLUTION_CONST) {
			solvedTreeCount++;
			solvedTreeTimeSum += val;
			solvedOptSum += term1;
			
			// if (flag != 1) {//solved w/o preprocessing
			//     solvedWOPrepCount++;
			//     solvedWOPrepTimeSum += val;
			// }
		    }
		    // System.out.println("inst: " + instance + ", val: " + val);

		} else {
		    if (val != NO_INTEGRALITY_GAP) {
			gapCount++;
			integralityGapSum += val;
		    }
		}
	    }

	    count[0] = solvedCount;
	    count[1] = solvedTreeCount;
	    if (gapCount > 0) {
		gap[0] = integralityGapSum / gapCount;
		gap[1] = gapCount;
	    // if (count[0] < NewStatistics.QT_INST) {	       
		// gap[0] = integralityGapSum / (NewStatistics.QT_INST - count[0]);
	    } else {
		gap[0] = 0;
		gap[1] = 0;
	    }	    
	    if (count[0] > 0)
		time[0] = solvedTimeSum/solvedCount;
	    else
		time[0] = 0;
	    if (solvedTreeCount > 0) {
		time[1] = solvedTreeTimeSum/solvedTreeCount;
		opt[0] = solvedOptSum / solvedTreeCount;
	    } else {
		time[1] = 0;
		opt[0] = 0;
	    }
	    // if (solvedWOPrepCount > 0)
	    // 	time[2] = solvedWOPrepTimeSum/solvedWOPrepCount;
	    // else
	    // 	time[2] = 0;

	    
	} catch (IOException x) {
	    //result = false;
	} finally {
	    //if (file != null) { try { file.close(); } catch(Throwable t) { } }
	    if (bufferedReader != null) { try { bufferedReader.close(); } catch(Throwable t) { } }
	    if (fileReader != null) { try { fileReader.close(); } catch(Throwable t) { } }
	}
    }

    
    private static void readTreeWeightedInput(String fileName, int[] count, double[] time, double[] opt, double[] gap, int[] qtColumns, int[] qtRows, int[] qtNZC) {
	File file = null;
	FileReader fileReader = null;
	BufferedReader bufferedReader = null;
	
	try {
	    System.out.println("nome arquivo: " + fileName);
	    file = new File(fileName);
	    fileReader = new FileReader(file);
	    bufferedReader = new BufferedReader(fileReader);

	    String line;

	    // System.out.println("---------------");
	    // System.out.println(fileName);
	    // System.out.println("---------------");

	    double solvedOptSum = 0;
	    double solvedTimeSum = 0;
	    double solvedTreeTimeSum = 0;
	    double integralityGapSum = 0;
	    int gapCount = 0;
	    //double solvedWOPrepTimeSum = 0;
	    int solvedCount = 0;
	    int solvedTreeCount = 0;
	    //int solvedWOPrepCount = 0;

	    int qtModelColumns = 0;
	    int qtModelRows = 0;
	    int qtModelNZC = 0;

	    for (int z = 0; z < NewStatistics.QT_INST; z++) {
	    // while ((line = bufferedReader.readLine()) != null) {
		line = bufferedReader.readLine();
		// System.out.println("line: "+ line);
		String[] splitArray = line.split("\\s+");
		int instance = Integer.parseInt(splitArray[0]);
		double term1 = Double.parseDouble(splitArray[1]);
		double val = Double.parseDouble(splitArray[2]);

		qtModelColumns += Integer.parseInt(splitArray[3]);
		qtModelRows += Integer.parseInt(splitArray[4]);
		qtModelNZC += Integer.parseInt(splitArray[5]);

		
		if (term1 != NewStatistics.TIME_LIMIT_CONST) {//instance solved
		    
		    solvedCount++;		    
		    solvedTimeSum += val;

		    if (term1 != NewStatistics.NO_SOLUTION_CONST) {
			solvedTreeCount++;
			solvedTreeTimeSum += val;
			solvedOptSum += term1;
			
			// if (flag != 1) {//solved w/o preprocessing
			//     solvedWOPrepCount++;
			//     solvedWOPrepTimeSum += val;
			// }
		    }
		    // System.out.println("inst: " + instance + ", val: " + val);

		} else {
		    if (val != NO_INTEGRALITY_GAP) {
			gapCount++;
			integralityGapSum += val;
		    }
		}
	    }

	    count[0] = solvedCount;
	    count[1] = solvedTreeCount;
	    if (gapCount > 0) {
		gap[0] = integralityGapSum / gapCount;
		gap[1] = gapCount;
	    // if (count[0] < NewStatistics.QT_INST) {	       
		// gap[0] = integralityGapSum / (NewStatistics.QT_INST - count[0]);
	    } else {
		gap[0] = 0;
		gap[1] = 0;
	    }	    
	    if (count[0] > 0)
		time[0] = solvedTimeSum/solvedCount;
	    else
		time[0] = 0;
	    if (solvedTreeCount > 0) {
		time[1] = solvedTreeTimeSum/solvedTreeCount;
		opt[0] = solvedOptSum / solvedTreeCount;
	    } else {
		time[1] = 0;
		opt[0] = 0;
	    }
	    // if (solvedWOPrepCount > 0)
	    // 	time[2] = solvedWOPrepTimeSum/solvedWOPrepCount;
	    // else
	    // 	time[2] = 0;

	    qtColumns[0] = (int)Math.round((1.0 * qtModelColumns) / NewStatistics.QT_INST);
	    qtRows[0] = (int)Math.round((1.0 * qtModelColumns) / NewStatistics.QT_INST);
	    qtNZC[0] = (int)Math.round((1.0 * qtModelNZC) / NewStatistics.QT_INST);

	    
	} catch (IOException x) {
	    //result = false;
	} finally {
	    //if (file != null) { try { file.close(); } catch(Throwable t) { } }
	    if (bufferedReader != null) { try { bufferedReader.close(); } catch(Throwable t) { } }
	    if (fileReader != null) { try { fileReader.close(); } catch(Throwable t) { } }
	}
    }


    private static void calculateAndWriteDataToFile(String inFileName, String outFileName, double sf, double param1, int param2, String edgeCost, int order) {

	File file = null;
	FileReader fileReader = null;
	BufferedReader bufferedReader = null;

	double[] optAvg = new double[1];
	int[] count;
	double[] timeAvg;
	double[] gap = new double[2];

	int[] qtModelColumns = new int[1];
	int[] qtModelRows = new int[1];
	int[] qtModelNZC = new int[1];


	optAvg[0] = order - 1;
	if (NewStatistics.TREE_VERSION) {

	    if (edgeCost.equals(new String("unit"))) {
		if (!NewStatistics.CG_ALGORITHM) {
		    count = new int[3];
		    timeAvg = new double[3];
		} else {
		    count = new int[2];
		    timeAvg = new double[2];		    
		}

		if (NewStatistics.GET_MODEL_STATS) {
		    readTreeUnitInput(inFileName, count, timeAvg, gap, qtModelColumns, qtModelRows, qtModelNZC);		    
		} else
		    readTreeUnitInput(inFileName, count, timeAvg, gap);
	    } else {
		count = new int[2];
		timeAvg = new double[2];

		if (NewStatistics.GET_MODEL_STATS)
		    readTreeWeightedInput(inFileName, count, timeAvg, optAvg, gap, qtModelColumns, qtModelRows, qtModelNZC);		    
		else
		    readTreeWeightedInput(inFileName, count, timeAvg, optAvg, gap);
	    }

	    if (NewStatistics.GET_MODEL_STATS)
		writeCountTimeOpt(outFileName, sf, param1, param2, edgeCost, count, timeAvg, optAvg, gap, qtModelColumns, qtModelRows, qtModelNZC);		
	    else
		writeCountTimeOpt(outFileName, sf, param1, param2, edgeCost, count, timeAvg, optAvg, gap);
	} else {
	    count = new int[1];
	    timeAvg = new double[1];

	    readSpannerCGInput(inFileName, count, timeAvg, optAvg, gap);

	    writeCountTimeOpt(outFileName, sf, param1, param2, edgeCost, count, timeAvg, optAvg, gap);
	}							    
	
    }

    

    private static void orderNotParamStatistics() {
    	for (int k = 0; k < NewStatistics.QT_EDGES_UNI.length; k++) {
    	// for (int i = 0; i < NewStatistics.SIZES.length; i++) {
    	    for (int d = 0; d < NewStatistics.DENS[k].length; d++) {
    	    // for (int k = 0; k < NewStatistics.QT_EDGES_UNI[i].length; k++) {
    		for (int ec = 0; ec < NewStatistics.EDGE_COST.length; ec++) {
    		    for (int aet = 0; aet < NewStatistics.ADD_EDGE_TYPE.length; aet++) {
    			for (int sf = 0; (sf < NewStatistics.STR_FACTORS.length); sf++) {

    			    // for (int j = 1; j <= NewStatistics.QT_INST; j++) {
    				int order = (int)Math.round((1 + Math.sqrt(1 + (8 * NewStatistics.QT_EDGES_UNI[k] / NewStatistics.DENS[k][d])))/2);
				if (DEBUG == 1)
				    System.out.println("order: " + order);

				
				String inFile = new String(""+NewStatistics.IN_DIR+NewStatistics.STR_FACTORS[sf] + "-" + NewStatistics.QT_EDGES_UNI[k] + "-" + (int)Math.round(100*NewStatistics.DENS[k][d]) + "-size" + "-" + NewStatistics.EDGE_COST[ec] + "-" + NewStatistics.ADD_EDGE_TYPE[aet] + "-" + NewStatistics.ALG[ec] + "-" + NewStatistics.TIME_OUT_FILE);
				if (DEBUG == 0)
				    System.out.println("arquivo de entrada: " + inFile);
							    

				String outFile = new String(NewStatistics.OUT_DIR+""+"treeSpannerSizeResults.dat");
				// calculateAndWriteDataToFile(inFile, outFile, NewStatistics.STR_FACTORS[sf], order, (int)Math.round(100*NewStatistics.DENS[k][d]), NewStatistics.EDGE_COST[ec], order);
				calculateAndWriteDataToFile(inFile, outFile, NewStatistics.STR_FACTORS[sf], NewStatistics.QT_EDGES_UNI[k], (int)Math.round(100*NewStatistics.DENS[k][d]), NewStatistics.EDGE_COST[ec], order);
			    // }
								
    			    // }
			    
    			}
			
    		    }
    		}
    	    }
    	}
	
    }


    private static void orderAndDegreeParamStatistics() {
    	for (int i = 0; i < NewStatistics.SIZES.length; i++) {
    	    for (int k = 0; k < NewStatistics.DEGREES[i].length; k++) {
    		for (int ec = 0; ec < NewStatistics.EDGE_COST.length; ec++) {
    		    for (int aet = 0; aet < NewStatistics.ADD_EDGE_TYPE.length; aet++) {

			
    			for (int sf = 0; (sf < NewStatistics.STR_FACTORS.length); sf++) {

    			    // for (int j = 1; j <= NewStatistics.QT_INST; j++) {
				double perc = (1.0*NewStatistics.DEGREES[i][k])/(NewStatistics.SIZES[i]-1);


				String inFile = new String(""+NewStatistics.IN_DIR+NewStatistics.STR_FACTORS[sf]+"-"+NewStatistics.DEGREES[i][k]+"-"+NewStatistics.SIZES[i]+"-order_degree-"+NewStatistics.EDGE_COST[ec]+"-"+NewStatistics.ADD_EDGE_TYPE[aet]+"-"+NewStatistics.ALG[ec]+"-"+NewStatistics.TIME_OUT_FILE);
							    

				String outFile = new String(NewStatistics.OUT_DIR+""+"treeSpannerSizeDegreeResults.dat");
				calculateAndWriteDataToFile(inFile, outFile, NewStatistics.STR_FACTORS[sf], NewStatistics.DEGREES[i][k], NewStatistics.SIZES[i], NewStatistics.EDGE_COST[ec], NewStatistics.SIZES[i]);
    			    // }
			    
    			}
			
    		    }
    		}
    	    }
    	}
	
    }

    
     private static void qtEdgesNotParamStatistics() {
    	for (int i = 0; i < NewStatistics.SIZES.length; i++) {
    	    for (int k = 0; k < NewStatistics.DENS[i].length; k++) {
    		for (int ec = 0; ec < NewStatistics.EDGE_COST.length; ec++) {
    		    for (int aet = 0; aet < NewStatistics.ADD_EDGE_TYPE.length; aet++) {
			
    			for (int sf = 0; (sf < NewStatistics.STR_FACTORS.length); sf++) {

    			    // for (int j = 1; j <= NewStatistics.QT_INST; j++) {


				String inFile = new String(""+NewStatistics.IN_DIR+NewStatistics.STR_FACTORS[sf]+"-"+ (int)Math.round(100*NewStatistics.DENS[i][k]) + "-" + NewStatistics.SIZES[i] + "-edges-" +NewStatistics.EDGE_COST[ec]+"-"+NewStatistics.ADD_EDGE_TYPE[aet]+"-"+NewStatistics.ALG[ec]+"-"+NewStatistics.TIME_OUT_FILE);
				System.out.println("linha: " + inFile + ", custo: " + NewStatistics.EDGE_COST[ec]);
							    

				String outFile = new String(NewStatistics.OUT_DIR+""+"treeSpannerEdgesResults.dat");
				calculateAndWriteDataToFile(inFile, outFile, NewStatistics.STR_FACTORS[sf], (int)Math.round(100*NewStatistics.DENS[i][k]), NewStatistics.SIZES[i], NewStatistics.EDGE_COST[ec], NewStatistics.SIZES[i]);

				
				
    			    // }
			    
    			}
			
    		    }
    		}
    	    }
    	}
	
    }


    private static void densNotParamStatistics() {

    	for (int i = 0; i < NewStatistics.SIZES.length; i++) {
    	    for (int k = 0; k < NewStatistics.QT_EDGES[i].length; k++) {
    		for (int ec = 0; ec < NewStatistics.EDGE_COST.length; ec++) {
    		    for (int aet = 0; aet < NewStatistics.ADD_EDGE_TYPE.length; aet++) {

			
    			for (int sf = 0; (sf < NewStatistics.STR_FACTORS.length); sf++) {			    

    			    // for (int j = 1; j <= NewStatistics.QT_INST; j++) {
    				double perc = (2.0 * NewStatistics.QT_EDGES[i][k])/(NewStatistics.SIZES[i] * (NewStatistics.SIZES[i]-1));

				String inFile = new String(""+NewStatistics.IN_DIR+NewStatistics.STR_FACTORS[sf]+"-" + NewStatistics.QT_EDGES[i][k] + "-" + NewStatistics.SIZES[i] + "-dens-" + NewStatistics.EDGE_COST[ec]+"-"+NewStatistics.ADD_EDGE_TYPE[aet]+"-"+NewStatistics.ALG[ec]+"-"+NewStatistics.TIME_OUT_FILE);

				String outFile = new String(NewStatistics.OUT_DIR+""+"treeSpannerEdgesResults.dat");
				calculateAndWriteDataToFile(inFile, outFile, NewStatistics.STR_FACTORS[sf], NewStatistics.QT_EDGES[i][k], NewStatistics.SIZES[i], NewStatistics.EDGE_COST[ec], NewStatistics.SIZES[i]);

				
    			    // }
			    
    			}
			
    		    }
    		}
    	    }
    	}
	
    }


    private static void writeCountTimeOpt(String fileName, double sf, double param1, int param2, String edgeCost, int[] count, double[] timeSum, double[] opt, double[] gap) {
	BufferedWriter bw = null;
	FileWriter fw = null;

	try {
	    // if (count[0] > 0) {
		fw = new FileWriter(fileName, true);
		bw = new BufferedWriter(fw);
		//sum = sum / count;
		// System.out.println("tam do vetor count: " + count.length);

		bw.write(String.valueOf(sf + " " + param1 + " " + param2 + " " + edgeCost + " " + count[0] + " " + timeSum[0] + " " + opt[0]));

		if (count.length >= 2)
		    bw.write(String.valueOf(" " + count[1] + " " + timeSum[1]));

		bw.write(String.valueOf(" " + gap[0] + " " + gap[1]));
		
		if (count.length >= 3)
		    bw.write(String.valueOf(" " + count[2] + " " + timeSum[2]));
		
		bw.newLine();
		//bw.write("teste");
		//System.out.println("escreveu no arquivo");
	    // }
	} catch (IOException x) {
	} finally {
	    if (bw != null) { try { bw.close(); } catch(Throwable t) { } }
	    if (fw != null) { try { fw.close(); } catch(Throwable t) { } }
	}
    }
    
       
    private static void writeCountTimeOpt(String fileName, double sf, double param1, int param2, String edgeCost, int[] count, double[] timeSum, double[] opt, double[] gap, int[] qtColumns, int[] qtRows, int[] qtNZC) {
	BufferedWriter bw = null;
	FileWriter fw = null;

	try {
	    // if (count[0] > 0) {
		fw = new FileWriter(fileName, true);
		bw = new BufferedWriter(fw);
		//sum = sum / count;
		// System.out.println("tam do vetor count: " + count.length);

		bw.write(String.valueOf(sf + " " + param1 + " " + param2 + " " + edgeCost + " " + count[0] + " " + timeSum[0] + " " + opt[0]));

		if (count.length >= 2)
		    bw.write(String.valueOf(" " + count[1] + " " + timeSum[1]));

		bw.write(String.valueOf(" " + gap[0] + " " + gap[1]));

		bw.write(String.valueOf(" " + qtColumns[0] + " " + qtRows[0] + " " + qtNZC[0]));
		
		if (count.length >= 3)
		    bw.write(String.valueOf(" " + count[2] + " " + timeSum[2]));
		
		bw.newLine();
		//bw.write("teste");
		//System.out.println("escreveu no arquivo");
	    // }
	} catch (IOException x) {
	} finally {
	    if (bw != null) { try { bw.close(); } catch(Throwable t) { } }
	    if (fw != null) { try { fw.close(); } catch(Throwable t) { } }
	}
    }

    
    private static void writeCountTimeOpt(String fileName, int[] count, double[] timeSum, double[] opt) {
	BufferedWriter bw = null;
	FileWriter fw = null;

	try {
	    // if (count[0] > 0) {
		fw = new FileWriter(fileName);
		bw = new BufferedWriter(fw);
		//sum = sum / count;

		bw.write(String.valueOf(count[0] + " " + timeSum[0] + " " + opt[0]));

		//bw.newLine();
		//bw.write("teste");
		System.out.println("escreveu no arquivo");
	    // }
	} catch (IOException x) {
	} finally {
	    if (bw != null) { try { bw.close(); } catch(Throwable t) { } }
	    if (fw != null) { try { fw.close(); } catch(Throwable t) { } }	    
	}
    }

    
    private static void writeCountAndTime(String fileName, int[] count, double[] timeSum) {
	BufferedWriter bw = null;
	FileWriter fw = null;

	try {
	    if (count[0] > 0) {
		fw = new FileWriter(fileName);
		bw = new BufferedWriter(fw);
		//sum = sum / count;

		bw.write(String.valueOf(timeSum[0] + " " + count[0] + " " + timeSum[1] + " " + count[1]));
		if (count.length == 3)
		    bw.write(String.valueOf(" " + timeSum[2] + " " + count[2]));

		//bw.newLine();
		//bw.write("teste");
		//System.out.println("escreveu no arquivo");
	    }
	} catch (IOException x) {
	} finally {
	    if (bw != null) { try { bw.close(); } catch(Throwable t) { } }
	    if (fw != null) { try { fw.close(); } catch(Throwable t) { } }	    
	}
    }
    
    public static void main(String[] args) {
	// NewStatistics.orderNotParamStatistics();
	// NewStatistics.orderAndDegreeParamStatistics();
	NewStatistics.qtEdgesNotParamStatistics();
	// NewStatistics.densNotParamStatistics();
    }
}
