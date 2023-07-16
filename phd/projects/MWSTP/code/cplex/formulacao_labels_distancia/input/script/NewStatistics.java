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
    //solved answer
    public static final int NOT_SOLVED = 0;
    //public static final int NO_TREE = 0;
    public static final int FOUND_TREE = 1;

    public static final int WITH_PREP = 5;
    public static final int WITHOUT_PREP = 5;

    //input file flags
    public static final int NO_TREE = -1;
    
    //public static final int NO_VALUE = -1;    
    
    public static final int QT_INST = 10;
    public static final int[] SIZES = {10, 20, 30, 40, 48};
    public static final int[] STR_FACTORS = {3, 4, 5, 6};
    public static final int[] EDGES_PERC = {20, 40, 60, 80};
    public static final String EDGE_COST = "euclidean";
    public static final String ALG = "tree_labels_dist_spanner_etc";
    public static final String FLOAT_ALG = "tree_labels_dist_spanner_etc_float";
    public static final String OUT_FILE_COMP = "-";
    public static final String TIME_DIR_OUT_FILE = "saida_euclidean/";
    public static final String TIME_OUT_FILE = NewStatistics.OUT_FILE_COMP+"time_output.dat";
    public static final String COUNT_TIME_FILE = NewStatistics.OUT_FILE_COMP+"count_time.dat";
    public static final String COUNT_TIME_GAP_FILE = NewStatistics.OUT_FILE_COMP+"count_time_gap.dat";
    public static final String COUNT_SOL_FILE = NewStatistics.OUT_FILE_COMP+"count_sol.dat";
    public static final String COUNT_SOL_GAP_FILE = NewStatistics.OUT_FILE_COMP+"count_sol_gap.dat";    
    public static final String FLOAT_COUNT_TIME_FILE = NewStatistics.OUT_FILE_COMP+"float_count_time.dat";
    public static final String FLOAT_COUNT_TIME_GAP_FILE = NewStatistics.OUT_FILE_COMP+"float_count_time_gap.dat";    
    public static final String FLOAT_COUNT_SOL_FILE = NewStatistics.OUT_FILE_COMP+"float_count_sol.dat";
    public static final String FLOAT_COUNT_SOL_GAP_FILE = NewStatistics.OUT_FILE_COMP+"float_count_sol_gap.dat";
    
    public static final String WEIGHTED = NewStatistics.OUT_FILE_COMP+"count_time.dat";
    //public static final String EDGE_OUT_FILE = NewStatistics.OUT_FILE_COMP+"edge_output.dat";
    //public static final String OPT_OUT_FILE = NewStatistics.OUT_FILE_COMP+"opt_output.dat";

    public static final int NULL_CONST = -2;


    private static void readUnitInput(String fileName, int[] count, double[] timeSum, double[] time, int[] solved) {
	File file = null;
	FileReader fileReader = null;
	BufferedReader bufferedReader = null;
	
	try {
	    System.out.println("nome arquivo: " + fileName);
	    file = new File(fileName);
	    fileReader = new FileReader(file);
	    bufferedReader = new BufferedReader(fileReader);

	    String line;

	    System.out.println("---------------");
	    System.out.println(fileName);
	    System.out.println("---------------");

	    double solvedTimeSum = 0;
	    double solvedTreeTimeSum = 0;
	    double solvedWPrepTimeSum = 0;
	    double solvedILPTimeSum = 0;
	    int solvedCount = 0;
	    int solvedTreeCount = 0;
	    int solvedWPrepCount = 0;
	    int solvedILPCount = 0;

	    int ind = 0;;
	    while ((line = bufferedReader.readLine()) != null) {
		System.out.println("line: "+ line);
		String[] splitArray = line.split("\\s+");
		int instance = Integer.parseInt(splitArray[0]);

		if (splitArray.length > 2) {//instance solved
		    int flag = Integer.parseInt(splitArray[1]);

		    solved[ind] = 1;
		    
		    solvedCount++;
		    double val = Double.parseDouble(splitArray[2]);
		    solvedTimeSum += val;
		    
		    if (flag != NewStatistics.NO_TREE) {
			solved[ind] = NewStatistics.FOUND_TREE;
			solvedTreeCount++;
			solvedTreeTimeSum += val;
			
			if (flag == NewStatistics.WITH_PREP) {//solved w preprocessing
			    solvedWPrepCount++;
			    solvedWPrepTimeSum += val;
			} else {
			    solvedILPCount++;
			    solvedILPTimeSum += val;
			}
		    } else {//do not admit solution
			solvedILPCount++;
			solvedILPTimeSum += val;
		    }
		    System.out.println("inst: " + instance + ", val: " + val);

		} else { //not solved
		    solved[ind] = NewStatistics.NOT_SOLVED;
		    time[ind] = NewStatistics.NOT_SOLVED;
		}

		ind++;
	    }

	    count[0] = solvedCount;
	    count[1] = solvedILPCount;
	    //count[2] = solvedWPrepCount;
	    count[2] = solvedTreeCount;
	    count[3] = solvedWPrepCount;

	    if (solvedCount > 0)
		timeSum[0] = solvedTimeSum/solvedCount;
	    else
		timeSum[0] = 0;
	    if (solvedILPCount > 0)
		timeSum[1] = solvedILPTimeSum/solvedILPCount;
	    else
		timeSum[1] = 0;
	    if (solvedTreeCount > 0)
		timeSum[2] = solvedTreeTimeSum/solvedTreeCount;
	    else
		timeSum[2] = 0;
	    if (solvedWPrepCount > 0)
		timeSum[3] = solvedWPrepTimeSum/solvedWPrepCount;
	    else
		timeSum[3] = 0;
			    
	} catch (IOException x) {
	    //result = false;
	} finally {
	    //if (file != null) { try { file.close(); } catch(Throwable t) { } }
	    if (bufferedReader != null) { try { bufferedReader.close(); } catch(Throwable t) { } }
	    if (fileReader != null) { try { fileReader.close(); } catch(Throwable t) { } }
	}
    }

    private static void calculateGapAverage(double[] intMatrix, double[] floatMatrix, int[] solved, int[] floatSolved, double[] average, int[] count) {
	double sum = average[0] = 0;
	count[0] = 0;

	double treeSum = average[1] = 0;
	count[1] = 0;
	
	for (int i = 0; i < NewStatistics.QT_INST; i++) {
	    if ((solved[i] != NewStatistics.NOT_SOLVED) && (floatSolved[i] != NewStatistics.NOT_SOLVED)) {
		count[0]++;
		sum += intMatrix[i] / floatMatrix[i];
	    }

	    if ((solved[i] == NewStatistics.FOUND_TREE) && (floatSolved[i] == NewStatistics.FOUND_TREE)) {
		count[1]++;
		treeSum += intMatrix[i] / floatMatrix[i];
	    }
	}

	if (count[0] > 0)
	    average[0] = sum / count[0];
	if (count[1] > 0)
	    average[1] = treeSum / count[1];

	//return count;
    }

    private static void readWeightedInput(String fileName, int[] count, double[] solSum, double[] sol, double[] timeSum, double[] time, int[] solved) {
	File file = null;
	FileReader fileReader = null;
	BufferedReader bufferedReader = null;
	
	try {
	    System.out.println("nome arquivo: " + fileName);
	    file = new File(fileName);
	    fileReader = new FileReader(file);
	    bufferedReader = new BufferedReader(fileReader);

	    String line;

	    System.out.println("---------------");
	    System.out.println(fileName);
	    System.out.println("---------------");

	    //double solvedSolSum = 0;
	    double solvedTreeSolSum = 0;
	    double solvedTimeSum = 0;
	    double solvedTreeTimeSum = 0;
	    //double solvedWOPrepTimeSum = 0;
	    int solvedCount = 0;
	    int solvedTreeCount = 0;
	    //int solvedWOPrepCount = 0;

	    int ind = 0;
	    
	    while ((line = bufferedReader.readLine()) != null) {
		System.out.println("line: "+ line);
		String[] splitArray = line.split("\\s+");
		int instance = Integer.parseInt(splitArray[0]);
		
		if (splitArray.length > 2) {//instance solved
		    //System.out.println(Math.ceil(Double.parseDouble(splitArray[1])));
		    int flag = (int)Math.ceil(Double.parseDouble(splitArray[1]));
		    
		    solvedCount++;
		    double val = Double.parseDouble(splitArray[2]);
		    solvedTimeSum += val;
		    time[ind] = val;
		    
		    if (flag != NewStatistics.NO_TREE) {
			solved[ind] = NewStatistics.FOUND_TREE;
			
			solvedTreeCount++;
			solvedTreeTimeSum += val;

			val = Double.parseDouble(splitArray[1]);	       	
			//solvedSolSum += val;
			sol[ind] = val;			
			solvedTreeSolSum += val;			
			
			// if (flag != 1) {//solved w/o preprocessing
			//     solvedWOPrepCount++;
			//     solvedWOPrepTimeSum += val;
			// }
		    } else {//do not admit tree
			solved[ind] = NewStatistics.NO_TREE;
			sol[ind] = NewStatistics.NO_TREE;
		    }
		    System.out.println("inst: " + instance + ", val: " + val);

		} else {
		    time[ind] = NewStatistics.NOT_SOLVED;
		    sol[ind] = NewStatistics.NOT_SOLVED;
		    solved[ind] = NewStatistics.NOT_SOLVED;
		}

		ind++;
		
	    }

	    count[0] = solvedCount;
	    count[1] = solvedTreeCount;
	    if (solvedCount > 0) {
		timeSum[0] = solvedTimeSum/solvedCount;
		//solSum[0] = solvedSolSum/solvedCount;
	    } else {
		timeSum[0] = 0;
		//solSum[0] = 0;
	    }
	    if (solvedTreeCount > 0) {
		timeSum[1] = solvedTreeTimeSum/solvedTreeCount;
		solSum[0] = solvedTreeSolSum/solvedTreeCount;
	    } else {
		timeSum[1] = 0;
		solSum[0] = 0;
	    }
	    // if (solvedWOPrepCount > 0)
	    // 	timeSum[2] = solvedWOPrepTimeSum/solvedWOPrepCount;
	    // else
	    // 	timeSum[2] = 0;
			    
	} catch (IOException x) {
	    //result = false;
	} finally {
	    //if (file != null) { try { file.close(); } catch(Throwable t) { } }
	    if (bufferedReader != null) { try { bufferedReader.close(); } catch(Throwable t) { } }
	    if (fileReader != null) { try { fileReader.close(); } catch(Throwable t) { } }
	}
    }


    private static void weightStatistics() {
	for (int k = 0; k < NewStatistics.EDGES_PERC.length; k++) {

	    for (int i = 0; i < NewStatistics.SIZES.length; i++) {

		    for (int f = 0; f < NewStatistics.STR_FACTORS.length; f++) {

			String intFileName = new String(""+NewStatistics.STR_FACTORS[f]+"-"+NewStatistics.EDGES_PERC[k]+"-"+NewStatistics.SIZES[i]+"-"+NewStatistics.EDGE_COST+"-"+NewStatistics.ALG+NewStatistics.TIME_OUT_FILE);

			File file = null;
			FileReader fileReader = null;
			BufferedReader bufferedReader = null;

			int[] count = new int[2];
			double[] timeSum = new double[2];
			double[] time = new double[NewStatistics.QT_INST];
			double[] solSum = new double[1];
			double[] sol = new double[NewStatistics.QT_INST];
			int[] solved = new int[NewStatistics.QT_INST];
			
			readWeightedInput(intFileName, count, solSum, sol, timeSum, time, solved);

			String floatFileName = new String(""+NewStatistics.STR_FACTORS[f]+"-"+NewStatistics.EDGES_PERC[k]+"-"+NewStatistics.SIZES[i]+"-"+NewStatistics.EDGE_COST+"-"+NewStatistics.FLOAT_ALG+NewStatistics.TIME_OUT_FILE);

			int[] floatCount = new int[2];
			double[] floatTimeSum = new double[2];
			double[] floatTime = new double[NewStatistics.QT_INST];
			double[] floatSolSum = new double[1];
			double[] floatSol = new double[NewStatistics.QT_INST];
			int[] floatSolved = new int[NewStatistics.QT_INST];
			
			readWeightedInput(floatFileName, floatCount, floatSolSum, floatSol, floatTimeSum, floatTime, floatSolved);
			
			if (count[0] > 0) { //at least one instance was solved
			    String outFile = new String(NewStatistics.TIME_DIR_OUT_FILE+""+NewStatistics.STR_FACTORS[f]+"-"+NewStatistics.EDGES_PERC[k]+"-"+NewStatistics.SIZES[i]+"-"+NewStatistics.EDGE_COST+NewStatistics.COUNT_TIME_FILE);
			    writeCountAndAvg(outFile, count, timeSum);			    
			}

			if (count[1] > 0) {//at least one instance admits a tree
			    String outFile = new String(NewStatistics.TIME_DIR_OUT_FILE+""+NewStatistics.STR_FACTORS[f]+"-"+NewStatistics.EDGES_PERC[k]+"-"+NewStatistics.SIZES[i]+"-"+NewStatistics.EDGE_COST+NewStatistics.COUNT_SOL_FILE);

			    int[] tempCount = new int[1];
			    tempCount[0] = count[1];
			    writeCountAndAvg(outFile, tempCount, solSum);
			}
			
			if (floatCount[0] > 0) {//at least one instance was solved
			    String outFile = new String(NewStatistics.TIME_DIR_OUT_FILE+""+NewStatistics.STR_FACTORS[f]+"-"+NewStatistics.EDGES_PERC[k]+"-"+NewStatistics.SIZES[i]+"-"+NewStatistics.EDGE_COST+NewStatistics.FLOAT_COUNT_TIME_FILE);
			    writeCountAndAvg(outFile, floatCount, floatTimeSum);
			}
			if (floatCount[1] > 0) {//at least one instance admits tree
			    String outFile = new String(NewStatistics.TIME_DIR_OUT_FILE+""+NewStatistics.STR_FACTORS[f]+"-"+NewStatistics.EDGES_PERC[k]+"-"+NewStatistics.SIZES[i]+"-"+NewStatistics.EDGE_COST+NewStatistics.FLOAT_COUNT_SOL_FILE);
			    int[] tempCount = new int[1];
			    tempCount[0] = floatCount[1];
			    writeCountAndAvg(outFile, tempCount, floatSolSum);
			}			

			double[] timeGapAvg = new double[2];
			int[] timeGapCount = new int[2];
			int[] solGapCount = new int[2];
			
			calculateGapAverage(time, floatTime, solved, floatSolved, timeGapAvg, timeGapCount);
			double[] solGapAvg = new double[2];
			calculateGapAverage(sol, floatSol, solved, floatSolved, solGapAvg, solGapCount);

			if (timeGapCount[0] > 0) {
			    String gapOutFile = new String(NewStatistics.TIME_DIR_OUT_FILE+""+NewStatistics.STR_FACTORS[f]+"-"+NewStatistics.EDGES_PERC[k]+"-"+NewStatistics.SIZES[i]+"-"+NewStatistics.EDGE_COST+NewStatistics.COUNT_TIME_GAP_FILE);
			    writeCountAndAvg(gapOutFile, timeGapCount, timeGapAvg);			    			    
			}

			if (solGapCount[0] > 0) {
			    String solGapOutFile = new String(NewStatistics.TIME_DIR_OUT_FILE+""+NewStatistics.STR_FACTORS[f]+"-"+NewStatistics.EDGES_PERC[k]+"-"+NewStatistics.SIZES[i]+"-"+NewStatistics.EDGE_COST+NewStatistics.COUNT_SOL_GAP_FILE);
			    writeCountAndAvg(solGapOutFile, solGapCount, solGapAvg);
			}
			
		    }
	    }
	}
    }
    
    
    private static void unitStatistics() {
	for (int k = 0; k < NewStatistics.EDGES_PERC.length; k++) {
	//for (int i = 0; i < NewStatistics.SIZES.length; i++) {
	//for (int f = 0; f < NewStatistics.STR_FACTORS.length; f++) {
	//for (int f = 0; f < 1; f++) {

	    for (int i = 0; i < NewStatistics.SIZES.length; i++) {
	    //for (int k = 0; k < NewStatistics.EDGES_PERC.length; k++){
	    //for (int i = 0; i < NewStatistics.SIZES.length; i++) {
	    //for (int i = 0; i < 1; i++) {

		//for (int j = 1; j <= NewStatistics.QT_INST; j++) {
		
		//for (int k = 0; k < NewStatistics.EDGES_PERC.length; k++){
		//for (int k = 0; k < 1; k++){
		    for (int f = 0; f < NewStatistics.STR_FACTORS.length; f++) {
		    //for (int j = 1; j <= NewStatistics.QT_INST; j++) {
		    //for (int j = 1; j <= 1; j++) {


			String intFileName = new String(""+NewStatistics.STR_FACTORS[f]+"-"+NewStatistics.EDGES_PERC[k]+"-"+NewStatistics.SIZES[i]+"-"+NewStatistics.EDGE_COST+"-"+NewStatistics.ALG+NewStatistics.TIME_OUT_FILE);

			System.out.println("intFileName: " + intFileName);

			File file = null;
			FileReader fileReader = null;
			BufferedReader bufferedReader = null;

			int[] count = new int[4];
			double[] timeSum = new double[4];
			double[] time = new double[NewStatistics.QT_INST];
			int[] solved = new int[NewStatistics.QT_INST];			
			readUnitInput(intFileName, count, timeSum, time, solved);

			String floatFileName = new String(""+NewStatistics.STR_FACTORS[f]+"-"+NewStatistics.EDGES_PERC[k]+"-"+NewStatistics.SIZES[i]+"-"+NewStatistics.EDGE_COST+"-"+NewStatistics.FLOAT_ALG+NewStatistics.TIME_OUT_FILE);			

			int[] floatCount = new int[4];
			double[] floatTimeSum = new double[4];
			double[] floatTime = new double[NewStatistics.QT_INST];
			int[] floatSolved = new int[NewStatistics.QT_INST];			
			readUnitInput(floatFileName, floatCount, floatTimeSum, floatTime, floatSolved);
			if (count[0] > 0) {
			    System.out.println("vai gerar os dados para int");
			    String outFile = new String(NewStatistics.TIME_DIR_OUT_FILE+""+NewStatistics.STR_FACTORS[f]+"-"+NewStatistics.EDGES_PERC[k]+"-"+NewStatistics.SIZES[i]+"-"+NewStatistics.EDGE_COST+NewStatistics.COUNT_TIME_FILE);
			    writeCountAndAvg(outFile, count, timeSum);
			}

			if (floatCount[0] > 0) {//at least one instance was solved
			    System.out.println("vai gerar os dados para float");
			    String outFile = new String(NewStatistics.TIME_DIR_OUT_FILE+""+NewStatistics.STR_FACTORS[f]+"-"+NewStatistics.EDGES_PERC[k]+"-"+NewStatistics.SIZES[i]+"-"+NewStatistics.EDGE_COST+NewStatistics.FLOAT_COUNT_TIME_FILE);
			    writeCountAndAvg(outFile, floatCount, floatTimeSum);
			}

			// double[] timeGapAvg = new double[2];
			// int[] timeGapCount = new int[2];
			// int[] solGapCount = new int[2];

			// System.out.println("vai gerar os dados do gap");
			// calculateGapAverage(time, floatTime, solved, floatSolved, timeGapAvg, timeGapCount);

			// if (timeGapCount[0] > 0) {
			//     String gapOutFile = new String(NewStatistics.TIME_DIR_OUT_FILE+""+NewStatistics.STR_FACTORS[f]+"-"+NewStatistics.EDGES_PERC[k]+"-"+NewStatistics.SIZES[i]+"-"+NewStatistics.EDGE_COST+NewStatistics.COUNT_TIME_GAP_FILE);
			//     writeCountAndAvg(gapOutFile, timeGapCount, timeGapAvg);
			// }			
		    }
	    }
	}
    }

    private static void writeCountAndAvg(String fileName, int[] count, double[] valAvg) {
	BufferedWriter bw = null;
	FileWriter fw = null;

	System.out.println("dentro de writeCountAndAvg");
	try {
	    if (count[0] > 0) {
		fw = new FileWriter(fileName);
		bw = new BufferedWriter(fw);
		//sum = sum / count;

		// if (count.length == 1) //for solution vector, we only consider instances which admit tree
		bw.write(String.valueOf(valAvg[0] + " " + count[0]));
		if (count.length >= 2)//count.length == 2
		    bw.write(String.valueOf(" " + valAvg[1] + " " + count[1]));
		if (count.length >= 3)
		    bw.write(String.valueOf(" " + valAvg[2] + " " + count[2]));
		if (count.length == 4)
		    bw.write(String.valueOf(" " + valAvg[3] + " " + count[3]));

		//bw.newLine();
		//bw.write("teste");
		System.out.println("escreveu no arquivo");
	    }
	} catch (IOException x) {
	} finally {
	    if (bw != null) { try { bw.close(); } catch(Throwable t) { } }
	    if (fw != null) { try { fw.close(); } catch(Throwable t) { } }	    
	}
    }
    
    public static void main(String[] args) {
	NewStatistics.weightStatistics();
	//NewStatistics.unitStatistics();
    }
}
