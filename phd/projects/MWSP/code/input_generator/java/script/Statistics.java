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

public class Statistics {
    public static final int QT_INST = 10;
    public static final int[] SIZES = {10, 20, 30, 40, 48};
    public static final int[] STR_FACTORS = {2, 3, 4, 5, 6};
    public static final int[] EDGES_PERC = {20, 40, 60, 80};
    public static final String EDGE_COST = "euclidean";
    public static final String ALG_FLOAT = "tree_labels_dist_spanner_float";
    public static final String ALG_PARTIAL_FLOAT = "tree_labels_dist_spanner_partialfloat";
    public static final String ALG = "tree_labels_dist_spanner-opt";
    public static final String OUT_FILtffffffffffffffff E_COMP = "-";
    public static final String TIME_OUT_FILE = Statistics.OUT_FILE_COMP+"time_output.dat";
    public static final String FLOAT_INT_TIME_RAT = Statistics.OUT_FILE_COMP+"float_int_time_rat_output.dat";
    public static final String PARFLOAT_INT_TIME_RAT = Statistics.OUT_FILE_COMP+"parfloat_int_time_rat_output.dat";
    public static final String FLOAT_PARFLOAT_TIME_RAT = Statistics.OUT_FILE_COMP+"float_parfloat_time_rat_output.dat";
    public static final String FLOAT_PARFLOAT_VAL_RAT = Statistics.OUT_FILE_COMP+"float_parfloat_value_rat_output.dat";
    public static final String FLOAT_INT_VAL_RAT = Statistics.OUT_FILE_COMP+"float_int_value_rat_output.dat";
    public static final String PARFLOAT_INT_VAL_RAT = Statistics.OUT_FILE_COMP+"parfloat_int_value_rat_output.dat";
    //public static final String EDGE_OUT_FILE = Statistics.OUT_FILE_COMP+"edge_output.dat";
    //public static final String OPT_OUT_FILE = Statistics.OUT_FILE_COMP+"opt_output.dat";

    public static final int NULL_CONST = -2;


    private static boolean readUnitInput(String fileName, double[] count, double[] time) {
	File file = null;
	FileReader fileReader = null;
	BufferedReader bufferedReader = null;

	boolean result = false;
	
	try {
	    file = new File(fileName);
	    fileReader = new FileReader(file);
	    bufferedReader = new BufferedReader(fileReader);

	    String line;

	    System.out.println("---------------");
	    System.out.println(fileName);
	    System.out.println("---------------");

	    double solvedTimeSum = 0;
	    double solvedTreeTimeSum = 0;
	    int solvedCount = 0;
	    int solvedTreeCount = 0;
	    while ((line = bufferedReader.readLine()) != null) {
		System.out.println("line: "+ line);
		String[] splitArray = line.split("\\s+");
		int instance = Integer.parseInt(splitArray[0]);

		int solved = Integer.parseInt(splitArray[1]);
		if (solved != 0) {
		    solvedCount++;
		    double val = Double.parseDouble(splitArray[2]);
		    solvedTimeSum += val;
		    if (solved != -1) {
			solvedTreeCount++;
			solvdTreeTimeSum += val;
		    }
		    //System.out.println("inst: " + instance + ", val: " + value[instance] + ", time: " + time[instance]);

		}
	    }

	    count[0] = solvedCount;
	    count[1] = solvedTreeCount;
	    time[0] = solvedTimeSum;
	    time[1] = solvedTreeTimeSum;
			    
	} catch (IOException x) {
	    //result = false;
	} finally {
	    //if (file != null) { try { file.close(); } catch(Throwable t) { } }
	    if (bufferedReader != null) { try { bufferedReader.close(); } catch(Throwable t) { } }
	    if (fileReader != null) { try { fileReader.close(); } catch(Throwable t) { } }
	}

	return result;
    }

    private static void unitStatistics() {
	for (int k = 0; k < Statistics.EDGES_PERC.length; k++) {
	//for (int i = 0; i < Statistics.SIZES.length; i++) {
	//for (int f = 0; f < Statistics.STR_FACTORS.length; f++) {
	//for (int f = 0; f < 1; f++) {

	    for (int i = 0; i < Statistics.SIZES.length; i++) {
	    //for (int k = 0; k < Statistics.EDGES_PERC.length; k++){
	    //for (int i = 0; i < Statistics.SIZES.length; i++) {
	    //for (int i = 0; i < 1; i++) {

		//for (int j = 1; j <= Statistics.QT_INST; j++) {
		
		//for (int k = 0; k < Statistics.EDGES_PERC.length; k++){
		//for (int k = 0; k < 1; k++){
		    for (int f = 0; f < Statistics.STR_FACTORS.length; f++) {
		    //for (int j = 1; j <= Statistics.QT_INST; j++) {
		    //for (int j = 1; j <= 1; j++) {


			String intFileName = new String(""+Statistics.STR_FACTORS[f]+"-"+Statistics.EDGES_PERC[k]+"-"+Statistics.SIZES[i]+"-"+Statistics.EDGE_COST+"-"+Statistics.ALG+Statistics.TIME_OUT_FILE);

			File file = null;
			FileReader fileReader = null;
			BufferedReader bufferedReader = null;

			int[] count = new int[2];
			double[] timeSum = new double[2];
			boolean intRet = readUnitInput(intFileName, count, timeSum);
			if (count[0]) {
			    String valueFile = new String(""+Statistics.STR_FACTORS[f]+"-"+Statistics.EDGES_PERC[k]+"-"+Statistics.SIZES[i]+"-"+Statistics.EDGE_COST+"-"+Statistics.PARFLOAT_INT_VAL_RAT);
			    writeRatio(valueFile, parFloatValue, intValue);			    
			}
			
			if (intRet && parFloatRet) {
			    //System.out.println("********************");
			    //System.out.println("dentro de int e parFloat");
			    String valueFile = new String(""+Statistics.STR_FACTORS[f]+"-"+Statistics.EDGES_PERC[k]+"-"+Statistics.SIZES[i]+"-"+Statistics.EDGE_COST+"-"+Statistics.PARFLOAT_INT_VAL_RAT);
			    writeRatio(valueFile, parFloatValue, intValue);
			    //System.out.println("********************");

			    String timeFile = new String(""+Statistics.STR_FACTORS[f]+"-"+Statistics.EDGES_PERC[k]+"-"+Statistics.SIZES[i]+"-"+Statistics.EDGE_COST+"-"+Statistics.PARFLOAT_INT_TIME_RAT);
			    writeRatio(timeFile, parFloatTime, intTime);
			}

			if (intRet && floatRet) {
			    if ((Statistics.STR_FACTORS[f] == 2) &&
				(Statistics.EDGES_PERC[k] == 20) &&
				(Statistics.SIZES[i] == 10)
				) {
				System.out.println("********************");
				System.out.println("dentro de int e float");
				
			    }
			    String valueFile = new String(""+Statistics.STR_FACTORS[f]+"-"+Statistics.EDGES_PERC[k]+"-"+Statistics.SIZES[i]+"-"+Statistics.EDGE_COST+"-"+Statistics.FLOAT_INT_VAL_RAT);
			    writeRatio(valueFile, floatValue, intValue);

			    System.out.println("vai calcular a razão do tempo");
			    String timeFile = new String(""+Statistics.STR_FACTORS[f]+"-"+Statistics.EDGES_PERC[k]+"-"+Statistics.SIZES[i]+"-"+Statistics.EDGE_COST+"-"+Statistics.FLOAT_INT_TIME_RAT);
			    writeRatio(timeFile, floatTime, intTime);


			    if ((Statistics.STR_FACTORS[f] == 2) &&
				(Statistics.EDGES_PERC[k] == 20) &&
				(Statistics.SIZES[i] == 10)
				) {
				System.out.println("********************");
				
			    }			    
			}

			if (parFloatRet && floatRet) {
			    String valueFile = new String(""+Statistics.STR_FACTORS[f]+"-"+Statistics.EDGES_PERC[k]+"-"+Statistics.SIZES[i]+"-"+Statistics.EDGE_COST+"-"+Statistics.FLOAT_PARFLOAT_VAL_RAT);
			    writeRatio(valueFile, floatValue, parFloatValue);

			    String timeFile = new String(""+Statistics.STR_FACTORS[f]+"-"+Statistics.EDGES_PERC[k]+"-"+Statistics.SIZES[i]+"-"+Statistics.EDGE_COST+"-"+Statistics.FLOAT_PARFLOAT_TIME_RAT);
			    writeRatio(timeFile, floatTime, parFloatTime);
			}
			
		    }	    
		    
		    //} j		
	    }	    
	}
    }

    private static void writeCountAndTime(String fileName, int[] count, double[] timeSum) {
	BufferedWriter bw = null;
	FileWriter fw = null;

	try {
	    double sum  = 0;
	    int count = 0;

	    for (int i = 1; i <= Statistics.QT_INST; i++) {
		if ((num[i] != Statistics.NULL_CONST) &&
		    (denom[i] != Statistics.NULL_CONST)) {
		    if ((denom[i] != 0) &&
			((num[i] != -1) && (denom[i] != -1))
			) {
			System.out.println("num: " + num[i] + ", denom: " + denom[i]);
			count++;
			sum += num[i] / denom[i];
		    }
		}
	    }

	    if (count > 0) {
		fw = new FileWriter(fileName);
		bw = new BufferedWriter(fw);
		sum = sum / count;

		bw.write(String.valueOf(sum + " " + count));
		//bw.newLine();
		//bw.write("teste");
	    }
	} catch (IOException x) {
	} finally {
	    if (bw != null) { try { bw.close(); } catch(Throwable t) { } }
	    if (fw != null) { try { fw.close(); } catch(Throwable t) { } }	    
	}
    }


    private static boolean readInput(String fileName, double[] value, double[] time) {
	File file = null;
	FileReader fileReader = null;
	BufferedReader bufferedReader = null;

	boolean result = false;
	
	try {
	    file = new File(fileName);
	    fileReader = new FileReader(file);
	    bufferedReader = new BufferedReader(fileReader);

	    String line;

	    System.out.println("---------------");
	    System.out.println(fileName);
	    System.out.println("---------------");
	    while ((line = bufferedReader.readLine()) != null) {
		System.out.println("line: "+ line);
		String[] splitArray = line.split("\\s+");
		int instance = Integer.parseInt(splitArray[0]);

		double val = Double.parseDouble(splitArray[1]);
		if (val != 0) {
		    result = true;
		    value[instance] = val;
		    time[instance] = Double.parseDouble(splitArray[2]);
		    System.out.println("inst: " + instance + ", val: " + value[instance] + ", time: " + time[instance]);

		}
	    }
			    
	} catch (IOException x) {
	    //result = false;
	} finally {
	    //if (file != null) { try { file.close(); } catch(Throwable t) { } }
	    if (bufferedReader != null) { try { bufferedReader.close(); } catch(Throwable t) { } }
	    if (fileReader != null) { try { fileReader.close(); } catch(Throwable t) { } }
	}

	return result;
    }

    private static void writeRatio(String fileName, double[] num, double[] denom) {
	BufferedWriter bw = null;
	FileWriter fw = null;

	try {
	    double sum  = 0;
	    int count = 0;

	    for (int i = 1; i <= Statistics.QT_INST; i++) {
		if ((num[i] != Statistics.NULL_CONST) &&
		    (denom[i] != Statistics.NULL_CONST)) {
		    if ((denom[i] != 0) &&
			((num[i] != -1) && (denom[i] != -1))
			) {
			System.out.println("num: " + num[i] + ", denom: " + denom[i]);
			count++;
			sum += num[i] / denom[i];
		    }
		}
	    }

	    if (count > 0) {
		fw = new FileWriter(fileName);
		bw = new BufferedWriter(fw);
		sum = sum / count;

		bw.write(String.valueOf(sum + " " + count));
		//bw.newLine();
		//bw.write("teste");
	    }
	} catch (IOException x) {
	} finally {
	    if (bw != null) { try { bw.close(); } catch(Throwable t) { } }
	    if (fw != null) { try { fw.close(); } catch(Throwable t) { } }	    
	}
    }
    
    public static void main(String[] args) {
	
	for (int k = 0; k < Statistics.EDGES_PERC.length; k++) {
	//for (int i = 0; i < Statistics.SIZES.length; i++) {
	//for (int f = 0; f < Statistics.STR_FACTORS.length; f++) {
	//for (int f = 0; f < 1; f++) {

	    for (int i = 0; i < Statistics.SIZES.length; i++) {
	    //for (int k = 0; k < Statistics.EDGES_PERC.length; k++){
	    //for (int i = 0; i < Statistics.SIZES.length; i++) {
	    //for (int i = 0; i < 1; i++) {

		//for (int j = 1; j <= Statistics.QT_INST; j++) {
		
		//for (int k = 0; k < Statistics.EDGES_PERC.length; k++){
		//for (int k = 0; k < 1; k++){
		    for (int f = 0; f < Statistics.STR_FACTORS.length; f++) {
		    //for (int j = 1; j <= Statistics.QT_INST; j++) {
		    //for (int j = 1; j <= 1; j++) {


			String intFileName = new String(""+Statistics.STR_FACTORS[f]+"-"+Statistics.EDGES_PERC[k]+"-"+Statistics.SIZES[i]+"-"+Statistics.EDGE_COST+"-"+Statistics.ALG+Statistics.TIME_OUT_FILE);
			String floatFileName = new String(""+Statistics.STR_FACTORS[f]+"-"+Statistics.EDGES_PERC[k]+"-"+Statistics.SIZES[i]+"-"+Statistics.EDGE_COST+"-"+Statistics.ALG_FLOAT+Statistics.TIME_OUT_FILE);
			String partialFloatFileName = new String(""+Statistics.STR_FACTORS[f]+"-"+Statistics.EDGES_PERC[k]+"-"+Statistics.SIZES[i]+"-"+Statistics.EDGE_COST+"-"+Statistics.ALG_PARTIAL_FLOAT+Statistics.TIME_OUT_FILE);			
			

			File file = null;
			FileReader fileReader = null;
			BufferedReader bufferedReader = null;

			int[] count = new int[2];
			double[] timeSum = new double[2];
			boolean intRet = readUnitInput(intFileName, count, timeSum);
			

			double[] intTime = new double[Statistics.QT_INST+1];
			java.util.Arrays.fill(intTime, Statistics.NULL_CONST);
			double[] intValue = new double[Statistics.QT_INST+1];
			java.util.Arrays.fill(intValue, Statistics.NULL_CONST);

			double[] parFloatTime = new double[Statistics.QT_INST+1];
			java.util.Arrays.fill(parFloatTime, Statistics.NULL_CONST);
			double[] parFloatValue = new double[Statistics.QT_INST+1];
			java.util.Arrays.fill(parFloatValue, Statistics.NULL_CONST);

			double[] floatTime = new double[Statistics.QT_INST+1];
			java.util.Arrays.fill(floatTime, Statistics.NULL_CONST);
			double[] floatValue = new double[Statistics.QT_INST+1];
			java.util.Arrays.fill(floatValue, Statistics.NULL_CONST);

			boolean intRet = readInput(intFileName, intValue, intTime);

			boolean parFloatRet = readInput(partialFloatFileName, parFloatValue, parFloatTime);
			boolean floatRet = readInput(floatFileName, floatValue, floatTime);
			if (intRet && parFloatRet) {
			    //System.out.println("********************");
			    //System.out.println("dentro de int e parFloat");
			    String valueFile = new String(""+Statistics.STR_FACTORS[f]+"-"+Statistics.EDGES_PERC[k]+"-"+Statistics.SIZES[i]+"-"+Statistics.EDGE_COST+"-"+Statistics.PARFLOAT_INT_VAL_RAT);
			    writeRatio(valueFile, parFloatValue, intValue);
			    //System.out.println("********************");

			    String timeFile = new String(""+Statistics.STR_FACTORS[f]+"-"+Statistics.EDGES_PERC[k]+"-"+Statistics.SIZES[i]+"-"+Statistics.EDGE_COST+"-"+Statistics.PARFLOAT_INT_TIME_RAT);
			    writeRatio(timeFile, parFloatTime, intTime);
			}

			if (intRet && floatRet) {
			    if ((Statistics.STR_FACTORS[f] == 2) &&
				(Statistics.EDGES_PERC[k] == 20) &&
				(Statistics.SIZES[i] == 10)
				) {
				System.out.println("********************");
				System.out.println("dentro de int e float");
				
			    }
			    String valueFile = new String(""+Statistics.STR_FACTORS[f]+"-"+Statistics.EDGES_PERC[k]+"-"+Statistics.SIZES[i]+"-"+Statistics.EDGE_COST+"-"+Statistics.FLOAT_INT_VAL_RAT);
			    writeRatio(valueFile, floatValue, intValue);

			    System.out.println("vai calcular a razão do tempo");
			    String timeFile = new String(""+Statistics.STR_FACTORS[f]+"-"+Statistics.EDGES_PERC[k]+"-"+Statistics.SIZES[i]+"-"+Statistics.EDGE_COST+"-"+Statistics.FLOAT_INT_TIME_RAT);
			    writeRatio(timeFile, floatTime, intTime);


			    if ((Statistics.STR_FACTORS[f] == 2) &&
				(Statistics.EDGES_PERC[k] == 20) &&
				(Statistics.SIZES[i] == 10)
				) {
				System.out.println("********************");
				
			    }			    
			}

			if (parFloatRet && floatRet) {
			    String valueFile = new String(""+Statistics.STR_FACTORS[f]+"-"+Statistics.EDGES_PERC[k]+"-"+Statistics.SIZES[i]+"-"+Statistics.EDGE_COST+"-"+Statistics.FLOAT_PARFLOAT_VAL_RAT);
			    writeRatio(valueFile, floatValue, parFloatValue);

			    String timeFile = new String(""+Statistics.STR_FACTORS[f]+"-"+Statistics.EDGES_PERC[k]+"-"+Statistics.SIZES[i]+"-"+Statistics.EDGE_COST+"-"+Statistics.FLOAT_PARFLOAT_TIME_RAT);
			    writeRatio(timeFile, floatTime, parFloatTime);
			}
			
		    }	    
		    
		    //} j		
	    }	    
	}
    }
}
