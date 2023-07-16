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

public class MergeData {
    
    public static final int QT_INST = 10;
    public static final int[] SIZES = {20, 30, 40, 50, 60};
    public static final int[] STR_FACTORS = {3, 4, 5, 6};
    public static final int[] EDGES_PERC = {20, 40, 60, 80};
    public static final String EDGE_COST = "unit";
    public static final String IN_FILE_DIR = "saida_unit/";
    public static final String OUT_FILE_DIR = "single_file/";


    public static final int NO_DATA = -1;
    public static final String NO_DATA_STR = "-";
    
    public static final String COUNT_TIME_FILE = "count_time.dat";
    public static final String FLOAT_COUNT_TIME_FILE = "float_count_time.dat";
    public static final String COUNT_SOL_GAP_FILE = "count_sol_gap.dat";
    public static final String FINAL_FILE = "all_data.dat";
    

    /* 
count[0] = # solved 
count[1] = # tree solved
count[2] = # relaxed solved 
count[3] = # relaxed tree solved
count[4] = # instances integrality gap
count[5] = # instances admit tree integrality gap
     */
    private static void readWeightedStatistics(String countTimeFileName, String floatCountTimeFileName, String countSolGapFileName, String[] count, String[] value) {
	File file = null;
	FileReader fileReader = null;
	BufferedReader bufferedReader = null;
	
	try {
	    
	    file = new File(countTimeFileName);
	    if (file.exists()) {
	    
		fileReader = new FileReader(file);
		bufferedReader = new BufferedReader(fileReader);

		String line = bufferedReader.readLine();
		String[] splitArray = line.split("\\s+");
		count[0] = splitArray[1];
		if (Integer.parseInt(count[0]) == 0)
		    value[0] = MergeData.NO_DATA_STR;
		else
		    value[0] = splitArray[0];
		
		count[1] = splitArray[3];
		if (Integer.parseInt(count[1]) == 0)
		    value[1] = MergeData.NO_DATA_STR;
		else
		    value[1] = splitArray[2];
		
		bufferedReader.close();
		fileReader.close();
	    } else {
		count[0] = MergeData.NO_DATA_STR;
		value[0] = MergeData.NO_DATA_STR;
		count[1] = MergeData.NO_DATA_STR;
		value[1] = MergeData.NO_DATA_STR;
	    }

	    file = new File(floatCountTimeFileName);
	    if (file.exists()) {
		fileReader = new FileReader(file);
		bufferedReader = new BufferedReader(fileReader);

		String line = bufferedReader.readLine();
		String[] splitArray = line.split("\\s+");

		count[2] = splitArray[1];
		if (Integer.parseInt(count[2]) == 0)
		    value[0] = MergeData.NO_DATA_STR;
		else		
		    value[2] = splitArray[0];
		
		count[3] = splitArray[3];
		if (Integer.parseInt(count[3]) == 0)
		    value[3] = MergeData.NO_DATA_STR;
		else
		    value[3] = splitArray[2];
		
		bufferedReader.close();
		fileReader.close();
	    } else {
		count[2] = MergeData.NO_DATA_STR;
		value[2] = MergeData.NO_DATA_STR;
		count[3] = MergeData.NO_DATA_STR;
		value[3] = MergeData.NO_DATA_STR;		
	    }

	    file = new File(countSolGapFileName);
	    if (file.exists()) {		
		fileReader = new FileReader(file);
		bufferedReader = new BufferedReader(fileReader);

		String line = bufferedReader.readLine();
		String[] splitArray = line.split("\\s+");
		count[4] = splitArray[1];
		if (Integer.parseInt(count[4]) == 0)
		    value[4] = MergeData.NO_DATA_STR;
		else				
		    value[4] = splitArray[0];
		count[5] = splitArray[3];
		if (Integer.parseInt(count[5]) == 0)
		    value[5] = MergeData.NO_DATA_STR;
		else				
		    value[5] = splitArray[2];
	    } else {
		count[4] = MergeData.NO_DATA_STR;		
		value[4] = MergeData.NO_DATA_STR;
		count[5] = MergeData.NO_DATA_STR;
		value[5] = MergeData.NO_DATA_STR;
	    }
	    
	} catch (IOException x) {
	    //result = false;
	} finally {
	    //if (file != null) { try { file.close(); } catch(Throwable t) { } }
	    if (bufferedReader != null) { try { bufferedReader.close(); } catch(Throwable t) { } }
	    if (fileReader != null) { try { fileReader.close(); } catch(Throwable t) { } }
	}
    }


    /* 
count[0] = # solved 
count[1] = # tree solved
count[2] = # relaxed solved 
count[3] = # relaxed tree solved
     */
    private static void readUnitStatistics(String countTimeFileName, String floatCountTimeFileName, String[] count, String[] value) {
	File file = null;
	FileReader fileReader = null;
	BufferedReader bufferedReader = null;
	
	try {
	    
	    file = new File(countTimeFileName);
	    if (file.exists()) {
	    
		fileReader = new FileReader(file);
		bufferedReader = new BufferedReader(fileReader);

		String line = bufferedReader.readLine();
		String[] splitArray = line.split("\\s+");
		count[0] = splitArray[1];
		if (Integer.parseInt(count[0]) == 0)
		    value[0] = MergeData.NO_DATA_STR;
		else
		    value[0] = splitArray[0];
		
		count[1] = splitArray[5];
		if (Integer.parseInt(count[1]) == 0)
		    value[1] = MergeData.NO_DATA_STR;
		else
		    value[1] = splitArray[4];
		
		bufferedReader.close();
		fileReader.close();
	    } else {
		count[0] = MergeData.NO_DATA_STR;
		value[0] = MergeData.NO_DATA_STR;
		count[1] = MergeData.NO_DATA_STR;
		value[1] = MergeData.NO_DATA_STR;
	    }

	    file = new File(floatCountTimeFileName);
	    if (file.exists()) {
		fileReader = new FileReader(file);
		bufferedReader = new BufferedReader(fileReader);

		String line = bufferedReader.readLine();
		String[] splitArray = line.split("\\s+");

		count[2] = splitArray[1];
		if (Integer.parseInt(count[2]) == 0)
		    value[0] = MergeData.NO_DATA_STR;
		else		
		    value[2] = splitArray[0];
		
		count[3] = splitArray[5];
		if (Integer.parseInt(count[3]) == 0)
		    value[3] = MergeData.NO_DATA_STR;
		else
		    value[3] = splitArray[4];
		
		bufferedReader.close();
		fileReader.close();
	    } else {
		count[2] = MergeData.NO_DATA_STR;
		value[2] = MergeData.NO_DATA_STR;
		count[3] = MergeData.NO_DATA_STR;
		value[3] = MergeData.NO_DATA_STR;		
	    }
	    
	} catch (IOException x) {
	    //result = false;
	} finally {
	    //if (file != null) { try { file.close(); } catch(Throwable t) { } }
	    if (bufferedReader != null) { try { bufferedReader.close(); } catch(Throwable t) { } }
	    if (fileReader != null) { try { fileReader.close(); } catch(Throwable t) { } }
	}
    }


    
    private static void mergeUnitData() {
	for (int f = 0; f < MergeData.STR_FACTORS.length; f++) {
	//for (int k = 0; k < MergeData.EDGES_PERC.length; k++) {

	    for (int k = 0; k < MergeData.EDGES_PERC.length; k++) {
	    //for (int i = 0; i < MergeData.SIZES.length; i++) {

		    for (int i = 0; i < MergeData.SIZES.length; i++) {
		    //for (int f = 0; f < MergeData.STR_FACTORS.length; f++) {

			String countTimeFileName = new String(""+MergeData.IN_FILE_DIR + MergeData.STR_FACTORS[f]+"-"+MergeData.EDGES_PERC[k]+"-"+MergeData.SIZES[i]+"-"+MergeData.EDGE_COST+"-"+MergeData.COUNT_TIME_FILE);
			System.out.println(countTimeFileName);
			String floatCountTimeFileName = new String(""+MergeData.IN_FILE_DIR + MergeData.STR_FACTORS[f]+"-"+MergeData.EDGES_PERC[k]+"-"+MergeData.SIZES[i]+"-"+MergeData.EDGE_COST+"-"+MergeData.FLOAT_COUNT_TIME_FILE);
			System.out.println(floatCountTimeFileName);

			String[] count = new String[4];
			String[] value = new String[4];
			
			readUnitStatistics(countTimeFileName, floatCountTimeFileName, count, value);

			String allDataFileName = new String(""+MergeData.OUT_FILE_DIR + MergeData.EDGE_COST + "-" + MergeData.FINAL_FILE);
			writeDataToSingleFile(MergeData.STR_FACTORS[f], MergeData.SIZES[i], MergeData.EDGES_PERC[k], allDataFileName, count, value);
		    }
	    }
	}
    }

    

    private static void mergeWeightedData() {
	for (int f = 0; f < MergeData.STR_FACTORS.length; f++) {
	//for (int k = 0; k < MergeData.EDGES_PERC.length; k++) {

	    for (int k = 0; k < MergeData.EDGES_PERC.length; k++) {
	    //for (int i = 0; i < MergeData.SIZES.length; i++) {

		    for (int i = 0; i < MergeData.SIZES.length; i++) {
		    //for (int f = 0; f < MergeData.STR_FACTORS.length; f++) {

			String countTimeFileName = new String(""+MergeData.IN_FILE_DIR + MergeData.STR_FACTORS[f]+"-"+MergeData.EDGES_PERC[k]+"-"+MergeData.SIZES[i]+"-"+MergeData.EDGE_COST+"-"+MergeData.COUNT_TIME_FILE);
			System.out.println(countTimeFileName);
			String floatCountTimeFileName = new String(""+MergeData.IN_FILE_DIR + MergeData.STR_FACTORS[f]+"-"+MergeData.EDGES_PERC[k]+"-"+MergeData.SIZES[i]+"-"+MergeData.EDGE_COST+"-"+MergeData.FLOAT_COUNT_TIME_FILE);
			System.out.println(floatCountTimeFileName);
			String countSolGapFileName = new String(""+MergeData.IN_FILE_DIR + MergeData.STR_FACTORS[f] + "-" + MergeData.EDGES_PERC[k]+"-"+MergeData.SIZES[i]+"-"+MergeData.EDGE_COST+"-"+MergeData.COUNT_SOL_GAP_FILE);
			System.out.println(countSolGapFileName);

			String[] count = new String[6];
			String[] value = new String[6];
			
			readWeightedStatistics(countTimeFileName, floatCountTimeFileName, countSolGapFileName, count, value);

			String allDataFileName = new String(""+MergeData.OUT_FILE_DIR + MergeData.EDGE_COST + "-" + MergeData.FINAL_FILE);
			writeDataToSingleFile(MergeData.STR_FACTORS[f], MergeData.SIZES[i], MergeData.EDGES_PERC[k], allDataFileName, count, value);
		    }
	    }
	}
    }
        
    private static void writeDataToSingleFile(int factor, int size, int dens, String fileName, String[] count, String[] value) {
	BufferedWriter bw = null;
	FileWriter fw = null;	

	//System.out.println("dentro de writeDataToSingleFile");
	try {
	    //if (count[0] > 0) {
	    fw = new FileWriter(fileName, true);
	    bw = new BufferedWriter(fw);
	    //sum = sum / count;

	    if (count.length == 4) {//unit
		bw.write(String.valueOf(factor + " " + size + " " + dens + " " + count[0] + " " + count[2] + " " + value[0] + " " + value[2] + " " + count[1] + " " + count[3] + " " + value[1] + " " + value[3]));
	    } else //weight
		bw.write(String.valueOf(factor + " " + size + " " + dens + " " + count[0] + " " + count[2] + " " + value[0] + " " + value[2] + " " + count[1] + " " + count[3] + " " + value[1] + " " + value[3] + " " + value[5]));
	    bw.newLine();
		//}
	} catch (IOException x) {
	} finally {
	    if (bw != null) { try { bw.close(); } catch(Throwable t) { } }
	    if (fw != null) { try { fw.close(); } catch(Throwable t) { } }	    
	}
    }
    
    public static void main(String[] args) {
	//MergeData.mergeWeightedData();
	MergeData.mergeUnitData();
    }
}
