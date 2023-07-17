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

public class CheckAllFiles {

    public static final int[] SIZES = {10, 20, 30, 40, 48};
    public static final int[] STR_FACTORS = {3, 4, 5, 6};
    public static final int[] EDGES_PERC = {20, 40, 60, 80};    
    public static final int QT_INST = 10;

    public static void main(String[] args) {

	// System.out.println(args[0]);
	//System.out.println(args[1]);
	String dirName = args[0];
	String fileName = args[1];

	for (int f = 0; f < CheckAllFiles.STR_FACTORS.length; f++) {
	//for (int k = 0; k < CheckAllFiles.EDGES_PERC.length; k++) {
	//for (int i = 0; i < CheckAllFiles.SIZES.length; i++) {
	//for (int f = 0; f < CheckAllFiles.STR_FACTORS.length; f++) {
	//for (int f = 0; f < 1; f++) {

	    for (int k = 0; k < CheckAllFiles.EDGES_PERC.length; k++) {
	    //for (int i = 0; i < CheckAllFiles.SIZES.length; i++) {
	    //for (int k = 0; k < CheckAllFiles.EDGES_PERC.length; k++){
	    //for (int i = 0; i < CheckAllFiles.SIZES.length; i++) {
	    //for (int i = 0; i < 1; i++) {

		//for (int k = 0; k < 1; k++){
		    for (int i = 0; i < CheckAllFiles.SIZES.length; i++) {
		    //for (int f = 0; f < CheckAllFiles.STR_FACTORS.length; f++) {
			
			String timeOutFile = new String(""+CheckAllFiles.STR_FACTORS[f]+"-"+CheckAllFiles.EDGES_PERC[k]+"-"+CheckAllFiles.SIZES[i]+"-"+fileName);



			File file = null;
			FileReader fileReader = null;
			BufferedReader bufferedReader = null;

			BufferedWriter bw = null;
			FileWriter fw = null;	
			try {
			    //System.out.println(dirName + timeOutFile);
			    file = new File(dirName + timeOutFile);
			    fileReader = new FileReader(file);
			    bufferedReader = new BufferedReader(fileReader);

			    String line;
	    
			    int instance = 1;
			    //for (line = bufferedReader.readLine(); line != null; line = bufferedReader.readLine()) {
			    while ((line = bufferedReader.readLine()) != null) {
			    //for (int a = 0; a < CheckAllFiles.QT_INST; a++) {
				//line = bufferedReader.readLine();
				//System.out.println(line);
				String[] splitArray = line.split("\\s+");

				//System.out.println("array[0]: " + splitArray[0] + ", array[1]: " + splitArray[1]);
				int val = Integer.parseInt(splitArray[0]);
				if (val != instance) {
				    System.out.println("Problema com a instância "+instance);
				    System.out.println(timeOutFile);
				    System.exit(0);
				}
				if (instance < CheckAllFiles.QT_INST)
				    instance++;
			    }

			    bufferedReader.close();
	    
			} catch (IOException e) {
			    //System.out.println(e.getMessage());
			    System.out.println("arquivo: " + timeOutFile);
			    
			    if (bufferedReader != null) { try { bufferedReader.close(); } catch(Throwable t) { } }
			    if (fileReader != null) { try { fileReader.close(); } catch(Throwable t) { } }

			    
			    System.exit(0);
			} finally {
			    if (bufferedReader != null) { try { bufferedReader.close(); } catch(Throwable t) { } }
			    if (fileReader != null) { try { fileReader.close(); } catch(Throwable t) { } }
			    //if (file != null) { try { file.close(); } catch(Throwable t) { } }
			}






			
		    }	    
		    	
	    }	    
	}
	


	
    }
}
