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

public class ConvertCoord2Dist {
    public static final int NUMBER_NODES = 131;
    public static final String DIR = "/home/hugo/Dropbox/USP/git/doutorado/projetos/MWSP/implementacao/gerar_entrada/scilab/input/";
    public static final String IN_FILE = "131-coord-vlsi.dat";
    public static final String OUT_FILE = "131-dist-vlsi.dat";

    public static  double[][] distance(double[] x, double[] y) {
        double diff_square_sum = 0.0;
	double dist[][] = new double[x.length][x.length];
        for (int i = 0; i < x.length; i++) {
	    dist[i][i] = 0;
	    for (int j = (i+1); j < x.length; j++) {
		dist[i][j] = Math.sqrt(Math.pow(x[j] - x[i],2) + Math.pow(y[j] - y[i],2));
		dist[j][i] = dist[i][j];
	    }
        }
        return dist;
    }

    public static int numberOfEdges(int n) {
	return (n*(n-1))/2;
    }
    
    public static void main(String[] args) {
	File file = null;
	FileReader fileReader = null;
	BufferedReader bufferedReader = null;

	BufferedWriter bw = null;
	FileWriter fw = null;	
	try {
	    //System.out.println(ConvertCoord2Dist.DIR + ConvertCoord2Dist.IN_FILE);
	    file = new File(ConvertCoord2Dist.DIR + ConvertCoord2Dist.IN_FILE);
	    fileReader = new FileReader(file);
	    bufferedReader = new BufferedReader(fileReader);

	    String line;

	    double x[] = new double[ConvertCoord2Dist.NUMBER_NODES];
	    double y[] = new double[ConvertCoord2Dist.NUMBER_NODES];
	    for (int i = 0; i < ConvertCoord2Dist.NUMBER_NODES; i++) {
		line = bufferedReader.readLine();
		//System.out.println(line);
		String[] splitArray = line.split("\\s+");
		x[i] = Double.parseDouble(splitArray[1]);
		y[i] = Double.parseDouble(splitArray[2]);
	    }

	    bufferedReader.close();
	    

	    double dist[][] = new double[ConvertCoord2Dist.NUMBER_NODES][ConvertCoord2Dist.NUMBER_NODES];
	    dist = ConvertCoord2Dist.distance(x,y);

	    fw = new FileWriter(ConvertCoord2Dist.DIR + ConvertCoord2Dist.OUT_FILE);
	    bw = new BufferedWriter(fw);
	    //bw.write(ConvertCoord2Dist.NUMBER_NODES+" "+ConvertCoord2Dist.numberOfEdges(ConvertCoord2Dist.NUMBER_NODES));
	    //bw.newLine();
	    for (int i = 0; i < ConvertCoord2Dist.NUMBER_NODES; i++) {
		for (int j = 0; j < (ConvertCoord2Dist.NUMBER_NODES-1); j++) {
		    bw.write(String.valueOf(dist[i][j]) + " ");
		    //bw.write(String.valueOf(i) + " " + String.valueOf(j) + " " + String.valueOf(dist[i][j]));
		    //bw.newLine();
		}
		bw.write(String.valueOf(dist[i][ConvertCoord2Dist.NUMBER_NODES-1]));
		bw.newLine();
	    }
	    

	    
	} catch (IOException x) {
	    System.out.println(x.toString());
	} finally {
	    if (bufferedReader != null) { try { bufferedReader.close(); } catch(Throwable t) { } }
	    if (fileReader != null) { try { fileReader.close(); } catch(Throwable t) { } }
	    //if (file != null) { try { file.close(); } catch(Throwable t) { } }
	    if (bw != null) { try { bw.close(); } catch(Throwable t) { } }
	    if (fw != null) { try { fw.close(); } catch(Throwable t) { } }
	}

    }
}
