package script;

public class MainEuclidean131FloatETC {

    private static final int EXIST_SIZES[][][] =
	{ {
	      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
	      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
	      {1, 1, 1, 1, 1, 1, 1, 1, 1, 0},
	      {1, 1, 1, 1, 1, 1, 1, 0, 0, 0}
	  }, //FATOR = 3
	  
	  {
	      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
	      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
	      {1, 1, 1, 1, 1, 1, 1, 1, 1, 0},
	      {1, 1, 1, 1, 1, 1, 1, 0, 0, 0}
	  }, //FATOR = 4
	  
	  {
	      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
	      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
	      {1, 1, 1, 1, 1, 1, 1, 1, 1, 0},
	      {1, 1, 1, 1, 1, 1, 1, 0, 0, 0}
	  }, //FATOR = 5
	  
	  {
	      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
	      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
	      {1, 1, 1, 1, 1, 1, 1, 1, 1, 0},
	      {1, 1, 1, 1, 1, 1, 1, 0, 0, 0}
	  } //FATOR = 6
	  
	};
    
    private static final int QT_INST = 10;
    private static final int[] SIZES = {10, 20, 30, 40, 50, 60, 80, 100, 120, 131};
    private static final int[] STR_FACTORS = {3, 4, 5, 6};
    private static final int[] EDGES_PERC = {20, 40, 60, 80};
    //private static final int[] EDGES_PERC_ind = {0};
    private static final String EDGE_COST = "euclidean-131";
    private static final String ALG = "tree_labels_dist_spanner_etc_float";
    private static final String OUT_FILE_COMP = "-";
    private static final String TIME_OUT_FILE = MainEuclidean131FloatETC.OUT_FILE_COMP+"time_output.dat";
    //private static final String EDGE_OUT_FILE = MainEuclidean131FloatETC.OUT_FILE_COMP+"edge_output.dat";
    //private static final String OPT_OUT_FILE = MainEuclidean131FloatETC.OUT_FILE_COMP+"opt_output.dat";
    
    
    public static void main(String[] args) {
	for (int k = 0; k < MainEuclidean131FloatETC.EDGES_PERC.length; k++) {
	//for (int i = 0; i < MainEuclidean131FloatETC.SIZES.length; i++) {
	//for (int f = 0; f < MainEuclidean131FloatETC.STR_FACTORS.length; f++) {
	//for (int f = 0; f < 1; f++) {

	    for (int i = 0; i < MainEuclidean131FloatETC.SIZES.length; i++) {
	    //for (int k = 0; k < MainEuclidean131FloatETC.EDGES_PERC.length; k++){
	    //for (int i = 0; i < MainEuclidean131FloatETC.SIZES.length; i++) {
	    //for (int i = 0; i < 1; i++) {

		for (int j = 1; j <= MainEuclidean131FloatETC.QT_INST; j++) {
		//for (int k = 0; k < MainEuclidean131FloatETC.EDGES_PERC.length; k++){
		//for (int k = 0; k < 1; k++){
		    for (int f = 0; f < MainEuclidean131FloatETC.STR_FACTORS.length; f++) {
		    //for (int j = 1; j <= MainEuclidean131FloatETC.QT_INST; j++) {
		    //for (int j = 1; j <= 1; j++) {
			if (MainEuclidean131FloatETC.EXIST_SIZES[f][k][i] == 1)
			    continue;
			
			String inFile = new String(j+"-"+MainEuclidean131FloatETC.EDGES_PERC[k]+"-"+MainEuclidean131FloatETC.SIZES[i]+"-"+MainEuclidean131FloatETC.EDGE_COST+"-input.dat");

			String timeOutFile = new String(""+MainEuclidean131FloatETC.STR_FACTORS[f]+"-"+MainEuclidean131FloatETC.EDGES_PERC[k]+"-"+MainEuclidean131FloatETC.SIZES[i]+"-"+MainEuclidean131FloatETC.EDGE_COST+"-"+MainEuclidean131FloatETC.ALG+MainEuclidean131FloatETC.TIME_OUT_FILE);
			String[] cmd = {"./"+MainEuclidean131FloatETC.ALG, inFile, ""+MainEuclidean131FloatETC.STR_FACTORS[f], timeOutFile, ""+j};
			System.out.println(cmd[0]+" "+cmd[1]+" "+cmd[2]+" "+cmd[3] +" "+cmd[4]);
			NativeExecute2 natExecute = new NativeExecute2(cmd);
			int exitValue = natExecute.getExitValue();
			if (exitValue == 1) break;
		    }	    
		    
		}		
	    }	    
	}
    }
}
