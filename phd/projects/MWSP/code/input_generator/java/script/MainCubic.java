package script;

//import main.Support;

public class MainCubic {
    public static final int QT_INST = 10;
    public static final int[] SIZES = {100};
    public static final int[] STR_FACTORS = {2};
    public static final int[] EDGES_PERC = {20};
    public static final String EDGE_COST = "unit";
    public static final String[] ALG = {"graph-arb_spanner"};
    public static final String OUT_FILE_COMP[] = {"-cubic"};
    //public static final String OUT_FILE_COMP = "-plus-";
    // public static final String TIME_OUT_FILE = Main.OUT_FILE_COMP+"time_output.dat";
    // public static final String EDGE_OUT_FILE = Main.OUT_FILE_COMP+"edge_output.dat";
    // public static final String OPT_OUT_FILE = Main.OUT_FILE_COMP+"opt_output.dat";
    
    
    public static void main(String[] args) {
	for (int z = 0; z < MainCubic.ALG.length; z++) {
	    String TIME_OUT_FILE = MainCubic.OUT_FILE_COMP[0]+"-time_output.dat";
	    String EDGE_OUT_FILE = MainCubic.OUT_FILE_COMP[0]+"-edge_output.dat";
	    //String OPT_OUT_FILE = MainCubic.OUT_FILE_COMP[z]+"opt_output.dat";	    
	    for (int f = 0; f < MainCubic.STR_FACTORS.length; f++) {
		//String str = new String(MainCubic.STR_FACTORS[f]);
		for (int i = 2; i <= (MainCubic.SIZES[0]/2); i++) {
		    int size = i*2;
		//for (int i = 0; i < MainCubic.SIZES.length; i++) {

		    //for (int k = 0; k < MainCubic.EDGES_PERC.length; k++){
		        //for (int j = 1; j <= MainCubic.QT_INST; j++) {
		    //String inFile = new String(j+"-"+MainCubic.EDGES_PERC[k]+"-"+MainCubic.SIZES[i]+"-"+MainCubic.EDGE_COST+"-input.dat");
		    String inFile = new String(size+"-"+MainCubic.EDGE_COST+"-input-cubic.dat");
			    //String file = new String("1input10_unit.dat");
			    //String timeOutFile = new String(""+MainCubic.STR_FACTORS[f]+"-"+MainCubic.EDGES_PERC[k]+"-"+MainCubic.SIZES[i]+"-"+MainCubic.EDGE_COST+"-"+MainCubic.ALG[z]+TIME_OUT_FILE);
		    String timeOutFile = new String(""+MainCubic.STR_FACTORS[f]+"-"+size+"-"+MainCubic.EDGE_COST+"-"+MainCubic.ALG[z]+TIME_OUT_FILE);
		    //String edgesOutFile = new String(""+MainCubic.STR_FACTORS[f]+"-"+MainCubic.EDGES_PERC[k]+"-"+MainCubic.SIZES[i]+"-"+MainCubic.EDGE_COST+"-"+MainCubic.ALG[z]+EDGE_OUT_FILE);
		    String edgesOutFile = new String(""+MainCubic.STR_FACTORS[f]+"-"+size+"-"+MainCubic.EDGE_COST+"-"+MainCubic.ALG[z]+EDGE_OUT_FILE);
			    //String optOutFile = new String(""+MainCubic.STR_FACTORS[f]+"-"+MainCubic.EDGES_PERC[k]+"-"+MainCubic.SIZES[i]+"-"+MainCubic.EDGE_COST+"-"+MainCubic.ALG[z]+OPT_OUT_FILE);
			    //System.out.println(timeOutFile);
			    String[] cmd = {"./"+MainCubic.ALG[z], inFile, ""+MainCubic.STR_FACTORS[f], timeOutFile, edgesOutFile};
			    System.out.println(cmd[0]+" "+cmd[1]+" "+cmd[2] +" "+cmd[3] +" "+cmd[4]);
			    //String workingDirFile = "input10.dat 2";
	
			    // NativeExecute natExecute = new NativeExecute(cmd, workingDirFile);
			    NativeExecute natExecute = new NativeExecute(cmd);
		
			//}	    
		    
		    //}		
		}	    
	    }
	}
    }
}
