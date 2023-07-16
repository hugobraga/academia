package script;

//import main.Support;

public class MainCubicGen {
    public static final int QT_INST = 10;
    public static final int[] SIZES = {100};
    public static final int[] STR_FACTORS = {2};
    public static final int[] EDGES_PERC = {20};
    public static final String EDGE_COST = "random";
    public static final String[] ALG = {"gen-regular"};
    public static final String OUT_FILE_COMP[] = {"-plus-"};
    //public static final String OUT_FILE_COMP = "-plus-";
    // public static final String TIME_OUT_FILE = Main.OUT_FILE_COMP+"time_output.dat";
    // public static final String EDGE_OUT_FILE = Main.OUT_FILE_COMP+"edge_output.dat";
    // public static final String OPT_OUT_FILE = Main.OUT_FILE_COMP+"opt_output.dat";
    
    
    public static void main(String[] args) {
	for (int z = 0; z < MainCubicGen.ALG.length; z++) {
	    String TIME_OUT_FILE = "-time_output.dat";
	    String EDGE_OUT_FILE = "-edge_output.dat";
	    //String OPT_OUT_FILE = Main.OUT_FILE_COMP[z]+"opt_output.dat";

	    for (int i = 2; i <= (MainCubicGen.SIZES[0]/2); i++) {
		int size = i*2;
		String[] cmd = {"./"+MainCubicGen.ALG[z], ""+size, "3"};
		System.out.println(cmd[0]+" "+cmd[1]+" "+cmd[2]);
		NativeExecute natExecute = new NativeExecute(cmd);
	    }
	}
    }
}
