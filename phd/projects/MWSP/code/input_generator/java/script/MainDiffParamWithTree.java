package script;

public class MainDiffParamWithTree {

    private static int QT_EDGES_UNI[] = {100,
					 // 125,
					 // 150,
					 // 175,
					 200,
					 // 225,
					 // 250,
					 // 275,
					 300};
    
    private static final double DENS[][] = {
	{0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0},
    	{0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0},	
	{0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0},
	{0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0}
	
    	// // {0.1, 0.15, 0.2, 0.25, 0.3, 0.35},
	// // {0.1, 0.15, 0.2, 0.25, 0.3, 0.35},
	// // {0.1, 0.15, 0.2, 0.25, 0.3, 0.35},
	// // {0.1, 0.15, 0.2, 0.25, 0.3, 0.35},
	// // {0.1, 0.15, 0.2, 0.25, 0.3, 0.35},
	// // {0.1, 0.15, 0.2, 0.25, 0.3, 0.35},
	// {// 0.1,
	//  // 0.15,
	//  // 0.2,
	//  // 0.25,
	//  // 0.3,
	//  // 0.35
	//  0.4,
	//  0.5
	// },
	
	// {// 0.1,
	//  // 0.15,
	//  // 0.2,
	//  // 0.25,
	//  // 0.3,
	//  // 0.35
	//  0.4,
	//  0.5	 
	// },
	
	// {// 0.1,
	//  // 0.15,
	//  // 0.2,
	//  // 0.25,
	//  // 0.3,
	//  // 0.35
	//  0.4,
	//  0.5	 
	// }
	
	// // {50, 75, 100, 125, 150, 175, 200},
	// // {75, 100, 125, 150, 175, 200},
	// // {100, 125, 150, 175, 200}
    };

    private static int DEGREES[][] = {
	{4, 8},
	{4, 8}
    };

    private static int QT_EDGES[][] = {
    	{// 50, 75, 100, 
	    100, 120},
    	{// 75, 100, 
	    100, 120}
    };
    
    
        
    // private static final int EXIST_SIZES[][][] =
    // 	{ {
    // 	      {0.23, 0.46, 0.69, 0, 0, 0, 0, 0},
    // 	      {0.13, 0.26, 0.39, 0.52, 0.65, 0, 0, 0},
    // 	      {8, 16, 24, 32, 40, 48, 56, 64}
    // 	  }, //FATOR = 4

    // 	  {
    // 	      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    // 	      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
    // 	      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
    // 	      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1}
    // 	  }, //FATOR = 5

    // 	  {
    // 	      {0, 0, 0, 0, 0, 0, 1, 1, 1, 1},
    // 	      {0, 0, 1, 1, 1, 0, 1, 1, 1, 1},
    // 	      {0, 0, 0, 0, 0, 1, 1, 1, 1, 1},
    // 	      {0, 0, 0, 0, 0, 1, 1, 1, 1, 1}
    // 	  } //FATOR = 6
	  
    // 	};

    private static final String NOME_PARAM = "tree";
    private static final int QT_INST = 20;
    private static final int[] SIZES = {20, 30, 40, 50};
    private static final double[] STR_FACTORS = {3};
    // private static final int[] DEGREE = {4, 8};
    //private static final int[] DEGREE_ind = {0};
    private static final String EDGE_COST[] = {"unit"};
    private static final String ADD_EDGE_TYPE[] = {"random"};
    // private static final String ALG[] = {"tree_labels_dist_spanner"};
    private static final String ALG[] = {"tree_find_dist_spanner"};
    private static final String IN_DIR = "testando3/";
    private static final String OUT_DIR = "tese/";
    private static final String OUT_FILE_COMP = "";
    private static final String TIME_OUT_FILE = MainDiffParamWithTree.OUT_FILE_COMP+"time_output.dat";

    private static final String TMP_FILE = "input.dat";
    
    //private static final String EDGE_OUT_FILE = MainDiffParamWithTree.OUT_FILE_COMP+"edge_output.dat";
    //private static final String OPT_OUT_FILE = MainDiffParamWithTree.OUT_FILE_COMP+"opt_output.dat";

    private static void orderNotParam(boolean insertTree) {
    	for (int k = 0; k < MainDiffParamWithTree.QT_EDGES_UNI.length; k++) {
    	// for (int i = 0; i < MainDiffParamWithTree.SIZES.length; i++) {
    	    for (int d = 0; d < MainDiffParamWithTree.DENS[k].length; d++) {
    	    // for (int k = 0; k < MainDiffParamWithTree.QT_EDGES_UNI[i].length; k++) {
    		for (int ec = 0; ec < MainDiffParamWithTree.EDGE_COST.length; ec++) {
    		    for (int aet = 0; aet < MainDiffParamWithTree.ADD_EDGE_TYPE.length; aet++) {
    			for (int sf = 0; (sf < MainDiffParamWithTree.STR_FACTORS.length); sf++) {

    			    for (int j = 1; j <= MainDiffParamWithTree.QT_INST; j++) {
    				int order = (int)Math.round((1 + Math.sqrt(1 + (8 * MainDiffParamWithTree.QT_EDGES_UNI[k] / MainDiffParamWithTree.DENS[k][d])))/2);

    				String sfName = new String("");
    				if (insertTree) {
    				    sfName = String.valueOf("-" + MainDiffParamWithTree.STR_FACTORS[sf]);
    				}

				
				String inFile = new String(MainDiffParamWithTree.IN_DIR + j + sfName + "-" + MainDiffParamWithTree.QT_EDGES_UNI[k] + "-" + (int)Math.round(100*MainDiffParamWithTree.DENS[k][d]) + "-size" + "-" + MainDiffParamWithTree.EDGE_COST[ec] + "-" + MainDiffParamWithTree.ADD_EDGE_TYPE[aet] + "-" + MainDiffParamWithTree.TMP_FILE);				

				String timeOutFile = new String(""+MainDiffParamWithTree.OUT_DIR+MainDiffParamWithTree.STR_FACTORS[sf]+"-"+MainDiffParamWithTree.QT_EDGES_UNI[k]+"-"+(int)Math.round(100*MainDiffParamWithTree.DENS[k][d])+"-size"+"-"+MainDiffParamWithTree.EDGE_COST[ec]+"-"+MainDiffParamWithTree.ADD_EDGE_TYPE[aet]+"-"+MainDiffParamWithTree.ALG[ec]+"-"+MainDiffParamWithTree.TIME_OUT_FILE);
				String[] cmd = {"./"+MainDiffParamWithTree.ALG[ec], inFile, ""+MainDiffParamWithTree.STR_FACTORS[sf], timeOutFile, ""+j};
				System.out.println(cmd[0]+" "+cmd[1]+" "+cmd[2]+" "+cmd[3] +" "+cmd[4]);
				NativeExecute2 natExecute = new NativeExecute2(cmd);
				int exitValue = natExecute.getExitValue();
    			    }
			    
    			}
			
    		    }
    		}
    	    }
    	}
	
    }


    private static void orderAndDegreeParam(boolean insertTree) {
    	for (int i = 0; i < MainDiffParamWithTree.SIZES.length; i++) {
    	    for (int k = 0; k < MainDiffParamWithTree.DEGREES[i].length; k++) {
    		for (int ec = 0; ec < MainDiffParamWithTree.EDGE_COST.length; ec++) {
    		    for (int aet = 0; aet < MainDiffParamWithTree.ADD_EDGE_TYPE.length; aet++) {

			
    			for (int sf = 0; (sf < MainDiffParamWithTree.STR_FACTORS.length); sf++) {

    			    for (int j = 1; j <= MainDiffParamWithTree.QT_INST; j++) {
				double perc = (1.0*MainDiffParamWithTree.DEGREES[i][k])/(MainDiffParamWithTree.SIZES[i]-1);


    				String sfName = new String("");
    				if (insertTree) {
    				    sfName = String.valueOf("-" + MainDiffParamWithTree.STR_FACTORS[sf]);
    				}

				
				String inFile = new String(MainDiffParamWithTree.IN_DIR + j + sfName + "-" + MainDiffParamWithTree.DEGREES[i][k] + "-" + MainDiffParamWithTree.SIZES[i] + "-order_degree-" + MainDiffParamWithTree.EDGE_COST[ec] + "-" + MainDiffParamWithTree.ADD_EDGE_TYPE[aet] + "-" + MainDiffParamWithTree.TMP_FILE);
				
				String timeOutFile = new String(""+MainDiffParamWithTree.OUT_DIR+MainDiffParamWithTree.STR_FACTORS[sf]+"-"+MainDiffParamWithTree.DEGREES[i][k]+"-"+MainDiffParamWithTree.SIZES[i]+"-order_degree-"+MainDiffParamWithTree.EDGE_COST[ec]+"-"+MainDiffParamWithTree.ADD_EDGE_TYPE[aet]+"-"+MainDiffParamWithTree.ALG[ec]+"-"+MainDiffParamWithTree.TIME_OUT_FILE);
				String[] cmd = {"./"+MainDiffParamWithTree.ALG[ec], inFile, ""+MainDiffParamWithTree.STR_FACTORS[sf], timeOutFile, ""+j};
				System.out.println(cmd[0]+" "+cmd[1]+" "+cmd[2]+" "+cmd[3] +" "+cmd[4]);
				NativeExecute2 natExecute = new NativeExecute2(cmd);
				int exitValue = natExecute.getExitValue();
    			    }
			    
    			}
			
    		    }
    		}
    	    }
    	}
	
    }


    private static void qtEdgesNotParam(boolean insertTree) {
    	for (int i = 0; i < MainDiffParamWithTree.SIZES.length; i++) {
    	    for (int k = 0; k < MainDiffParamWithTree.DENS[i].length; k++) {
    		for (int ec = 0; ec < MainDiffParamWithTree.EDGE_COST.length; ec++) {
    		    for (int aet = 0; aet < MainDiffParamWithTree.ADD_EDGE_TYPE.length; aet++) {
			
    			for (int sf = 0; (sf < MainDiffParamWithTree.STR_FACTORS.length); sf++) {

    			    for (int j = 1; j <= MainDiffParamWithTree.QT_INST; j++) {
				
    				String sfName = new String("");
    				if (insertTree) {
    				    sfName = String.valueOf("-" + MainDiffParamWithTree.STR_FACTORS[sf]);
    				}
				
				String inFile = new String(MainDiffParamWithTree.IN_DIR + j + sfName + "-" + (int)Math.round(100*MainDiffParamWithTree.DENS[i][k]) + "-" + MainDiffParamWithTree.SIZES[i] + "-edges-" + MainDiffParamWithTree.EDGE_COST[ec] + "-" + MainDiffParamWithTree.ADD_EDGE_TYPE[aet] + "-" + MainDiffParamWithTree.TMP_FILE);
				
				String timeOutFile = new String(""+MainDiffParamWithTree.OUT_DIR+MainDiffParamWithTree.STR_FACTORS[sf]+"-"+ (int)Math.round(100*MainDiffParamWithTree.DENS[i][k]) + "-" + MainDiffParamWithTree.SIZES[i] + "-edges-" +MainDiffParamWithTree.EDGE_COST[ec]+"-"+MainDiffParamWithTree.ADD_EDGE_TYPE[aet]+"-"+MainDiffParamWithTree.ALG[ec]+"-"+MainDiffParamWithTree.TIME_OUT_FILE);
				
				String[] cmd = {"./"+MainDiffParamWithTree.ALG[ec], inFile, ""+MainDiffParamWithTree.STR_FACTORS[sf], timeOutFile, ""+j};
				System.out.println(cmd[0]+" "+cmd[1]+" "+cmd[2]+" "+cmd[3] +" "+cmd[4]);
				NativeExecute2 natExecute = new NativeExecute2(cmd);
				int exitValue = natExecute.getExitValue();
				
    			    }
			    
    			}
			
    		    }
    		}
    	    }
    	}
	
    }
    
    private static void densNotParam(boolean insertTree) {

    	for (int i = 0; i < MainDiffParamWithTree.SIZES.length; i++) {
    	    for (int k = 0; k < MainDiffParamWithTree.QT_EDGES[i].length; k++) {
    		for (int ec = 0; ec < MainDiffParamWithTree.EDGE_COST.length; ec++) {
    		    for (int aet = 0; aet < MainDiffParamWithTree.ADD_EDGE_TYPE.length; aet++) {

			
    			for (int sf = 0; (sf < MainDiffParamWithTree.STR_FACTORS.length); sf++) {			    

    			    for (int j = 1; j <= MainDiffParamWithTree.QT_INST; j++) {
    				double perc = (2.0 * MainDiffParamWithTree.QT_EDGES[i][k])/(MainDiffParamWithTree.SIZES[i] * (MainDiffParamWithTree.SIZES[i]-1));


    				String sfName = new String("");
    				if (insertTree) {
    				    sfName = String.valueOf("-" + MainDiffParamWithTree.STR_FACTORS[sf]);
    				}
				
				String inFile = new String(MainDiffParamWithTree.IN_DIR + j + sfName + "-" + MainDiffParamWithTree.QT_EDGES[i][k] + "-" + MainDiffParamWithTree.SIZES[i] + "-dens-" + MainDiffParamWithTree.EDGE_COST[ec] + "-" + MainDiffParamWithTree.ADD_EDGE_TYPE[aet] + "-" + MainDiffParamWithTree.TMP_FILE);
				
				String timeOutFile = new String(""+MainDiffParamWithTree.OUT_DIR+MainDiffParamWithTree.STR_FACTORS[sf]+"-" + MainDiffParamWithTree.QT_EDGES[i][k] + "-" + MainDiffParamWithTree.SIZES[i] + "-dens-" + MainDiffParamWithTree.EDGE_COST[ec]+"-"+MainDiffParamWithTree.ADD_EDGE_TYPE[aet]+"-"+MainDiffParamWithTree.ALG[ec]+"-"+MainDiffParamWithTree.TIME_OUT_FILE);
				
				String[] cmd = {"./"+MainDiffParamWithTree.ALG[ec], inFile, ""+MainDiffParamWithTree.STR_FACTORS[sf], timeOutFile, ""+j};
				System.out.println(cmd[0]+" "+cmd[1]+" "+cmd[2]+" "+cmd[3] +" "+cmd[4]);
				NativeExecute2 natExecute = new NativeExecute2(cmd);
				int exitValue = natExecute.getExitValue();
			       				
    			    }
			    
    			}
			
    		    }
    		}
    	    }
    	}
	
    }

    
    
    public static void main(String[] args) {
	// MainDiffParamWithTree.orderNotParam(false);
	MainDiffParamWithTree.qtEdgesNotParam(false);
	/*
	for (int i = 0; i < MainDiffParamWithTree.SIZES.length; i++) {

	    for (int k = 0; k < MainDiffParamWithTree.DENS[i].length; k++) {

		for (int d = 0; d < MainDiffParamWithTree.ADD_EDGE_TYPE.length; d++) {

		    for (int j = 1; j <= MainDiffParamWithTree.QT_INST; j++) {
			for (int f = 0; f < MainDiffParamWithTree.STR_FACTORS.length; f++) {
			
			    String inFile = new String(j+"-"+MainDiffParamWithTree.STR_FACTORS[f]+"-"+MainDiffParamWithTree.DENS[i][k]+"-"+MainDiffParamWithTree.SIZES[i]+"-"+MainDiffParamWithTree.NOME_PARAM+"-"+MainDiffParamWithTree.EDGE_COST+"-"+MainDiffParamWithTree.ADD_EDGE_TYPE[d]+"-input.dat");

			    String timeOutFile = new String(""+MainDiffParamWithTree.STR_FACTORS[f]+"-"+MainDiffParamWithTree.DENS[i][k]+"-"+MainDiffParamWithTree.SIZES[i]+"-"+MainDiffParamWithTree.NOME_PARAM+"-"+MainDiffParamWithTree.EDGE_COST+"-"+MainDiffParamWithTree.ADD_EDGE_TYPE[d]+"-"+MainDiffParamWithTree.ALG+"-"+MainDiffParamWithTree.TIME_OUT_FILE);
			    String[] cmd = {"./"+MainDiffParamWithTree.ALG, inFile, ""+MainDiffParamWithTree.STR_FACTORS[f], timeOutFile, ""+j};
			    System.out.println(cmd[0]+" "+cmd[1]+" "+cmd[2]+" "+cmd[3] +" "+cmd[4]);
			    NativeExecute2 natExecute = new NativeExecute2(cmd);
			    int exitValue = natExecute.getExitValue();
			    //if (exitValue == 1) break;
			}	    
		    
		
		    }//SIZES	    
		    
		}//ADD_EDGE_TYPE
	    }

	}//QT_INST	
	*/
    }
}
