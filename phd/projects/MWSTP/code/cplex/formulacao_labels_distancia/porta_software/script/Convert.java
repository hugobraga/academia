package script;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.io.File;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.Files;

//import main.Support;
/*
Converte o padrao oldName pelo padrao newName no arquivo mencionado.
 */

public class Convert {
    public static final String oldName[] = {
	"x.0_1", "x.0_3", "x.1_3", "x.2_3", "z.0.0_1", "z.0.1_0", "z.0.0_3", "z.0.3_0", "z.0.1_3", "z.0.3_1", "z.0.2_3", "z.0.3_2", "z.1.0_1", "z.1.1_0", "z.1.0_3", "z.1.3_0", "z.1.1_3", "z.1.3_1", "z.1.2_3", "z.1.3_2", "z.2.0_1", "z.2.1_0", "z.2.0_3", "z.2.3_0", "z.2.1_3", "z.2.3_1", "z.2.2_3", "z.2.3_2", "z.3.0_1", "z.3.1_0", "z.3.0_3", "z.3.3_0", "z.3.1_3", "z.3.3_1", "z.3.2_3", "z.3.3_2", "u.0.0", "u.0.1", "u.0.2", "u.0.3", "u.1.0", "u.1.1", "u.1.2", "u.1.3", "u.2.0", "u.2.1", "u.2.2", "u.2.3", "u.3.0", "u.3.1", "u.3.2", "u.3.3"
    };
    
    public static final String newName[] = {
	"x1", "x2", "x3", "x4", "x5", "x6", "x7", "x8", "x9", "x10", "x11", "x12", "x13", "x14", "x15", "x16", "x17", "x18", "x19", "x20", "x21", "x22", "x23", "x24", "x25", "x26", "x27", "x28", "x29", "x30", "x31", "x32", "x33", "x34", "x35", "x36", "x37", "x38", "x39", "x40", "x41", "x42", "x43", "x44", "x45", "x46", "x47", "x48", "x49", "x50", "x51", "x52"
    };

    
    public static final int NUMBER_NODES = 131;
    public static final String DIR = "/home/hugo/Dropbox/USP/git/doutorado/projetos/MWSTP/implementacao/cplex/formulacao_labels_distancia/skeleton/";
    public static final String IN_FILE = "input.ieq";
    public static final String OUT_FILE = "input-out.ieq";

    public static void main(String[] args) {

	String inFileName = args[0] + args[1];
	String outFileName = args[0] + args[2];

	try {
	    //System.out.println(inFileName);
	    //System.out.println(outFileName);
	    Path inPath = Paths.get(inFileName);
	    //Charset charset = StandardCharsets.UTF_8;

	    //String content = new String(Files.readAllBytes(path), charset);
	    String content = new String(Files.readAllBytes(inPath));
	    //System.out.println("string: " + content);

	    for (int i = 0; i < Convert.oldName.length; i++) {
		System.out.println(Convert.oldName[i] + " " + Convert.newName[i]);
		content = content.replaceAll(Convert.oldName[i], Convert.newName[i]);
	    }
	    System.out.println(content);
	    Path outPath = Paths.get(outFileName);
	    Files.write(outPath, content.getBytes());	    
	} catch(Exception e){
	    System.out.println(e.getMessage());
	    //System.out.println("exception");
	}	
    }
}
