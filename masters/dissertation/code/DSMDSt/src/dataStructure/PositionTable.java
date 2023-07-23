package dataStructure;

import java.io.*;
import java.util.*;

public class PositionTable {
    int size;
    PositionTableEntry[] ptable;

    public PositionTable (int tablesize) {
	size = tablesize;

	ptable = new PositionTableEntry[size];
	for (int i =0; i < size; i++)
	    ptable[i] = new PositionTableEntry();
    }

    public void insertEntry (int i, double x, double y) {
	ptable[i].setX(x);
	ptable[i].setY(y);
    }

    public double getX (int i) { return ptable[i].getX(); }

    public double getY (int i) { return ptable[i].getY(); }

    public void readFromTopologyFile (String fname) {
	String s;

	try {
	    BufferedReader in = new BufferedReader(new FileReader(fname));
	    String aLine;

	    for (int i = 0; i < size; i++) {
		aLine = in.readLine();
		StringTokenizer stx = new StringTokenizer(aLine);

		s = stx.nextToken();    // read ns node id
		s = stx.nextToken();    // read word 'set'
		s = stx.nextToken();    // read word 'X_'
		s = stx.nextToken();    // read X coordinate
		ptable[i].setX(Double.parseDouble(s));

		aLine = in.readLine();
		StringTokenizer sty = new StringTokenizer(aLine);

		s = sty.nextToken();    // read ns node id
		s = sty.nextToken();    // read word 'set'
		s = sty.nextToken();    // read word 'Y_'
		s = sty.nextToken();    // read Y coordinate
		ptable[i].setY(Double.parseDouble(s));
		//System.out.println(i+", X: "+ptable[i].getX()+" Y: "+ptable[i].getY());

		aLine = in.readLine();   // read Z coordinate line
		aLine = in.readLine();   // read initial position line
		aLine = in.readLine();  // read empty line
	    }
	    in.close();
	} catch (IOException e) {
		//System.out.println("deu erro na hora de ler topo");
	}
    }

    public void print() {
	System.out.println ("Position Table:");
	for (int i = 0; i < size; i++) {
	    System.out.println("no[" + i + "]: X: " + ptable[i].getX() + " Y: " + ptable[i].getY());   
	}
    }

}
	
