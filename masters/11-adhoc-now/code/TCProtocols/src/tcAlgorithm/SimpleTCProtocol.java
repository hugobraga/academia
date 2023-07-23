package tcAlgorithm;

import dataStructure.PositionTable;
import dataStructure.Weight;

public abstract class SimpleTCProtocol extends TCProtocol{
    
    SimpleTCProtocol (int nsize, PositionTable npt, double nmprange, Weight nedgeWeight) {
    	super(nsize, npt, nmprange, nedgeWeight);
    }
    
    public abstract boolean execute();
    
    public abstract String getName();
    
}
