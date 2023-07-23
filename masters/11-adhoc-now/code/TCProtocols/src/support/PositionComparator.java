package support;

import java.util.Comparator;

import dataStructure.Position;

public class PositionComparator implements Comparator<Position> {
	boolean distance2CenterComp;

	public PositionComparator(boolean distance2Center) {
		distance2CenterComp = distance2Center;
	}

	@Override
	public int compare(Position arg0, Position arg1) {
		double dist0, dist1;
		
		if (distance2CenterComp) {
			dist0 = arg0.getDistance2Center();
			dist1 = arg1.getDistance2Center();
		} else {
			dist0 = arg0.getDistance();
			dist1 = arg1.getDistance();
		}
		
		if (dist0 < dist1)
			return -1;
		else if (dist0 > dist1)
			return 1;
		else
			return 0;
	}

}
