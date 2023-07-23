package dataStructure;

public class Position {
	int index;
	double distance;
	double distance2Center;
	
	public Position(int index, double distance) {
		this.index = index;
		this.distance = distance;
	}
	
	public int getIndex() {
		return index;
	}
	
	public double getDistance() {
		return distance;
	}
	
	public void setDistance2Center(double distance) {
		distance2Center = distance;
	}
	
	public double getDistance2Center() {
		return distance2Center;
	}
}
