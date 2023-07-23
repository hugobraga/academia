package support;

public class Neighbor implements Comparable<Neighbor>{
	int id;
	double distance;
	
	public Neighbor(int id, double dist) {
		this.id = id;
		distance = dist;
	}
	
	public int getId() {return id;}
	public double getDistance() {return distance;}

	@Override
	public int compareTo(Neighbor neigh) {
		if (distance < neigh.getDistance())
			return -1;
		else if (distance == neigh.getDistance())
		// TODO Auto-generated method stub
			return 0;
		else
			return 1;
	}
}
