package support;

public class EdgeIDs implements Comparable<EdgeIDs> {

	public int id1, id2;
	
	public EdgeIDs(int i1, int i2) {
		if (i1 < i2) {
			id1 = i1;
			id2 = i2;
		} else {
			id1 = i2;
			id2 = i1;
		}
	}
	
	@Override
	public int compareTo(EdgeIDs arg0) {
		if (id1 < arg0.id1)
			return -1;
		else if ((id1 == arg0.id1) && (id2 < arg0.id2))
			return -1;
		else if ((id1 == arg0.id1) && (id2 == arg0.id2))
			return 0;
		else
			return 1;
	}
	
	public boolean equals(EdgeIDs arg0) {
		if ((id1 == arg0.id1) && (id2 == arg0.id2))
			return true;
		else
			return false;
	}	

}
