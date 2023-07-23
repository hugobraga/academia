package support;

public class Vertices implements Comparable<Vertices> {
	int id1, id2;
	
	public Vertices(int i1, int i2) {
		if (i2 < i1) {
			id1 = i2;
			id2 = i1;;
		} else {
			id1 = i1;
			id2 = i2;;			
		}
	}
	
	public int compareTo(Vertices o) {
		if (id1 < o.id1)
			return -1;
		else if ((id1 == o.id1) && (id2 == o.id2))
			return 0;
		else
			return 1;
	}
	
	public boolean equals(Vertices e) {
		if (((id1 == e.id1) && (id2 == e.id2)) || ((id1 == e.id2) && (id2 == e.id1)))
			return true;
		else
			return false;
	}	
}
