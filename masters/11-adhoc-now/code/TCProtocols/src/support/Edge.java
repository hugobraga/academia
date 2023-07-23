package support;

public class Edge implements Comparable<Edge>{
	public int id1, id2;
	int IS1, IS2;
	int coverage;
	int buckhartCoverage;
	
	public Edge(int i1, int i2, int is1, int is2, int cov, int buckhartCov) {
		if (i2 < i1) {
			id1 = i2;
			id2 = i1;
			IS1 = is2;
			IS2 = is1;
		} else {
			id1 = i1;
			id2 = i2;
			IS1 = is1;
			IS2 = is2;			
		}
		coverage = cov;
		buckhartCoverage = buckhartCov;
	}
	
	public int getCoverage() {
		return coverage;
	}
	
	public int getBuckhartCoverage() {
		return buckhartCoverage;
	}
	
	public void setIS(int is1, int is2) {
		IS1 = is1;
		IS2 = is2;
	}
	
	public int getIS1() {
		return IS1;
	}
	
	public int getIS2() {
		return IS2;
	}	

	@Override
	public int compareTo(Edge o) {
		if (coverage < o.coverage)
			return -1;
		else if (coverage == o.coverage)
			return 0;
		else
			return 1;
	}
	
	public boolean equals(Edge e) {
		if (((id1 == e.id1) && (id2 == e.id2)) || ((id1 == e.id2) && (id2 == e.id1)))
			return true;
		else
			return false;
	}
}
