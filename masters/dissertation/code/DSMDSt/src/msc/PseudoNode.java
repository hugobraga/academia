package msc;

public class PseudoNode {
	Node v, u;
	public PseudoNode(Node informed, Node uninformed) {
		v = informed;
		u = uninformed;
	}
	
	public Node getInformed() {
		return v;
	}
	
	public Node getUninformed() {
		return u;
	}
	
	public boolean equals(PseudoNode i) {
		if ((i.getInformed().getId() == v.getId()) && (i.getUninformed().getId() == u.getId()))
			return true;
		else
			return false;
	}	
}
