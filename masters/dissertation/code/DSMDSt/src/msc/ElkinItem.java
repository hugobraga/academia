package msc;

public class ElkinItem {
	PseudoNode pseudo;
	Node node;
	
	public ElkinItem(PseudoNode source, Node dest) {
		pseudo = source;
		node = dest;
	}

	public Node getNode() {
		// TODO Auto-generated method stub
		return node;
	}
	
	public PseudoNode getPseudo() {
		return pseudo;
	}
	
	public ElkinItem copy() {
		ElkinItem it;
		if (pseudo == null)
			it = new ElkinItem(null, new Node(node.getId()));
		else 
			it = new ElkinItem(new PseudoNode(new Node(pseudo.getInformed().getId()), new Node(pseudo.getUninformed().getId())), new Node(node.getId()));
		return it;
	}
	
	public void print() {
		System.out.println("imprimindo ElkinItem");
		if (pseudo == null)
			System.out.println("outro no: "+node.getId());
		else
			System.out.println("pseudo-informed: "+pseudo.getInformed().getId()+", pseudo-uninformed: "+pseudo.getUninformed().getId()+", outro no: "+node.getId());		
	}

	public boolean equals(Object o) {
		ElkinItem ei = (ElkinItem)o;
		if (node.getId() != ei.getNode().getId()) {
			return false;
		} else if ((pseudo != null) && (ei.getPseudo() != null)) {
			if ((pseudo.getInformed().getId() != ei.getPseudo().getInformed().getId()) || 
					(pseudo.getUninformed().getId() != ei.getPseudo().getUninformed().getId()))
				return false;
		}
		return true;
	}
}
