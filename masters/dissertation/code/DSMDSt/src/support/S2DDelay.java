package support;

public class S2DDelay {
	int source, destination;
	int linkNode;
	double shortestDelay;
	
	public S2DDelay (int s, int d, double delay) {
		source = s;
		destination = d;
		shortestDelay = delay;
	}
	
	public void addLinkNode(int vId) {
		linkNode = vId;
	}
	
	public int getLinkNode() {
		return linkNode;
	}
	
	public double getDelay() {
		return shortestDelay;
	}
	
	public int getSource() {
		return source;
	}
	
	public int getDestination() {
		return destination;
	}
}
