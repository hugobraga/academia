package support;

import java.util.Comparator;

public class DegreeComparator implements Comparator<NodeDegree> {

	@Override
	public int compare(NodeDegree arg0, NodeDegree arg1) {
		if (arg0.getDegree() < arg1.getDegree())
			return -1;
		else if (arg0.getDegree() > arg1.getDegree())
			return 1;
		else {
			if (arg0.getId() < arg1.getId())
				return -1;
			else if (arg0.getId() > arg1.getId())
				return 1;
		}
		return 0;
	}

}
