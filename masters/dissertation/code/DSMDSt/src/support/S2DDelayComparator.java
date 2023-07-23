package support;

import java.util.Comparator;

public class S2DDelayComparator implements Comparator<S2DDelay> {

	@Override
	public int compare(S2DDelay arg0, S2DDelay arg1) {
		if (arg0.getDelay() < arg1.getDelay())
			return -1;
		else if (arg0.getDelay() > arg1.getDelay())
			return 1;
		else {
			if (arg0.getSource() < arg1.getSource())
				return -1;
			else if (arg0.getSource() > arg1.getSource())
				return 1;
			else {
				if (arg0.getDestination() < arg1.getDestination())
					return -1;
				else if (arg0.getDestination() > arg1.getDestination())
					return 1;
			}
		}
		return 0;
	}

}
