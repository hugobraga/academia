package support;

import java.util.Comparator;

public class IntegerComparator implements Comparator<Integer> {

	@Override
	public int compare(Integer arg0, Integer arg1) {
		// TODO Auto-generated method stub
		if (arg0 < arg1)
			return -1;
		else if (arg0 > arg1)
			return 1;
		else
			return 0;
	}

}
