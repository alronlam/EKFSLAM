package util;

import java.util.List;

public class AverageHelper {

	public static double getAverage(List<Double> list) {
		double avg = 0;
		for (Double val : list)
			avg += val;
		return avg / list.size();
	}

}
