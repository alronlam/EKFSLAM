package util;

import java.util.List;

public class MathHelper {

	public static double getAverage(List<Double> list) {
		double avg = 0;
		for (Double val : list)
			avg += val;
		return avg / list.size();
	}

	public static double getSum(List<Double> list) {
		double sum = 0;
		for (Double val : list)
			sum += val;
		return sum;
	}

}
