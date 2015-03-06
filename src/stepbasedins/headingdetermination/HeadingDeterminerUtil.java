package stepbasedins.headingdetermination;

import java.util.ArrayList;

import stepbasedins.data.SensorEntry;

public class HeadingDeterminerUtil {
	public static double calculateAvgSensorEntry(ArrayList<SensorEntry> batch) {
		if (batch.size() == 0)
			return 0;

		double avg = 0;
		for (SensorEntry entry : batch) {
			if (entry != null)
				avg += entry.getOrient_x();
		}

		return avg / batch.size();
	}

	public static double calculateAvgDouble(ArrayList<Double> batch) {
		double avg = 0;
		for (Double val : batch) {
			avg += val;
		}

		return avg / batch.size();
	}

	// Might consider improving this by increasing the number of possible
	// discrete angle values
	public static double getNearestDiscreteHeading(double average) {
		// return average;

		// Log.d("Average angle is ", average+"");
		if (average > 22.5 && average <= 67.5) {
			return 45;
		} else if (average > 67.5 && average <= 112.5) {
			return 90;
		} else if (average > 112.5 && average <= 157.5) {
			return 135;
		} else if (average > 157.5 && average <= 202.5) {
			return 180;
		} else if (average > 202.5 && average <= 247.5) {
			return 225;
		} else if (average > 247.5 && average <= 292.5) {
			return 270;
		} else if (average > 292.5 && average <= 337.5) {
			return 315;
		}

		return 0;
	}

}
