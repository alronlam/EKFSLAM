package stepbasedins.headingdetermination;

import java.util.ArrayList;

import stepbasedins.data.SensorEntry;

public class SmootheningHeadingDeterminer implements HeadingDeterminer {

	private int batchCount;
	private Double runningAvg;
	private int windowSize = 3;
	private ArrayList<Double> window;

	public SmootheningHeadingDeterminer() {
		window = new ArrayList<Double>();
	}

	@Override
	public double getHeading(ArrayList<SensorEntry> batch) {
		double avg = HeadingDeterminerUtil.calculateAvgSensorEntry(batch);
		window.add(avg);

		if (window.size() < windowSize) {
			runningAvg = HeadingDeterminerUtil.calculateAvgDouble(window);
		} else {
			runningAvg -= window.remove(0) / windowSize;
			runningAvg += avg / windowSize;
		}

		return runningAvg;
	}

}
