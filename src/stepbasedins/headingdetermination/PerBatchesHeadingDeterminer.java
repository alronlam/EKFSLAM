package stepbasedins.headingdetermination;

import java.util.ArrayList;

import stepbasedins.data.SensorEntry;

public class PerBatchesHeadingDeterminer implements HeadingDeterminer {
	private Double currHeading;
	private int batchCount;
	private double runningSum;
	private int windowSize = 3;

	public PerBatchesHeadingDeterminer(int windowSize) {
		this.windowSize = windowSize;
	}

	@Override
	public double getHeading(ArrayList<SensorEntry> batch) {
		double avg = HeadingDeterminerUtil.calculateAvgSensorEntry(batch);
		runningSum += avg;

		if (currHeading == null) {
			currHeading = avg;
			runningSum = 0;
		} else if (batchCount == 0) {
			currHeading = runningSum / windowSize;
			runningSum = 0;
		}

		batchCount = (batchCount + 1) % windowSize;

		return currHeading;
	}
}
