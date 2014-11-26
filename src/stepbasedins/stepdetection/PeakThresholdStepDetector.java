package stepbasedins.stepdetection;

import java.util.ArrayList;

import stepbasedins.data.DetectedEntry;
import stepbasedins.data.SensorEntry;

public class PeakThresholdStepDetector implements StepDetector {

	private final double PEAK_THRESHOLD = 12;
	private final double TIME_GAP = 350; // ms

	private DetectedEntry lastDetectedEntry;

	/*
	 * Just detects the entries that pass the threshold, while using the time
	 * gap to make sure no entries are too close to each other.
	 */
	public ArrayList<DetectedEntry> detectSteps(ArrayList<SensorEntry> batch) {
		// insert code for the negative locomotion classifier here

		ArrayList<DetectedEntry> peaks = new ArrayList<DetectedEntry>();
		int size = batch.size();
		for (int i = 0; i < size; i++) {
			SensorEntry currEntry = batch.get(i);
			if (currEntry != null) {
				double accelerometerNorm = currEntry.getAcc_norm();
				// Log.d("Norm is ", accelerometerNorm+"");

				if (lastDetectedEntry != null)
					System.out.println(accelerometerNorm
							+ " time: "
							+ (currEntry.getTimeRecorded() - lastDetectedEntry.getTimeRecorded() + " = "
									+ currEntry.getTimeRecorded() + " - " + lastDetectedEntry.getTimeRecorded()));

				if (accelerometerNorm >= PEAK_THRESHOLD
						&& (lastDetectedEntry == null || (lastDetectedEntry != null && currEntry.getTimeRecorded()
								- lastDetectedEntry.getTimeRecorded() >= TIME_GAP))) {

					DetectedEntry newStep = new DetectedEntry(currEntry.getTimeRecorded(), accelerometerNorm);
					peaks.add(newStep);
					lastDetectedEntry = newStep;
				}
			}
		}

		return peaks;
	}

}