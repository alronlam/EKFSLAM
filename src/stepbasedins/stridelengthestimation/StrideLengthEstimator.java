package stepbasedins.stridelengthestimation;

import java.util.ArrayList;

import stepbasedins.data.DetectedEntry;

public interface StrideLengthEstimator {
	public double estimateLength(ArrayList<DetectedEntry> detectedSteps);

}
