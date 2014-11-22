package stepbasedins.controller;

import java.util.ArrayList;

import stepbasedins.data.BatchProcessingResults;
import stepbasedins.data.DetectedEntry;
import stepbasedins.data.SensorEntry;
import stepbasedins.headingdetermination.HeadingDeterminer;
import stepbasedins.headingdetermination.SimpleHeadingDeterminer;
import stepbasedins.stepdetection.PeakThresholdStepDetector;
import stepbasedins.stepdetection.StepDetector;
import stepbasedins.stridelengthestimation.LinearStrideLengthEstimator;
import stepbasedins.stridelengthestimation.StrideLengthEstimator;

public class StepBasedINSController {

	// Plan to make all these three interfaces so we can easily switch
	// techniques. Just not sure yet what parameters are needed.
	private StepDetector stepDetector;
	private StrideLengthEstimator strideLengthEstimator;
	private HeadingDeterminer headingDeterminer;

	public StepBasedINSController() {
		this.stepDetector = new PeakThresholdStepDetector();
		this.strideLengthEstimator = new LinearStrideLengthEstimator();
		this.headingDeterminer = new SimpleHeadingDeterminer();

	}

	public BatchProcessingResults processSensorEntryBatch(ArrayList<SensorEntry> batch) {

		BatchProcessingResults results = new BatchProcessingResults();

		// Step detection
		ArrayList<DetectedEntry> detectedSteps = stepDetector.detectSteps(batch);

		// Stride length estimation
		double strideLength = strideLengthEstimator.estimateLength(detectedSteps);

		// Heading determination
		double headingAngle = headingDeterminer.getHeading(batch);

		// return results

		results.setDetectedSteps(detectedSteps.size());
		results.setStrideLength(strideLength);
		results.setHeadingAngle(headingAngle);

		return results;
	}

}
