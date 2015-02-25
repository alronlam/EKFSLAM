package dummies.features;

import java.util.List;

import org.opencv.core.Point;

public class AsyncOpticalFlowResult {
	private List<Point> flowingFeatures;
	private List<Boolean> isGoodFeatures;
	
	AsyncOpticalFlowResult(List<Point> flowingFeatures, List<Boolean> isGoodFeatures) {
		this.flowingFeatures = flowingFeatures;
		this.isGoodFeatures = isGoodFeatures;
	}
	
	List<Point> getFlowingFeatures() {
		return flowingFeatures;
	}
	
	List<Boolean> getIsGoodFeatures() {
		return isGoodFeatures;
	}
}
