package dummies.features;


import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;
import org.opencv.video.Video;

public class FeatureMatcher {
	private final int MAX_FEATURES = 60;
	private final double QUALITY_LEVEL = 0.01;
	private final double MIN_DISTANCE = 20;
	private final int BLOCK_SIZE = 3;
	private final boolean USE_HARRIS = false;
	private final double k = 0.04;
	
	
	// Largely based off of idp.OpticalFlow
	Result matchUsingOpticalFlow(Mat leftImage, Mat rightImage) {
		
		// Detect new features from image1
		MatOfPoint rawLeftImageFeatures = new MatOfPoint();
		Mat detectMask = new Mat();
		Imgproc.goodFeaturesToTrack(leftImage, rawLeftImageFeatures, MAX_FEATURES, QUALITY_LEVEL, MIN_DISTANCE,
					detectMask, BLOCK_SIZE, USE_HARRIS, k);
		
		// Perform optical flow
		MatOfPoint2f leftImageFeatures = new MatOfPoint2f(rawLeftImageFeatures.toArray());
		MatOfPoint2f rightImageFeatures = new MatOfPoint2f();
		MatOfByte statusMat = new MatOfByte();
		MatOfFloat errorMat = new MatOfFloat();
		
		Video.calcOpticalFlowPyrLK(leftImage, rightImage, leftImageFeatures, rightImageFeatures, statusMat,
				errorMat);
		
		// Find good features
		List<Point> leftImageFeaturesList = leftImageFeatures.toList();
		List<Point> rightImageFeaturesList = rightImageFeatures.toList();
		List<Point> goodLeftFeaturesList = new ArrayList<>();
		List<Point> goodRightFeaturesList = new ArrayList<>();
		List<Byte> statusList = statusMat.toList();
		
		
		for (int index = 0; index < statusList.size(); index++) {
			Byte status = statusList.get(index);
			if (status.intValue() == 1) {
				goodLeftFeaturesList.add( leftImageFeaturesList.get(index) );
				goodRightFeaturesList.add( rightImageFeaturesList.get(index) );
			}
		}
		
		return new Result(goodLeftFeaturesList, goodRightFeaturesList);
	}
	
	class Result {
		List<Point> leftFeatures;
		List<Point> rightFeatures;
		
		private Result(List<Point> leftFeatures, List<Point> rightFeatures) {
			this.leftFeatures = leftFeatures;
			this.rightFeatures = rightFeatures;
		}
	}
}
