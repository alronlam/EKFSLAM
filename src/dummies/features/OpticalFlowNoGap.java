package dummies.features;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.TermCriteria;
import org.opencv.features2d.KeyPoint;
import org.opencv.imgproc.Imgproc;
import org.opencv.video.Video;

class OpticalFlowNoGap {
	private final Scalar BLACK = new Scalar(0);
	private final Scalar WHITE = new Scalar(255);
	private final Scalar RED = new Scalar(255, 0, 0);
	
	// Good Features to Track fields
	private final int MAX_FEATURES = 40;
	private final double QUALITY_LEVEL = 0.01;
	private final double MIN_DISTANCE = 20;
	private final int BLOCK_SIZE = 3;
	private final boolean USE_HARRIS = false;
	private final double k = 0.04;

	// Optical flow fields
	private final Size OPFLOW_WIN_SIZE = new Size(20, 20);
	private final int MAX_LEVEL = 5;
	private final double MIN_EIG_THRESHOLD = 1e-4;
	
	// cornerSubPix fields
	private final Size WIN_SIZE = new Size(15, 15);
	private final Size ZERO_ZONE = new Size(-1, -1);
	
	private ImageIO io;
	
	OpticalFlowNoGap() {
		io = new ImageIO();
		io.deletePhotos();
	}
	
	OpticalFlowResult getFeatures(Mat previousImage, Mat nextImage, MatOfPoint2f previousFeatures) {
		// Create detect mask
		
		Mat detectMask = previousImage.clone();
		detectMask.setTo(WHITE);
		List<Point> previousFeaturesList = previousFeatures.toList();
		for (int i = 0; i < previousFeaturesList.size(); i++) {
			Core.circle(detectMask, previousFeaturesList.get(i), 10, BLACK, -1);
		}
		
		// Detect new features
		double previousSize = previousFeatures.size().height;
		MatOfPoint rawNewFeatures = new MatOfPoint();
		int toFind = MAX_FEATURES - (int) previousSize;
		if (toFind > 0) {
			Imgproc.goodFeaturesToTrack(nextImage, rawNewFeatures, toFind, QUALITY_LEVEL, MIN_DISTANCE, detectMask,
					BLOCK_SIZE, USE_HARRIS, k);
		}
		MatOfPoint2f newFeatures = new MatOfPoint2f(rawNewFeatures.toArray());
		TermCriteria termCriteria = new TermCriteria(TermCriteria.EPS + TermCriteria.MAX_ITER, 40, 0.001);
		
		// Optical Flow
		MatOfPoint2f leftFeatures = new MatOfPoint2f();
		leftFeatures.push_back(previousFeatures);
		leftFeatures.push_back(newFeatures);
		MatOfPoint2f rightFeatures = new MatOfPoint2f();
		
		MatOfByte status = new MatOfByte();
		MatOfFloat error = new MatOfFloat();
		
		if (leftFeatures.size().height > 0) {
			TermCriteria opflowcriteria = new TermCriteria(TermCriteria.EPS + TermCriteria.MAX_ITER, 40, 0.001);
			Video.calcOpticalFlowPyrLK(previousImage, nextImage, leftFeatures, rightFeatures, status, error, 
				OPFLOW_WIN_SIZE, MAX_LEVEL, opflowcriteria, Video.OPTFLOW_LK_GET_MIN_EIGENVALS, MIN_EIG_THRESHOLD);	
		}
		
		// Segregate good, bad, and new

		List<Point> goodLeftFeaturesList = new ArrayList<>();
		List<Point> goodRightFeaturesList = new ArrayList<>();
		List<Integer> badPointsList = new ArrayList<>();
		
		List<Point> leftFeaturesList = leftFeatures.toList();
		List<Point> rightFeaturesList = rightFeatures.toList();
		Mat imageDebug = nextImage.clone();
		
		int index = 0;
		int currents = 0;
		
		for (Byte item : status.toList()) {	
			if (item.intValue() == 1) {
				if (index < previousSize) {
					currents++;
				}
				goodLeftFeaturesList.add(leftFeaturesList.get(index));
				goodRightFeaturesList.add(rightFeaturesList.get(index));
				Core.circle(imageDebug, rightFeaturesList.get(index), 2, RED, 1);
				Core.line(imageDebug, leftFeaturesList.get(index), rightFeaturesList.get(index), RED);

			} else if (index < previousSize) {
				badPointsList.add(Integer.valueOf(index));
			}
			index++;
		}
		io.saveNext(imageDebug);
		
		
		MatOfPoint2f goodNearFeatures = new MatOfPoint2f();
		MatOfPoint2f goodFarFeatures = new MatOfPoint2f();
		goodNearFeatures.fromList(goodLeftFeaturesList);
		goodFarFeatures.fromList(goodRightFeaturesList);
		
		OpticalFlowResult result = new OpticalFlowResult(goodNearFeatures, goodFarFeatures, badPointsList, currents);
		return result;
	}

	private MatOfPoint2f convert(MatOfKeyPoint keyPoints) {
		KeyPoint[] keyPointsArray = keyPoints.toArray();
		Point[] pointsArray = new Point[keyPointsArray.length];

		for (int i = 0; i < keyPointsArray.length; i++) {
			pointsArray[i] = (Point) keyPointsArray[i].pt;
		}

		return new MatOfPoint2f(pointsArray);
	}

	private MatOfPoint2f convert(MatOfPoint keyPoints) {
		Point[] keyPointsArray = keyPoints.toArray();
		Point[] pointsArray = new Point[keyPointsArray.length];

		for (int i = 0; i < keyPointsArray.length; i++) {
			pointsArray[i] = (Point) keyPointsArray[i];
		}

		return new MatOfPoint2f(pointsArray);
	}
}
