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

class OpticalFlow {
	private final Scalar BLACK = new Scalar(0);
	private final Scalar WHITE = new Scalar(255);
	private final Scalar RED = new Scalar(255, 0, 0);
	
	// Good Features to Track fields
	private final int MAX_FEATURES = 150;
	private final double QUALITY_LEVEL = 0.01;
	private final double MIN_DISTANCE = 20;
	private final int BLOCK_SIZE = 3;
	private final boolean USE_HARRIS = false;
	private final double k = 0.04;

	// Optical flow fields
	private final Size OPFLOW_WIN_SIZE = new Size(20, 20);
	private final int MAX_LEVEL = 5;
	private final double MIN_EIG_THRESHOLD = 1e-5;
	private TermCriteria opflowcriteria; 
	
	// cornerSubPix fields
	private final Size WIN_SIZE = new Size(15, 15);
	private final Size ZERO_ZONE = new Size(-1, -1);
	
	private ImageIO io;
	
	OpticalFlow() {
		opflowcriteria = new TermCriteria(TermCriteria.EPS + TermCriteria.MAX_ITER, 40, 0.001);
		
		io = new ImageIO();
		io.deletePhotos();
		
	}
	
	OpticalFlowResult getFeatures(Mat checkpointImage, Mat nearImage, Mat farImage, MatOfPoint2f checkpointFeatures) {

		// Checkpoint to near frame

		MatOfPoint2f nearFeatures = new MatOfPoint2f();
		MatOfByte cpNearStatus = new MatOfByte();
		MatOfFloat cpNearError = new MatOfFloat();

		Mat detectMask = nearImage.clone();
		
		Mat nearImageDebug = nearImage.clone();
		
		detectMask.setTo(WHITE);
		List<Point> nearFeaturesList = new ArrayList<>();

		boolean hasCurrent = false;
		double currentSize = checkpointFeatures.size().height;

		if (checkpointFeatures.size().height > 0) {
			hasCurrent = true;
//			Video.calcOpticalFlowPyrLK(checkpointImage, nearImage, checkpointFeatures, nearFeatures, cpNearStatus, cpNearError);
			
			Video.calcOpticalFlowPyrLK(checkpointImage, nearImage, checkpointFeatures, nearFeatures, cpNearStatus, cpNearError, 
				OPFLOW_WIN_SIZE, MAX_LEVEL, opflowcriteria, Video.OPTFLOW_LK_GET_MIN_EIGENVALS, MIN_EIG_THRESHOLD);
			List<Point> checkpointFeaturesList = checkpointFeatures.toList();
			nearFeaturesList = nearFeatures.toList();

			// draw mask for detection
			int index = 0;
			for (Byte item : cpNearStatus.toList()) {
				if (item.intValue() == 1) {
					Core.circle(detectMask, nearFeaturesList.get(index), 10, BLACK, -1);
					Core.circle(nearImageDebug, checkpointFeaturesList.get(index), 2, RED, 1);
					Core.line(nearImageDebug, checkpointFeaturesList.get(index), nearFeaturesList.get(index), RED);
				}
				index++;
			}
		}
		
		io.saveNext(nearImageDebug);

		// detect new features

		MatOfPoint rawNearNewFeatures = new MatOfPoint();
		int toFind = MAX_FEATURES - (int) currentSize;
		if (toFind > 0) {
			Imgproc.goodFeaturesToTrack(nearImage, rawNearNewFeatures, toFind, QUALITY_LEVEL, MIN_DISTANCE, detectMask,
					BLOCK_SIZE, USE_HARRIS, k);
		}
		
		MatOfPoint2f nearNewFeatures = new MatOfPoint2f(rawNearNewFeatures.toArray());
		TermCriteria termCriteria = new TermCriteria(TermCriteria.EPS + TermCriteria.MAX_ITER, 40, 0.001);
		// Imgproc.cornerSubPix(nearImage, nearNewFeatures, WIN_SIZE, ZERO_ZONE, termCriteria);
		
		/*
		 * MatOfKeyPoint rawNearNewFeatures = new MatOfKeyPoint();
		 * detector.detect(nearImage, rawNearNewFeatures, detectMask); if
		 * (rawNearNewFeatures.size().height > 0) { nearNewFeatures =
		 * convert(rawNearNewFeatures); }
		 */

		// // Near frame to far frame

		nearFeatures.push_back(nearNewFeatures);
		nearFeaturesList = nearFeatures.toList();

		MatOfPoint2f farFeatures = new MatOfPoint2f();
		MatOfByte nearFarStatus = new MatOfByte();
		MatOfFloat nearFarError = new MatOfFloat();

		if (nearFeatures.size().height > 0) {
			Video.calcOpticalFlowPyrLK(nearImage, farImage, nearFeatures, farFeatures, nearFarStatus, nearFarError);
			// Video.calcOpticalFlowPyrLK(nearImage, farImage, nearFeatures, farFeatures, nearFarStatus, nearFarError, 
			//		OPFLOW_WIN_SIZE, MAX_LEVEL, opflowcriteria, Video.OPTFLOW_LK_GET_MIN_EIGENVALS, MIN_EIG_THRESHOLD);
				
		}

		List<Point> farFeaturesList = farFeatures.toList();
		List<Point> goodNearFeaturesList = new ArrayList<>();
		List<Point> goodFarFeaturesList = new ArrayList<>();
		List<Integer> badPointsIndex = new ArrayList<>();

		// Find good features, bad features index

		int index = 0;

		List<Byte> cpNearStatusList = null;
		if (hasCurrent) {
			cpNearStatusList = cpNearStatus.toList();
		}

		int current = 0;
		if (nearFarStatus.size().height > 0) {
			for (Byte firstStatus : nearFarStatus.toList()) {
				boolean isGood = false;
				if (index < currentSize) {
					Byte secondStatus = cpNearStatusList.get(index);
					if ((firstStatus.intValue() & secondStatus.intValue()) == 1) {
						// System.out.println("GOOD: " + index);
						current++;
						isGood = true;
					} else {
						// System.out.println("BAD: " + index);
						badPointsIndex.add(Integer.valueOf(index));
					}
				} else {
					// System.out.println("NEW: " + index);
					isGood = true;
				}

				if (isGood) {
					goodNearFeaturesList.add(nearFeaturesList.get(index));
					goodFarFeaturesList.add(farFeaturesList.get(index));
				}
				index++;
			}
		}

		MatOfPoint2f goodNearFeatures = new MatOfPoint2f();
		MatOfPoint2f goodFarFeatures = new MatOfPoint2f();
		goodNearFeatures.fromList(goodNearFeaturesList);
		goodFarFeatures.fromList(goodFarFeaturesList);

		OpticalFlowResult result = new OpticalFlowResult(goodNearFeatures, goodFarFeatures, badPointsIndex, current);
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
