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
	
	
	/**
	 * Performs optical flow between two consecutive images.
	 * 
	 * @param previousImage
	 * @param nextImage
	 * @param previousFeatures
	 * @return
	 */
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
				Core.circle(imageDebug, rightFeaturesList.get(index), 2, RED, -1);
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
	
	/**
	 * Used for asynchronous feature updates. Keeps the features in a single list.
	 * @param previousImage
	 * @param nextImage
	 * @param flowingFeatures
	 * @param checkpointSize
	 * @param isGoodFeatures
	 */
	AsyncOpticalFlowResult unfilteredFlow(Mat previousImage, Mat nextImage, List<Point> flowingFeaturesList, 
			int checkpointSize, List<Boolean> isGoodFeatures) {
		
		MatOfPoint2f flowingFeatures = new MatOfPoint2f(flowingFeaturesList.toArray(new Point[0]));
		
		/* Create mask */
		Mat detectMask = previousImage.clone();
		detectMask.setTo(WHITE);
		
		for (int i = 0; i < flowingFeaturesList.size(); i++) {
			boolean isGoodFeature = true;
			if (i < checkpointSize) {
				isGoodFeature = isGoodFeatures.get(i);
			}
			if (isGoodFeature) {
				Core.circle(detectMask, flowingFeaturesList.get(i), 10, BLACK, -1);
			}
		}
		
		
		/* Detect new features*/
		double previousSize = flowingFeatures.size().height;
		MatOfPoint rawNewFeatures = new MatOfPoint();
		int toFind = MAX_FEATURES - (int) previousSize;
		
		if (toFind > 0) {
			Imgproc.goodFeaturesToTrack(nextImage, rawNewFeatures, toFind, QUALITY_LEVEL, MIN_DISTANCE, detectMask,
					BLOCK_SIZE, USE_HARRIS, k);
		}
		MatOfPoint2f nextNewFeatures = new MatOfPoint2f(rawNewFeatures.toArray());
		
		
		/* Optical flow */
		MatOfPoint2f leftFeatures = new MatOfPoint2f();
		leftFeatures.push_back(flowingFeatures);
		leftFeatures.push_back(nextNewFeatures);
		MatOfPoint2f rightFeatures = new MatOfPoint2f();
		MatOfByte statuses = new MatOfByte();
		MatOfFloat error = new MatOfFloat();
		
		if (leftFeatures.size().height > 0) {
			TermCriteria opflowcriteria = new TermCriteria(TermCriteria.EPS + TermCriteria.MAX_ITER, 40, 0.001);
			Video.calcOpticalFlowPyrLK(previousImage, nextImage, leftFeatures, rightFeatures, statuses, error, 
				OPFLOW_WIN_SIZE, MAX_LEVEL, opflowcriteria, Video.OPTFLOW_LK_GET_MIN_EIGENVALS, MIN_EIG_THRESHOLD);	
		}
		
		
		/* Filter bad features */
		List<Point> leftFeaturesList = leftFeatures.toList();
		List<Point> rightFeaturesList = rightFeatures.toList();
		List<Point> filteredFeatures = new ArrayList<>();
		List<Byte> statusList = statuses.toList();
		Mat imageDebug = nextImage.clone();
				
		for (int i = 0; i < rightFeatures.size().height; i++) {
			int status = statusList.get(i).intValue();
			if (i < checkpointSize) {
				boolean oldValue = isGoodFeatures.get(i);
				boolean newValue = oldValue && (status == 1);
				isGoodFeatures.set(i, newValue);
				
				// Checkpoint features get added regardless if good or bad
				filteredFeatures.add( rightFeaturesList.get(i) );
			
			} else if (status == 1) { 
				// Remove new features that are bad
				filteredFeatures.add( rightFeaturesList.get(i) );		
			}
		}
		
		
		int index = 0;
		for (Byte item : statuses.toList()) {	
			if (item.intValue() == 1) {
				Core.circle(imageDebug, rightFeaturesList.get(index), 2, RED, 1);
				Core.line(imageDebug, leftFeaturesList.get(index), rightFeaturesList.get(index), RED);
			} 
			index++;
		}
		io.saveNext(imageDebug);

		AsyncOpticalFlowResult result = new AsyncOpticalFlowResult(filteredFeatures, isGoodFeatures);
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
