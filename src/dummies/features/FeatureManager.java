package dummies.features;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.Core.MinMaxLocResult;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Size;
import org.opencv.features2d.DMatch;
import org.opencv.features2d.KeyPoint;

import commondata.PointDouble;

public class FeatureManager {

	private static final String TAG = "Feature Manager";

	private static boolean DEBUG_MODE = true;

	// Boolean things (how does one even name this)
	private final boolean USE_SCALE = true;
	private final boolean SWAP_IMAGES = true;

	private static final int LS_TRIANGULATION = 0;
	private static final int ITER_LS_TRIANGULATION = 1;
	private static final int OPENCV_TRIANGULATION = 2;

	private static final int TRIANGULATION_METHOD = LS_TRIANGULATION;

	// Image Capture fields
	private int frames = 0;
	private final int FRAME_INTERVAL = 0;
	private boolean framesReady = false;
	private List<Mat> images;

	// Optical flow fields
	private Mat checkpointImage;
	private Mat cameraImage;
	private MatOfPoint2f checkpointFeatures;
	private OpticalFlow opticalFlow;

	// Triangulation fields
	private Size imageSize;
	// DANGER Rot1, Rot2, T1, and T2 are modified in DecomposeEtoRandT()
	private Mat cameraMatrix, distCoeffs, Rot2, Rot1, T2, T1;
	private Mat R1, R2, P1, P2, Q;
	private Mat points4D1, points4D2;
	private Mat F, E, W;
	private Mat u, w, vt;
	private Mat nullMatF, tempMat;

	// Status Checking
	public static int STEP_VALID_UPDATE = 0;
	public static int STEP_IMAGE_CAPTURE = 1;
	public static int STEP_OPTICAL_FLOW = 2;
	public static int STEP_ESSENTIAL_MATRIX = 3;
	public static int STEP_TRIANGULATION = 4;

	public static int CURRENT_STEP;

	// Valid Projection

	public static int ROT_1 = 0;
	public static int ROT_2 = 1;
	public static int TRAN_1 = 2;
	public static int TRAN_2 = 3;

	public static int VALID_ROTATION;
	public static int VALID_TRANSLATION;

	private Mat validRot, validT;

	private double xScale;
	private double zScale;
	boolean first = true;

	// DANGER Gets assigned in triangulatePoints(). Too lazy to return properly.
	private double reprojErr1, reprojErr2;

	public FeatureManager() {

		opticalFlow = new OpticalFlow();
		checkpointFeatures = new MatOfPoint2f();
		checkpointImage = new Mat();
		images = new ArrayList<>();

		nullMatF = Mat.zeros(0, 0, CvType.CV_64F);
		initRectifyVariables();
	}

	private void initRectifyVariables() {
		// INITIALIZATION FOR STEREORECTIFY()

		// INPUT VARIABLES

		cameraMatrix = Mat.zeros(3, 3, CvType.CV_64F);
		distCoeffs = Mat.zeros(5, 1, CvType.CV_64F);
		imageSize = new Size(240, 320);

		Rot1 = Mat.eye(3, 3, CvType.CV_64F);
		Rot2 = Mat.eye(3, 3, CvType.CV_64F);
		T1 = Mat.ones(3, 1, CvType.CV_64F);
		T2 = Mat.ones(3, 1, CvType.CV_64F);

		// CALIBRATION RESULTS FOR 320 x 240

		cameraMatrix.put(0, 0, 287.484405747163);
		cameraMatrix.put(0, 2, 119.5);
		cameraMatrix.put(1, 1, 287.484405747163);
		cameraMatrix.put(1, 2, 159.5);
		cameraMatrix.put(2, 2, 1);

		distCoeffs.put(0, 0, 0.1831508618865668);
		distCoeffs.put(1, 0, -0.8391135375141514);
		distCoeffs.put(2, 0, 0);
		distCoeffs.put(3, 0, 0);
		distCoeffs.put(4, 0, 1.067914298622483);

		// OUTPUT VARIABLES

		R1 = Mat.zeros(3, 3, CvType.CV_64F);
		R2 = Mat.zeros(3, 3, CvType.CV_64F);
		P1 = Mat.zeros(3, 4, CvType.CV_64F);
		P2 = Mat.zeros(3, 4, CvType.CV_64F);
		Q = Mat.zeros(4, 4, CvType.CV_64F);
	}

	/** For Opencv has failed us yet again **/
	private List<KeyPoint> convertMatOfPoint2fToListOfKeyPoints(MatOfPoint2f ps) {
		List<KeyPoint> kps = new ArrayList<>();

		for (int i = 0; i < ps.rows(); ++i) {
			double pt[] = ps.get(i, 0);
			kps.add(new KeyPoint((float) pt[0], (float) pt[1], 1.0f));
		}

		return kps;
	}

	private List<Point> checkpointFeaturesList;
	private List<Point> flowingFeatures;
	private List<Boolean> isGoodFeatures;
	private int currentSize = 0;
	private Mat prevImage;

	public void flowImage(Mat currentImage) {
		AsyncOpticalFlowResult opflowresult = opticalFlow.unfilteredFlow(prevImage, currentImage, flowingFeatures,
				checkpointFeaturesList.size(), isGoodFeatures);
		currentImage.copyTo(prevImage);
		isGoodFeatures = opflowresult.getIsGoodFeatures();
		flowingFeatures = opflowresult.getFlowingFeatures();
		// System.out.println("Flowing features: " + flowingFeatures.size());
		// System.out.println("Is Good Features: " + isGoodFeatures.size());
	}

	private void asyncExit(List<Point> nextNewFeatures) {
		if (first) {
			first = false;
			currentSize = (int) checkpointFeaturesList.size();
			checkpointFeaturesList.addAll(nextNewFeatures);
		}

		flowingFeatures = new ArrayList<>();
		for (Point point : checkpointFeaturesList) {
			flowingFeatures.add(new Point(point.x, point.y));
		}
		isGoodFeatures.clear();

		for (int i = 0; i < flowingFeatures.size(); i++) {
			isGoodFeatures.add(true);
		}

		System.out.println("Async exit. ENDING CPFL " + checkpointFeaturesList.size());
	}

	private void nullExit(List<Point> nextNewFeatures) {
		flowingFeatures = new ArrayList<>();
		for (Point point : checkpointFeaturesList) {
			flowingFeatures.add(new Point(point.x, point.y));
		}
		isGoodFeatures.clear();

		for (int i = 0; i < flowingFeatures.size(); i++) {
			isGoodFeatures.add(true);
		}

		System.out.println("Null exit. ENDING CPFL " + checkpointFeaturesList.size());
	}

	public FeatureUpdate getAsyncFeatureUpdate(Mat currentImage, double translationX, double translationZ,
			PointDouble cameraPosition) {
		/* For first call. */
		if (prevImage == null) {
			prevImage = new Mat(); // not sure if needed
			checkpointFeaturesList = new ArrayList<>();
			flowingFeatures = new ArrayList<>();
			isGoodFeatures = new ArrayList<>();
			currentImage.copyTo(prevImage);
			return FeatureScaler.getFeatureScaler().getScaledFeatureUpdate(null, cameraPosition);
		}

		/* Perform final optical flow. */
		this.flowImage(currentImage);

		/* Group features accordingly. */
		List<Integer> badPointsIndex = new ArrayList<>();
		List<Point> nextNewFeatures = new ArrayList<>();
		List<Point> goodCheckpointFeatures = new ArrayList<>();
		List<Point> goodFlowedCheckpointFeatures = new ArrayList<>();

		int goodCurrents = 0;

		for (int i = 0; i < flowingFeatures.size(); i++) {
			// Split off features into checkpoint and next new features.
			if (i < checkpointFeaturesList.size()) {
				// Prepare good current and new features for triangulation
				if (isGoodFeatures.get(i)) {
					if (i < currentSize) {
						goodCurrents++;

					} else {

					}
					goodCheckpointFeatures.add(checkpointFeaturesList.get(i));
					goodFlowedCheckpointFeatures.add(flowingFeatures.get(i));

					// Add bad point indices of current features.
				} else if (i < currentSize) {
					badPointsIndex.add(Integer.valueOf(i));

				}

			} else {
				nextNewFeatures.add(flowingFeatures.get(i));

			}
		}
		System.out.println("current size " + currentSize);
		System.out.println("gcf " + goodCheckpointFeatures.size());
		System.out.println("good currents " + goodCurrents);
		/* Triangulation */
		// WARNING. Copy paste mode. Changes on function below must be reflected
		// here.
		// TODO convert to function
		MatOfPoint2f goodOld = new MatOfPoint2f();
		MatOfPoint2f goodNew = new MatOfPoint2f();
		goodOld.fromList(goodCheckpointFeatures);
		goodNew.fromList(goodFlowedCheckpointFeatures);

		// sketchy code ahead
		if (goodOld.empty() || goodNew.empty()) {
			asyncExit(nextNewFeatures);
			return FeatureScaler.getFeatureScaler().getScaledFeatureUpdate(null, cameraPosition);
		}
		FMatResult fMatResult = null;
		points4D1 = new Mat();

		if (!goodOld.empty() && !goodNew.empty()) {

			// does this work
			if (SWAP_IMAGES) {
				MatOfPoint2f temp = goodOld;
				goodOld = goodNew;
				goodNew = temp;
			}

			// SOLVING FOR THE ROTATION AND TRANSLATION MATRICES

			// GETTING THE FUNDAMENTAL MATRIX

			// There is a case that fundamental matrix is not found

			List<KeyPoint> kpGoodOld = new ArrayList<KeyPoint>(), kpGoodNew = new ArrayList<KeyPoint>();

			kpGoodOld = convertMatOfPoint2fToListOfKeyPoints(goodOld);
			kpGoodNew = convertMatOfPoint2fToListOfKeyPoints(goodNew);

			CURRENT_STEP = this.STEP_ESSENTIAL_MATRIX;
			fMatResult = getFundamentalMat(kpGoodOld, kpGoodNew, badPointsIndex, goodCurrents);
			F = fMatResult.F;

			// SOBRANG HASSLE
			// A bit scary
			goodOld = fMatResult.superGoodPoints1;
			goodNew = fMatResult.superGoodPoints2;
			tempMat = nullMatF.clone();
			E = nullMatF.clone();

			// GETTING THE ESSENTIAL MATRIX
			Core.gemm(cameraMatrix.t(), F, 1, nullMatF, 0, tempMat);
			Core.gemm(tempMat, cameraMatrix, 1, nullMatF, 0, E);

			if (Math.abs(Core.determinant(E)) > 1e-07) {
				if (this.DEBUG_MODE)
					System.out.println("det(E) != 0 : " + Core.determinant(E));
				P2 = Mat.zeros(3, 4, CvType.CV_64F);

				nullExit(nextNewFeatures);
				return FeatureScaler.getFeatureScaler().getScaledFeatureUpdate(null, cameraPosition);
			}

			if (!decomposeEtoRandT(E))

				return FeatureScaler.getFeatureScaler().getScaledFeatureUpdate(null, cameraPosition);

			if (Core.determinant(R1) + 1.0 < 1e-09) {
				// according to
				// http://en.wikipedia.org/wiki/Essential_matrix#Showing_that_it_is_valid

				if (this.DEBUG_MODE)
					System.out.println("det(R) == -1 [" + Core.determinant(R1) + "]: flip E's sign");
				E = E.mul(Mat.ones(E.size(), E.type()), -1);

				if (!decomposeEtoRandT(E))
					nullExit(nextNewFeatures);
				return FeatureScaler.getFeatureScaler().getScaledFeatureUpdate(null, cameraPosition);
			}

			P1.put(0, 0, 1, 0, 0, 0);
			P1.put(1, 0, 0, 1, 0, 0);
			P1.put(2, 0, 0, 0, 1, 0);

			if (!checkCoherentRotation(Rot1)) {
				P2 = Mat.zeros(3, 4, CvType.CV_64F);
				nullExit(nextNewFeatures);
				return FeatureScaler.getFeatureScaler().getScaledFeatureUpdate(null, cameraPosition);
			}

			CURRENT_STEP = this.STEP_TRIANGULATION;

			this.VALID_ROTATION = this.ROT_1;
			this.VALID_TRANSLATION = this.TRAN_1;
			validRot = Rot1;
			validT = T1;

			// Combination 1
			P2.put(0, 0, Rot1.get(0, 0)[0], Rot1.get(0, 1)[0], Rot1.get(0, 2)[0], T1.get(0, 0)[0]);
			P2.put(1, 0, Rot1.get(1, 0)[0], Rot1.get(1, 1)[0], Rot1.get(1, 2)[0], T1.get(1, 0)[0]);
			P2.put(2, 0, Rot1.get(2, 0)[0], Rot1.get(2, 1)[0], Rot1.get(2, 2)[0], T1.get(2, 0)[0]);

			points4D1 = triangulatePoints(goodOld, goodNew, cameraMatrix, P1, P2, true);
			points4D2 = triangulatePoints(goodNew, goodOld, cameraMatrix, P2, P1, false);

			if (!testTriangulation(points4D2, P1) || !testTriangulation(points4D1, P2) || reprojErr1 > 100
					|| reprojErr2 > 100) { // TODO: Test Triangulation,
				this.VALID_ROTATION = this.ROT_1;
				this.VALID_TRANSLATION = this.TRAN_2;
				validRot = Rot1;
				validT = T2;

				// Combination 2
				P2.put(0, 0, Rot1.get(0, 0)[0], Rot1.get(0, 1)[0], Rot1.get(0, 2)[0], T2.get(0, 0)[0]);
				P2.put(1, 0, Rot1.get(1, 0)[0], Rot1.get(1, 1)[0], Rot1.get(1, 2)[0], T2.get(1, 0)[0]);
				P2.put(2, 0, Rot1.get(2, 0)[0], Rot1.get(2, 1)[0], Rot1.get(2, 2)[0], T2.get(2, 0)[0]);

				points4D1 = triangulatePoints(goodOld, goodNew, cameraMatrix, P1, P2, true);
				points4D2 = triangulatePoints(goodNew, goodOld, cameraMatrix, P2, P1, false);

				if (!testTriangulation(points4D2, P1) || !testTriangulation(points4D1, P2) || reprojErr1 > 100
						|| reprojErr2 > 100) { // TODO: Test Triangulation
					if (!checkCoherentRotation(Rot2)) {
						P2 = Mat.zeros(3, 4, CvType.CV_64F);

						return FeatureScaler.getFeatureScaler().getScaledFeatureUpdate(null, cameraPosition);
					}
					this.VALID_ROTATION = this.ROT_2;
					this.VALID_TRANSLATION = this.TRAN_1;
					validRot = Rot2;
					validT = T1;

					// Combination 3
					P2.put(0, 0, Rot2.get(0, 0)[0], Rot2.get(0, 1)[0], Rot2.get(0, 2)[0], T1.get(0, 0)[0]);
					P2.put(1, 0, Rot2.get(1, 0)[0], Rot2.get(1, 1)[0], Rot2.get(1, 2)[0], T1.get(1, 0)[0]);
					P2.put(2, 0, Rot2.get(2, 0)[0], Rot2.get(2, 1)[0], Rot2.get(2, 2)[0], T1.get(2, 0)[0]);

					points4D1 = triangulatePoints(goodOld, goodNew, cameraMatrix, P1, P2, true);
					points4D2 = triangulatePoints(goodNew, goodOld, cameraMatrix, P2, P1, false);
					if (!testTriangulation(points4D2, P1) || !testTriangulation(points4D1, P2) || reprojErr1 > 100
							|| reprojErr2 > 100) { // TODO: Test Triangulation
						this.VALID_ROTATION = this.ROT_2;
						this.VALID_TRANSLATION = this.TRAN_2;
						validRot = Rot2;
						validT = T2;

						// Combination 4
						P2.put(0, 0, Rot2.get(0, 0)[0], Rot2.get(0, 1)[0], Rot2.get(0, 2)[0], T2.get(0, 0)[0]);
						P2.put(1, 0, Rot2.get(1, 0)[0], Rot2.get(1, 1)[0], Rot2.get(1, 2)[0], T2.get(1, 0)[0]);
						P2.put(2, 0, Rot2.get(2, 0)[0], Rot2.get(2, 1)[0], Rot2.get(2, 2)[0], T2.get(2, 0)[0]);

						points4D1 = triangulatePoints(goodOld, goodNew, cameraMatrix, P1, P2, true);
						points4D2 = triangulatePoints(goodNew, goodOld, cameraMatrix, P2, P1, false);
						if (!testTriangulation(points4D2, P1) || !testTriangulation(points4D1, P2) || reprojErr1 > 100
								|| reprojErr2 > 100) { // TODO: Test
														// Triangulation
							// Triangulation failed.
							nullExit(nextNewFeatures);
							return FeatureScaler.getFeatureScaler().getScaledFeatureUpdate(null, cameraPosition);
						}
					}
				}
			}
		}

		FeatureUpdate update = new FeatureUpdate();
		List<PointDouble> currentPoints = new ArrayList<>();
		List<PointDouble> newPoints = new ArrayList<>();

		// Appending additional bad points from Fundamental Matrix calculation
		List<Integer> badPoints = badPointsIndex;
		List<Integer> additionalBadPoints = fMatResult.additionalBadPoints;
		{
			List<Integer> finalBadPoints = new ArrayList<>();

			int badPointsDuplicatesCount = 0;

			for (Integer index : badPoints) {
				if (finalBadPoints.contains(index)) {
					badPointsDuplicatesCount++;
				} else {
					finalBadPoints.add(index);
				}
			}
			List<Integer> noDuplicatesAdditionalBadPoints = new ArrayList<>();

			int additionalBadPointsDuplicatesCount = 0;
			for (Integer index : additionalBadPoints) {
				if (noDuplicatesAdditionalBadPoints.contains(index)) {
					additionalBadPointsDuplicatesCount++;
				} else {
					noDuplicatesAdditionalBadPoints.add(index);
				}
			}

			for (Integer index : noDuplicatesAdditionalBadPoints) {
				if (finalBadPoints.contains(index)) {
					badPointsDuplicatesCount++;
				} else {
					finalBadPoints.add(index);
				}
			}

			badPoints = finalBadPoints;
			Collections.sort(badPoints);
			// System.out.println(badPoints);

			goodCurrents = goodCurrents - fMatResult.additionalBadPoints.size() + additionalBadPointsDuplicatesCount
					+ badPointsDuplicatesCount;
			// System.out.println(currentSize + " " +
			// additionalBadPointsDuplicatesCount + " " +
			// badPointsDuplicatesCount);
		}
		// if (this.VALID_ROTATION == this.ROT_1)
		// System.out.println("Rotation Matrix 1 is Valid.");
		// else
		// System.out.println("Rotation Matrix 2 is Valid.");
		// if (this.VALID_TRANSLATION == this.TRAN_1)
		// System.out.println("Translation Vector 1 is Valid.");
		// else
		// System.out.println("Translation Vector 2 is Valid.");

		Mat translationMatrix = T2;
		if (this.VALID_TRANSLATION == this.TRAN_1)
			translationMatrix = T1;

		// double xScale = translationX / translationMatrix.get(0, 0)[0];
		// double zScale = translationZ / translationMatrix.get(2, 0)[0];

		if (!USE_SCALE) { // HAHA WOT.
			// xScale = 1;
			// zScale = 1;
		}
		Mat points4D = new Mat();
		switch (this.TRIANGULATION_METHOD) {
		case LS_TRIANGULATION:
			points4D = points4D1;
			break;

		case ITER_LS_TRIANGULATION:
			P2.put(0, 0, validRot.get(0, 0)[0], validRot.get(0, 1)[0], validRot.get(0, 2)[0], validT.get(0, 0)[0]);
			P2.put(1, 0, validRot.get(1, 0)[0], validRot.get(1, 1)[0], validRot.get(1, 2)[0], validT.get(1, 0)[0]);
			P2.put(2, 0, validRot.get(2, 0)[0], validRot.get(2, 1)[0], validRot.get(2, 2)[0], validT.get(2, 0)[0]);

			points4D = points4D1 = triangulatePoints(goodOld, goodNew, cameraMatrix, P1, P2, false);
			break;
		case OPENCV_TRIANGULATION:
			Calib3d.triangulatePoints(P1, P2, goodOld, goodNew, points4D);
			break;
		}

		for (int i = 0; i < points4D.cols(); i++) {
			double w = points4D.get(3, i)[0];
			double x = points4D.get(0, i)[0] / w;
			double y = points4D.get(1, i)[0] / w;
			double z = points4D.get(2, i)[0] / w;

			PointDouble point = new PointDouble(x, z);
			// System.out.println(point);
			if (i < goodCurrents) {
				currentPoints.add(point);
			} else {
				newPoints.add(point);
			}
		}

		update.setCurrentPoints(currentPoints);
		update.setBadPointsIndex(badPoints);
		update.setNewPoints(newPoints);

		/* Initialize variables for next cycle */
		fMatResult.superGoodPoints1.copyTo(checkpointFeatures);
		checkpointFeaturesList = new ArrayList<>(checkpointFeatures.toList());
		currentSize = checkpointFeaturesList.size();
		checkpointFeaturesList.addAll(nextNewFeatures);

		flowingFeatures = new ArrayList<>();
		for (Point point : checkpointFeaturesList) {
			flowingFeatures.add(new Point(point.x, point.y));
		}
		isGoodFeatures.clear();

		for (int i = 0; i < flowingFeatures.size(); i++) {
			isGoodFeatures.add(true);
		}
		frames++;

		if (this.DEBUG_MODE) {
			System.out.println(update.getCurrentPoints().size() + update.getBadPointsIndex().size());
			System.out.println(update.getCurrentPoints().size() + update.getNewPoints().size());
		}

		CURRENT_STEP = this.STEP_VALID_UPDATE;
		// System.out.println(update);
		return FeatureScaler.getFeatureScaler().getScaledFeatureUpdate(update, cameraPosition);
	}

	public FeatureUpdate getFeatureUpdate(Mat currentImage, double translationX, double translationZ,
			PointDouble cameraPosition) {
		if (!framesReady) {
			Mat toAdd = new Mat();
			currentImage.copyTo(toAdd);
			images.add(toAdd);
			if (frames == FRAME_INTERVAL + 1) {
				images.get(0).copyTo(checkpointImage);
				images.remove(0);
				framesReady = true;
			}
			frames++;
			CURRENT_STEP = this.STEP_IMAGE_CAPTURE;
			return FeatureScaler.getFeatureScaler().getScaledFeatureUpdate(null, cameraPosition);
		}
		CURRENT_STEP = this.STEP_OPTICAL_FLOW;

		Mat nearImage = new Mat();
		images.get(0).copyTo(nearImage);
		Mat farImage = new Mat();
		currentImage.copyTo(farImage);
		images.add(farImage);
		images.remove(0);

		OpticalFlowResult opflowresult = opticalFlow.getFeatures(checkpointImage, nearImage, farImage,
				checkpointFeatures);
		// OpticalFlowResult opflowresult =
		// opticalFlow.getFeatures(checkpointImage, nearImage,
		// checkpointFeatures);

		MatOfPoint2f goodOld = opflowresult.getNearFeatures();
		MatOfPoint2f goodNew = opflowresult.getFarFeatures();

		FMatResult fMatResult = null;
		points4D1 = new Mat();

		if (!goodOld.empty() && !goodNew.empty()) {
			System.out.println("has good old");
			// does this work
			if (SWAP_IMAGES) {
				MatOfPoint2f temp = goodOld;
				goodOld = goodNew;
				goodNew = temp;
			}

			// SOLVING FOR THE ROTATION AND TRANSLATION MATRICES

			// GETTING THE FUNDAMENTAL MATRIX

			// There is a case that fundamental matrix is not found

			List<KeyPoint> kpGoodOld = new ArrayList<KeyPoint>(), kpGoodNew = new ArrayList<KeyPoint>();

			kpGoodOld = convertMatOfPoint2fToListOfKeyPoints(goodOld);
			kpGoodNew = convertMatOfPoint2fToListOfKeyPoints(goodNew);

			CURRENT_STEP = this.STEP_ESSENTIAL_MATRIX;
			fMatResult = getFundamentalMat(kpGoodOld, kpGoodNew, opflowresult.getBadPointsIndex(),
					opflowresult.getCurrentSize());
			F = fMatResult.F;

			// SOBRANG HASSLE
			// A bit scary
			goodOld = fMatResult.superGoodPoints1;
			goodNew = fMatResult.superGoodPoints2;

			tempMat = nullMatF.clone();
			E = nullMatF.clone();

			// GETTING THE ESSENTIAL MATRIX
			Core.gemm(cameraMatrix.t(), F, 1, nullMatF, 0, tempMat);
			Core.gemm(tempMat, cameraMatrix, 1, nullMatF, 0, E);

			if (Math.abs(Core.determinant(E)) > 1e-07) {
				if (this.DEBUG_MODE)
					System.out.println("det(E) != 0 : " + Core.determinant(E));
				P2 = Mat.zeros(3, 4, CvType.CV_64F);
				return FeatureScaler.getFeatureScaler().getScaledFeatureUpdate(null, cameraPosition);
			}

			if (!decomposeEtoRandT(E))

				return FeatureScaler.getFeatureScaler().getScaledFeatureUpdate(null, cameraPosition);

			if (Core.determinant(R1) + 1.0 < 1e-09) {
				// according to
				// http://en.wikipedia.org/wiki/Essential_matrix#Showing_that_it_is_valid

				if (this.DEBUG_MODE)
					System.out.println("det(R) == -1 [" + Core.determinant(R1) + "]: flip E's sign");
				E = E.mul(Mat.ones(E.size(), E.type()), -1);

				if (!decomposeEtoRandT(E))

					return FeatureScaler.getFeatureScaler().getScaledFeatureUpdate(null, cameraPosition);
			}

			P1.put(0, 0, 1, 0, 0, 0);
			P1.put(1, 0, 0, 1, 0, 0);
			P1.put(2, 0, 0, 0, 1, 0);

			if (!checkCoherentRotation(Rot1)) {
				P2 = Mat.zeros(3, 4, CvType.CV_64F);
				return FeatureScaler.getFeatureScaler().getScaledFeatureUpdate(null, cameraPosition);
			}

			CURRENT_STEP = this.STEP_TRIANGULATION;

			this.VALID_ROTATION = this.ROT_1;
			this.VALID_TRANSLATION = this.TRAN_1;
			validRot = Rot1;
			validT = T1;

			// Combination 1
			P2.put(0, 0, Rot1.get(0, 0)[0], Rot1.get(0, 1)[0], Rot1.get(0, 2)[0], T1.get(0, 0)[0]);
			P2.put(1, 0, Rot1.get(1, 0)[0], Rot1.get(1, 1)[0], Rot1.get(1, 2)[0], T1.get(1, 0)[0]);
			P2.put(2, 0, Rot1.get(2, 0)[0], Rot1.get(2, 1)[0], Rot1.get(2, 2)[0], T1.get(2, 0)[0]);

			points4D1 = triangulatePoints(goodOld, goodNew, cameraMatrix, P1, P2, true);
			points4D2 = triangulatePoints(goodNew, goodOld, cameraMatrix, P2, P1, false);

			if (!testTriangulation(points4D2, P1) || !testTriangulation(points4D1, P2) || reprojErr1 > 100
					|| reprojErr2 > 100) { // TODO: Test Triangulation,
				this.VALID_ROTATION = this.ROT_1;
				this.VALID_TRANSLATION = this.TRAN_2;
				validRot = Rot1;
				validT = T2;

				// Combination 2
				P2.put(0, 0, Rot1.get(0, 0)[0], Rot1.get(0, 1)[0], Rot1.get(0, 2)[0], T2.get(0, 0)[0]);
				P2.put(1, 0, Rot1.get(1, 0)[0], Rot1.get(1, 1)[0], Rot1.get(1, 2)[0], T2.get(1, 0)[0]);
				P2.put(2, 0, Rot1.get(2, 0)[0], Rot1.get(2, 1)[0], Rot1.get(2, 2)[0], T2.get(2, 0)[0]);

				points4D1 = triangulatePoints(goodOld, goodNew, cameraMatrix, P1, P2, true);
				points4D2 = triangulatePoints(goodNew, goodOld, cameraMatrix, P2, P1, false);

				if (!testTriangulation(points4D2, P1) || !testTriangulation(points4D1, P2) || reprojErr1 > 100
						|| reprojErr2 > 100) { // TODO: Test Triangulation
					if (!checkCoherentRotation(Rot2)) {
						P2 = Mat.zeros(3, 4, CvType.CV_64F);

						return FeatureScaler.getFeatureScaler().getScaledFeatureUpdate(null, cameraPosition);
					}
					this.VALID_ROTATION = this.ROT_2;
					this.VALID_TRANSLATION = this.TRAN_1;
					validRot = Rot2;
					validT = T1;

					// Combination 3
					P2.put(0, 0, Rot2.get(0, 0)[0], Rot2.get(0, 1)[0], Rot2.get(0, 2)[0], T1.get(0, 0)[0]);
					P2.put(1, 0, Rot2.get(1, 0)[0], Rot2.get(1, 1)[0], Rot2.get(1, 2)[0], T1.get(1, 0)[0]);
					P2.put(2, 0, Rot2.get(2, 0)[0], Rot2.get(2, 1)[0], Rot2.get(2, 2)[0], T1.get(2, 0)[0]);

					points4D1 = triangulatePoints(goodOld, goodNew, cameraMatrix, P1, P2, true);
					points4D2 = triangulatePoints(goodNew, goodOld, cameraMatrix, P2, P1, false);
					if (!testTriangulation(points4D2, P1) || !testTriangulation(points4D1, P2) || reprojErr1 > 100
							|| reprojErr2 > 100) { // TODO: Test Triangulation
						this.VALID_ROTATION = this.ROT_2;
						this.VALID_TRANSLATION = this.TRAN_2;
						validRot = Rot2;
						validT = T2;

						// Combination 4
						P2.put(0, 0, Rot2.get(0, 0)[0], Rot2.get(0, 1)[0], Rot2.get(0, 2)[0], T2.get(0, 0)[0]);
						P2.put(1, 0, Rot2.get(1, 0)[0], Rot2.get(1, 1)[0], Rot2.get(1, 2)[0], T2.get(1, 0)[0]);
						P2.put(2, 0, Rot2.get(2, 0)[0], Rot2.get(2, 1)[0], Rot2.get(2, 2)[0], T2.get(2, 0)[0]);

						points4D1 = triangulatePoints(goodOld, goodNew, cameraMatrix, P1, P2, true);
						points4D2 = triangulatePoints(goodNew, goodOld, cameraMatrix, P2, P1, false);
						if (!testTriangulation(points4D2, P1) || !testTriangulation(points4D1, P2) || reprojErr1 > 100
								|| reprojErr2 > 100) { // TODO: Test
														// Triangulation
							// Triangulation failed.

							return FeatureScaler.getFeatureScaler().getScaledFeatureUpdate(null, cameraPosition);
						}
					}
				}
			}
		}

		FeatureUpdate update = new FeatureUpdate();
		List<PointDouble> currentPoints = new ArrayList<>();
		List<PointDouble> newPoints = new ArrayList<>();

		// Appending additional bad points from Fundamental Matrix calculation
		List<Integer> badPoints = opflowresult.getBadPointsIndex();
		List<Integer> additionalBadPoints = fMatResult.additionalBadPoints;
		int currentSize;
		{
			List<Integer> finalBadPoints = new ArrayList<>();

			int badPointsDuplicatesCount = 0;

			for (Integer index : badPoints) {
				if (finalBadPoints.contains(index)) {
					badPointsDuplicatesCount++;
				} else {
					finalBadPoints.add(index);
				}
			}
			List<Integer> noDuplicatesAdditionalBadPoints = new ArrayList<>();

			int additionalBadPointsDuplicatesCount = 0;
			for (Integer index : additionalBadPoints) {
				if (noDuplicatesAdditionalBadPoints.contains(index)) {
					additionalBadPointsDuplicatesCount++;
				} else {
					noDuplicatesAdditionalBadPoints.add(index);
				}
			}

			for (Integer index : noDuplicatesAdditionalBadPoints) {
				if (finalBadPoints.contains(index)) {
					badPointsDuplicatesCount++;
				} else {
					finalBadPoints.add(index);
				}
			}

			badPoints = finalBadPoints;
			Collections.sort(badPoints);
			// System.out.println(badPoints);

			currentSize = (int) opflowresult.getCurrentSize() - fMatResult.additionalBadPoints.size()
					+ additionalBadPointsDuplicatesCount + badPointsDuplicatesCount;

			System.out.println(opflowresult.getCurrentSize() + " " + additionalBadPointsDuplicatesCount + " "
					+ badPointsDuplicatesCount);
		}
		// if (this.VALID_ROTATION == this.ROT_1)
		// System.out.println("Rotation Matrix 1 is Valid.");
		// else
		// System.out.println("Rotation Matrix 2 is Valid.");
		// if (this.VALID_TRANSLATION == this.TRAN_1)
		// System.out.println("Translation Vector 1 is Valid.");
		// else
		// System.out.println("Translation Vector 2 is Valid.");

		Mat translationMatrix = T2;
		if (this.VALID_TRANSLATION == this.TRAN_1)
			translationMatrix = T1;

		double xScale = translationX / translationMatrix.get(0, 0)[0];
		double zScale = translationZ / translationMatrix.get(2, 0)[0];

		if (!USE_SCALE) { // HAHA WOT.
			xScale = 1;
			zScale = 1;
		}

		Mat points4D = new Mat();
		switch (this.TRIANGULATION_METHOD) {
		case LS_TRIANGULATION:
			points4D = points4D1;
			break;

		case ITER_LS_TRIANGULATION:
			P2.put(0, 0, validRot.get(0, 0)[0], validRot.get(0, 1)[0], validRot.get(0, 2)[0], validT.get(0, 0)[0]);
			P2.put(1, 0, validRot.get(1, 0)[0], validRot.get(1, 1)[0], validRot.get(1, 2)[0], validT.get(1, 0)[0]);
			P2.put(2, 0, validRot.get(2, 0)[0], validRot.get(2, 1)[0], validRot.get(2, 2)[0], validT.get(2, 0)[0]);

			points4D = points4D1 = triangulatePoints(goodOld, goodNew, cameraMatrix, P1, P2, false);
			break;
		case OPENCV_TRIANGULATION:
			Calib3d.triangulatePoints(P1, P2, goodOld, goodNew, points4D);
			break;
		}

		for (int i = 0; i < points4D1.cols(); i++) {
			double w = points4D.get(3, i)[0];
			double x = points4D.get(0, i)[0] / w;
			double y = points4D.get(1, i)[0] / w;
			double z = points4D.get(2, i)[0] / w;

			PointDouble point = new PointDouble(x, z);
			// System.out.println(point);
			if (i < currentSize) {
				currentPoints.add(point);
			} else {
				newPoints.add(point);
			}
		}

		// BADPOINTS MOVED UP

		update.setCurrentPoints(currentPoints);
		update.setBadPointsIndex(badPoints);
		update.setNewPoints(newPoints);

		// Assignment of values for next cycle
		// Only gets called when nothing went wrong
		fMatResult.superGoodPoints1.copyTo(checkpointFeatures);
		nearImage.copyTo(checkpointImage);
		frames++;

		if (this.DEBUG_MODE) {
			System.out.println(update.getCurrentPoints().size() + update.getBadPointsIndex().size());
			System.out.println(update.getCurrentPoints().size() + update.getNewPoints().size());

		}

		CURRENT_STEP = this.STEP_VALID_UPDATE;
		// System.out.println(update);
		return FeatureScaler.getFeatureScaler().getScaledFeatureUpdate(update, cameraPosition);
	}

	private void logPoints4D() {
		File dir = new File("trilogs");
		File file = new File(dir + "\\points.csv");
		File fileFirst = new File(dir + "\\firstSet.csv");
		File fileTran = new File(dir + "\\trans.csv");
		File fileScale = new File(dir + "\\pointsScaled.csv");

		FileOutputStream outputStream = null;
		FileOutputStream outputStream2 = null;
		FileOutputStream outputStream3 = null;
		FileOutputStream outputStream4 = null;

		try {

			if (!dir.exists())
				if (!dir.mkdirs())
					throw new IOException();

			if (first) {
				file.delete();
				fileFirst.delete();
				fileTran.delete();
				fileScale.delete();
			}

			if (!file.exists())
				file.createNewFile();
			if (!fileTran.exists())
				fileTran.createNewFile();
			if (!fileScale.exists())
				fileTran.createNewFile();

			if (first) {
				if (!fileFirst.exists())
					fileFirst.createNewFile();
			}

			outputStream = new FileOutputStream(file, true);
			outputStream3 = new FileOutputStream(fileTran, true);
			outputStream4 = new FileOutputStream(fileScale, true);
			if (first) {
				outputStream2 = new FileOutputStream(fileFirst);
			}

			StringBuffer sb = new StringBuffer();
			for (int i = 0; i < points4D1.cols(); i++) {
				for (int j = 0; j < 4; j++)
					sb.append(points4D1.get(j, i)[0] + (j != 3 ? "," : ""));
				sb.append("\n");
			}

			StringBuffer sb2 = new StringBuffer();
			for (int i = 0; i < points4D2.cols(); i++) {
				for (int j = 0; j < 4; j++)
					sb2.append(points4D2.get(j, i)[0] + (j != 3 ? "," : ""));
				sb2.append("\n");
			}

			sb2.append("\n");

			// System.out.println(sb.toString());

			String str = validT.get(0, 0)[0] + "," + validT.get(1, 0)[0] + "," + validT.get(2, 0)[0];
			str += "," + this.xScale + "," + this.zScale;
			outputStream3.write((str + "\n").getBytes());

			sb.append("\n");
			outputStream.write((sb.toString()).getBytes());
			outputStream4.write((sb2.toString()).getBytes());
			if (first) {
				outputStream2.write((sb.toString()).getBytes());
				outputStream2.close();
			}

			outputStream.close();
			outputStream3.close();
			outputStream4.close();

			first = false;
		} catch (Exception e) {

			System.out.println("called");
		} finally {
			if (outputStream != null)
				try {
					outputStream.close();
				} catch (IOException e) {
					e.printStackTrace();
				}
		}
	}

	private List<Point> KeyPointsToPoints(List<KeyPoint> kps) {
		List<Point> ps = new ArrayList<>();

		for (KeyPoint kp : kps)
			ps.add(kp.pt);

		return ps;
	}

	private List<KeyPoint> PointsToKeyPoints(List<Point> ps) {
		List<KeyPoint> kps = new ArrayList<>();

		for (Point p : ps)
			// Note: I assumed that the third parameter is size, but I'm not
			// sure
			kps.add(new KeyPoint((float) p.x, (float) p.y, 1.0f));

		return kps;
	}

	private void GetAlignedPointsFromMatch(List<KeyPoint> imgpts1, List<KeyPoint> imgpts2, List<DMatch> matches,
			List<KeyPoint> pt_set1, List<KeyPoint> pt_set2) {
		for (int i = 0; i < matches.size(); ++i) {
			pt_set1.add(imgpts1.get(matches.get(i).queryIdx));
			pt_set2.add(imgpts2.get(matches.get(i).trainIdx));
		}
	}

	private Mat convertMatOfPoint2fToMat(MatOfPoint2f mpf) {
		Mat mat = new Mat(mpf.rows(), mpf.cols() * mpf.channels(), CvType.CV_32F);

		for (int i = 0; i < mpf.rows(); ++i) {
			for (int j = 0; j < mpf.cols(); ++j) {
				mat.put(i, j * 2, mpf.get(i, j)[0]);
				mat.put(i, j * 2 + 1, mpf.get(i, j)[1]);
			}
		}
		return mat;
	}

	private FMatResult getFundamentalMat(List<KeyPoint> imgpts1, List<KeyPoint> imgpts2, List<Integer> badpointsList,
			double currentSize) {
		Mat status = new Mat();

		List<KeyPoint> imgpts1_tmp;
		List<KeyPoint> imgpts2_tmp;

		// if (matches.size() <= 0) {
		imgpts1_tmp = imgpts1;
		imgpts2_tmp = imgpts2;
		// } else
		// ;// TODO: GetAlignedPointsFromMatch

		Mat F = null;
		List<Point> pts1, pts2;
		pts1 = KeyPointsToPoints(imgpts1_tmp);
		pts2 = KeyPointsToPoints(imgpts2_tmp);

		MatOfPoint2f pts1Mat = new MatOfPoint2f();
		MatOfPoint2f pts2Mat = new MatOfPoint2f();
		pts1Mat.fromList(pts1);
		pts2Mat.fromList(pts2);

		MatOfPoint2f veryGoodpts1 = new MatOfPoint2f();
		MatOfPoint2f veryGoodpts2 = new MatOfPoint2f();

		// Note: There is no minmaxIdx in java opencv
		MinMaxLocResult res = Core.minMaxLoc(convertMatOfPoint2fToMat(pts1Mat));

		// threshold from [Snavely07 4.1]
		F = Calib3d.findFundamentalMat(pts1Mat, pts2Mat, Calib3d.FM_RANSAC, 0.006 * res.maxVal, 0.99, status);

		// Point Filtering
		int badpointsCompensation = 0;
		List<Integer> additionaBadpoints = new ArrayList<>();
		for (int statusIndex = 0; statusIndex < status.size().height; statusIndex++) {
			if (!badpointsList.isEmpty() && badpointsCompensation < badpointsList.size()
					&& statusIndex + badpointsCompensation == badpointsList.get(badpointsCompensation)) {
				// System.out.println((statusIndex + badpointsCompensation) +
				// " existing bad");
				badpointsCompensation++;
				statusIndex--;
				continue;
			}
			int actualStatus = (int) status.get(statusIndex, 0)[0];
			if (actualStatus == 1) {
				// System.out.println((statusIndex + badpointsCompensation) +
				// " very good");
				veryGoodpts1.push_back(pts1Mat.submat(statusIndex, statusIndex + 1, 0, 1));
				veryGoodpts2.push_back(pts2Mat.submat(statusIndex, statusIndex + 1, 0, 1));

			} else if (statusIndex < currentSize) {
				Integer additionalBadpoint = statusIndex + badpointsCompensation;
				additionaBadpoints.add(additionalBadpoint);
				// System.out.println((statusIndex + badpointsCompensation) +
				// " additional bad");
			} else {
				// System.out.println((statusIndex + badpointsCompensation));
			}
		}

		FMatResult result = new FMatResult(F, veryGoodpts1, veryGoodpts2, additionaBadpoints);
		return result;
	}

	private class FMatResult {
		Mat F;
		MatOfPoint2f superGoodPoints1;
		MatOfPoint2f superGoodPoints2;
		List<Integer> additionalBadPoints;

		private FMatResult(Mat F, MatOfPoint2f imgpts1_good, MatOfPoint2f imgpts2_good,
				List<Integer> additionalBadPoints) {
			this.F = F;
			this.superGoodPoints1 = imgpts1_good;
			this.superGoodPoints2 = imgpts2_good;
			this.additionalBadPoints = additionalBadPoints;
		}
	}

	// modifies Rot1, Rot2, T1, T2
	private boolean decomposeEtoRandT(Mat E) {
		W = Mat.zeros(3, 3, CvType.CV_64F);
		W.put(0, 0, 0, -1, 0);
		W.put(1, 0, 1, 0, 0);
		W.put(2, 0, 0, 0, 1);
		u = nullMatF.clone();
		w = nullMatF.clone();
		vt = nullMatF.clone();
		Core.SVDecomp(E, w, u, vt);

		// check if first and second singular values are the same (as they
		// should be)
		double singular_values_ratio = Math.abs(w.get(0, 0)[0]) / Math.abs(w.get(1, 0)[0]);
		if (singular_values_ratio > 1.0)
			singular_values_ratio = 1.0 / singular_values_ratio; // flip ratio
																	// to keep
																	// it [0,1]
		if (singular_values_ratio < 0.7) {
			if (this.DEBUG_MODE)
				System.out.println("Singular values too far apart");
			return false;
		}

		Core.gemm(u, W, 1, nullMatF, 0, tempMat);
		Core.gemm(tempMat, vt, 1, nullMatF, 0, Rot1);
		T1 = u.col(2);

		Core.gemm(u, W.t(), 1, nullMatF, 0, tempMat);
		Core.gemm(tempMat, vt, 1, nullMatF, 0, Rot2);
		T2 = u.col(2).mul(Mat.ones(3, 1, CvType.CV_64F), -1);

		return true;
	}

	private boolean checkCoherentRotation(Mat R) {
		if (Math.abs(Core.determinant(R)) - 1.0 > 1e-07) {
			if (this.DEBUG_MODE)
				System.out.println("resulting rotation is not coherent");
			return false;
		}
		return true;
	}

	private int countNonZero(List<Integer> status) {
		int nonzeros = 0;
		for (int value : status) {
			if (value > 0) {
				nonzeros++;
			}
		}

		return nonzeros;
	}

	/**
	 * @param points4d
	 *            Matrix of 4D homogenous points.
	 * @return The euclidean representation of the input matrix.
	 */
	private List<Point3> homogenizeToList(Mat points4d) {
		List<Point3> homogenized = new ArrayList<>();

		for (int i = 0; i < points4d.cols(); i++) {

			double w = points4d.get(3, i)[0];
			double x = points4d.get(0, i)[0] / w;
			double y = points4d.get(1, i)[0] / w;
			double z = points4d.get(2, i)[0] / w;
			// Point3 wiw = new Point3(x, y, z);
			// Log.d(wiw.toString());
			homogenized.add(new Point3(x, y, z));

		}
		return homogenized;
	}

	private Mat convert1ChannelMatTo4ChannelMat(Mat mat1Chan) {
		Mat mat4Chan = new Mat(mat1Chan.rows() / 4, mat1Chan.cols(), CvType.CV_32FC4);

		double cell[] = new double[4];
		for (int i = 0; i < mat4Chan.rows(); ++i) {
			for (int j = 0; j < mat4Chan.cols(); ++j) {
				for (int k = 0; k < 4; ++k)
					cell[k] = mat1Chan.get(i * 4 + k, j)[0];
				mat4Chan.put(i, j, cell);
			}
		}
		return mat4Chan;
	}

	private Mat convertHPointsToMat4Channel(Mat points4d) {
		Mat mat = new Mat(points4d.cols(), 1, CvType.CV_32FC3);

		double cell[] = new double[3];
		for (int i = 0; i < points4d.cols(); i++) {

			for (int j = 0; j < 3; ++j)
				cell[j] = points4d.get(j, i)[0] / points4d.get(3, i)[0];

			mat.put(i, 0, cell);

		}
		return mat;
	}

	/*
	 * template<typename T> static void perspectiveTransform_( const T* src, T*
	 * dst, const double* m, int len, int scn, int dcn ) { const double eps =
	 * FLT_EPSILON; int i;
	 * 
	 * if( scn == 2 && dcn == 2 ) { for( i = 0; i < len*2; i += 2 ) { T x =
	 * src[i], y = src[i + 1]; double w = x*m[6] + y*m[7] + m[8];
	 * 
	 * if( fabs(w) > eps ) { w = 1./w; dst[i] = (T)((x*m[0] + y*m[1] + m[2])*w);
	 * dst[i+1] = (T)((x*m[3] + y*m[4] + m[5])*w); } else dst[i] = dst[i+1] =
	 * (T)0; } } else if( scn == 3 && dcn == 3 ) { for( i = 0; i < len*3; i += 3
	 * ) { T x = src[i], y = src[i + 1], z = src[i + 2]; double w = x*m[12] +
	 * y*m[13] + z*m[14] + m[15];
	 * 
	 * if( fabs(w) > eps ) { w = 1./w; dst[i] = (T)((x*m[0] + y*m[1] + z*m[2] +
	 * m[3]) * w); dst[i+1] = (T)((x*m[4] + y*m[5] + z*m[6] + m[7]) * w);
	 * dst[i+2] = (T)((x*m[8] + y*m[9] + z*m[10] + m[11]) * w); } else dst[i] =
	 * dst[i+1] = dst[i+2] = (T)0; } } else if( scn == 3 && dcn == 2 ) { for( i
	 * = 0; i < len; i++, src += 3, dst += 2 ) { T x = src[0], y = src[1], z =
	 * src[2]; double w = x*m[8] + y*m[9] + z*m[10] + m[11];
	 * 
	 * if( fabs(w) > eps ) { w = 1./w; dst[0] = (T)((x*m[0] + y*m[1] + z*m[2] +
	 * m[3])*w); dst[1] = (T)((x*m[4] + y*m[5] + z*m[6] + m[7])*w); } else
	 * dst[0] = dst[1] = (T)0; } } else { for( i = 0; i < len; i++, src += scn,
	 * dst += dcn ) { const double* _m = m + dcn*(scn + 1); double w = _m[scn];
	 * int j, k; for( k = 0; k < scn; k++ ) w += _m[k]*src[k]; if( fabs(w) > eps
	 * ) { _m = m; for( j = 0; j < dcn; j++, _m += scn + 1 ) { double s =
	 * _m[scn]; for( k = 0; k < scn; k++ ) s += _m[k]*src[k]; dst[j] = (T)(s*w);
	 * } } else for( j = 0; j < dcn; j++ ) dst[j] = 0; } } }
	 */

	/** Because OpenCv has failed us one more time **/
	private Mat perspectiveTransform(Mat src, Mat mtx) {
		Mat dst = new Mat(src.size(), src.type());
		int depth = src.depth();
		int scn = src.channels();
		int dcn = mtx.cols() - 1;

		// System.out.println("Assert: " + (scn + 1) + " == " + mtx.cols());
		// System.out.println("Assert: " + (dcn));
		// System.out.println("Assert: " + depth + " == " + CvType.CV_32F +
		// " || " + depth + " == " + CvType.CV_64F);

		int mtype = CvType.CV_64F;
		for (int i = 0; i < src.rows(); ++i)
			dst.put(i, 0, singlePointPerspectiveTransform(src, mtx, src.rows(), scn, dcn));

		return dst;
	}

	private double[] singlePointPerspectiveTransform(Mat src, Mat mtx, int len, int scn, int dcn) {
		double cell[] = new double[scn];
		// TODO: implement this

		for (int i = 0; i < len * 3; i += 3) {
			//
			// x = src[i],
			// y = src[i + 1]
			// z = src[i + 2];
			// double w = x*m[12] + y*m[13] + z*m[14] + m[15];
			//
			// if( fabs(w) > eps )
			// {
			// w = 1./w;
			// dst[i] = (T)((x*m[0] + y*m[1] + z*m[2] + m[3]) * w);
			// dst[i+1] = (T)((x*m[4] + y*m[5] + z*m[6] + m[7]) * w);
			// dst[i+2] = (T)((x*m[8] + y*m[9] + z*m[10] + m[11]) * w);
			// }
			// else
			// dst[i] = dst[i+1] = dst[i+2] = (T)0;
		}

		return cell;
	}

	private boolean testTriangulation(final Mat points4d, final Mat P) {
		final int TYPE = points4d.type();

		List<Point3> pcloud_pt3d = homogenizeToList(points4d); // CloudPointsToPoints(pcloud);
		Mat pcloud_pt3d_projected = new Mat(0, 0, CvType.CV_32FC3);// points4d.rows(),
																	// points4d.cols()
																	// - 1,
																	// TYPE);

		Mat P4x4 = Mat.eye(4, 4, TYPE);
		for (int i = 0; i < 12; i++) {
			int row = i / 4;
			int col = i % 4;
			P4x4.put(row, col, P.get(row, col));
		}

		// perspectiveTransform() requires Mat, but source uses a vector.
		Mat points4d32F = convert1ChannelMatTo4ChannelMat(points4d);
		// System.out.println(points4d.size());
		// System.out.println(points4d32F.size());

		// perspectiveTransform(convertHPointsToMat4Channel(points4d), P);

		Mat pcloud_mat = new Mat();

		// Calib3d.convertPointsFromHomogeneous(points4d32F, pcloud_mat);

		// System.out.println();
		// System.out.println(convertHPointsToMat4Channel(points4d).size());
		// System.out.println(convertHPointsToMat4Channel(points4d).channels());
		// System.out.println(convertHPointsToMat4Channel(points4d).type());
		// System.out.println();
		// System.out.println(points4d32F.size());
		// System.out.println(points4d32F.channels());
		// System.out.println(points4d32F.type());
		// System.out.println();
		// System.out.println(points4d.size());
		// System.out.println(points4d.channels());
		// System.out.println(points4d.type());
		// System.out.println();
		// System.out.println(pcloud_mat.size());
		// System.out.println(pcloud_mat.channels());
		// System.out.println(pcloud_mat.type());
		// System.out.println();
		// System.out.println(Converters.vector_Point3d_to_Mat(pcloud_pt3d).size());
		// System.out.println(Converters.vector_Point3d_to_Mat(pcloud_pt3d).channels());
		// System.out.println(Converters.vector_Point3d_to_Mat(pcloud_pt3d).type());
		// System.out.println(pcloud_pt3d_projected.dump());
		// System.out.println();
		// System.out.println(pcloud_pt3d_projected.size());
		// System.out.println(pcloud_pt3d_projected.channels());
		// System.out.println(pcloud_pt3d_projected.type());
		Core.perspectiveTransform(convertHPointsToMat4Channel(points4d), pcloud_pt3d_projected, P4x4);
		// System.out.println();
		// System.out.println(pcloud_pt3d_projected.size());
		// System.out.println(pcloud_pt3d_projected.channels());
		// System.out.println(pcloud_pt3d_projected.type());

		List<Integer> status = new ArrayList<>(pcloud_pt3d.size());
		for (int i = 0; i < pcloud_pt3d.size(); i++) {
			double homogenizedValue = pcloud_pt3d_projected.get(i, 0)[2]; // z
			status.add(Integer.valueOf((homogenizedValue > 0) ? 1 : 0));
		}
		int count = countNonZero(status);

		double percentage = ((double) count / (double) pcloud_pt3d.size());
		String message = count + "/" + pcloud_pt3d.size() + "=" + percentage * 100.0 + "% are in front of the camera.";
		if (percentage < 0.75) {
			return false; // less than 75% of the points are in front of the
							// camera
		}
		return true;
	}

	private Mat triangulatePoints(MatOfPoint2f goodOld, MatOfPoint2f goodNew, Mat K, Mat p1, Mat p2, boolean isFirst) {
		return triangulatePoints(goodOld, goodNew, K, p1, p2, isFirst, true);
	}

	private Mat triangulatePoints(MatOfPoint2f goodOld, MatOfPoint2f goodNew, Mat K, Mat p1, Mat p2, boolean isFirst,
			boolean useLSTriang) {
		Mat Kinv = K.inv(); // Retained k. I needed that ._.

		Mat points4Dtemp = Mat.zeros(goodNew.height(), 0, CvType.CV_64F);

		Mat u1 = Mat.zeros(1, 3, CvType.CV_64F);
		Mat u2 = Mat.zeros(1, 3, CvType.CV_64F);
		Mat um1 = Mat.zeros(3, 1, CvType.CV_64F);
		Mat um2 = Mat.zeros(3, 1, CvType.CV_64F);
		Mat x;

		double reprojectionSum = 0;
		Mat KP1 = new Mat();

		// DANGER dunno what's happening with all the transposes in other
		// gemm()'s and shit
		// GEMM I just stuffed the things there as described in the source
		Core.gemm(K, p2, 1, nullMatF, 0, KP1); // line 188

		for (int i = 0; i < goodOld.height(); i++) {
			// Log.i("Triangulation", "Triangulating, attempt "+ i);
			u1.put(0, 0, goodOld.get(i, 0)[0], goodOld.get(i, 0)[1], 1);

			Core.gemm(Kinv, u1.t(), 1, nullMatF, 0, um1);

			u2.put(0, 0, goodNew.get(i, 0)[0], goodNew.get(i, 0)[1], 1);
			Core.gemm(Kinv, u2.t(), 1, nullMatF, 0, um2);

			if (useLSTriang)
				x = linearLSTriangulation(um1.t(), p1, um2.t(), p2);
			else
				x = iterativeLinearLSTriangulation(um1.t(), p1, um2.t(), p2);

			points4Dtemp.push_back(x.t());

			// BEGIN Reprojection error code
			Mat xPt_img = new Mat();
			Core.gemm(KP1, x, 1, nullMatF, 0, xPt_img); // Reproject
			Point xPt_img_ = new Point(xPt_img.get(0, 0)[0] / xPt_img.get(2, 0)[0], xPt_img.get(1, 0)[0]
					/ xPt_img.get(2, 0)[0]); // (Triangulation.cpp:210)
			Point kp1 = new Point(goodNew.get(i, 0)[0], goodNew.get(i, 0)[1]);
			Point difference = new Point(xPt_img_.x - kp1.x, xPt_img_.y - kp1.y);
			MatOfPoint differenceMat = new MatOfPoint(difference);
			reprojectionSum += Core.norm(differenceMat);
		}

		if (isFirst)
			reprojErr1 = reprojectionSum / goodOld.height();
		else
			reprojErr2 = reprojectionSum / goodOld.height();

		return points4Dtemp.t();
	}

	private Mat linearLSTriangulation(Mat u, Mat p1, Mat u1, Mat p2) {

		double arrMatA[][] = {
				{ u.get(0, 0)[0] * p1.get(2, 0)[0] - p1.get(0, 0)[0],
						u.get(0, 0)[0] * p1.get(2, 1)[0] - p1.get(0, 1)[0],
						u.get(0, 0)[0] * p1.get(2, 2)[0] - p1.get(0, 2)[0] },

				{ u.get(0, 1)[0] * p1.get(2, 0)[0] - p1.get(1, 0)[0],
						u.get(0, 1)[0] * p1.get(2, 1)[0] - p1.get(1, 1)[0],
						u.get(0, 1)[0] * p1.get(2, 2)[0] - p1.get(1, 2)[0] },

				{ u1.get(0, 0)[0] * p2.get(2, 0)[0] - p2.get(0, 0)[0],
						u1.get(0, 0)[0] * p2.get(2, 1)[0] - p2.get(0, 1)[0],
						u1.get(0, 0)[0] * p2.get(2, 2)[0] - p2.get(0, 2)[0] },

				{ u1.get(0, 1)[0] * p2.get(2, 0)[0] - p2.get(1, 0)[0],
						u1.get(0, 1)[0] * p2.get(2, 1)[0] - p2.get(1, 1)[0],
						u1.get(0, 1)[0] * p2.get(2, 2)[0] - p2.get(1, 2)[0] } };

		Mat A = Mat.zeros(4, 3, CvType.CV_64F);
		A.put(0, 0, arrMatA[0]);
		A.put(1, 0, arrMatA[1]);
		A.put(2, 0, arrMatA[2]);
		A.put(3, 0, arrMatA[3]);

		double arrMatB[][] = { { -(u.get(0, 0)[0] * p1.get(2, 3)[0] - p1.get(0, 3)[0]) },
				{ -(u.get(0, 1)[0] * p1.get(2, 3)[0] - p1.get(1, 3)[0]) },
				{ -(u1.get(0, 0)[0] * p2.get(2, 3)[0] - p2.get(0, 3)[0]) },
				{ -(u1.get(0, 1)[0] * p2.get(2, 3)[0] - p2.get(1, 3)[0]) } };

		Mat B = Mat.zeros(4, 1, CvType.CV_64F);
		B.put(0, 0, arrMatB[0]);
		B.put(1, 0, arrMatB[1]);
		B.put(2, 0, arrMatB[2]);
		B.put(3, 0, arrMatB[3]);

		Mat x = Mat.zeros(0, 0, CvType.CV_64F);
		Core.solve(A, B, x, Core.DECOMP_SVD);
		x.push_back(Mat.ones(1, 1, CvType.CV_64F));

		return x;
	}

	double EPSILON = 0.0001;

	private Mat iterativeLinearLSTriangulation(Mat u, Mat p1, Mat u1, Mat p2) {

		double wi = 1, wi1 = 1;

		Mat x = null;
		Mat X = linearLSTriangulation(u, p1, u1, p2);

		Mat p2xm, temp;

		double p2x, p2x1;

		// Log.i("Tri", X.dump());

		for (int i = 0; i < 10; i++) { // Hartley suggests 10 iterations at most

			// recalculate weights
			p2xm = Mat.zeros(0, 0, CvType.CV_64F);
			temp = Mat.zeros(1, 4, CvType.CV_64F);
			temp.put(0, 0, p1.get(2, 0)[0], p1.get(2, 1)[0], p1.get(2, 2)[0], p1.get(2, 3)[0]);
			Core.gemm(temp, X, 1, nullMatF, 0, p2xm);

			// Log.i("Tri", "Temp "+temp.dump() + "\nP1 " + p1.dump() +
			// "\nP2xm "+p2xm.dump());
			//
			p2x = p2xm.get(0, 0)[0];

			p2xm = Mat.zeros(0, 0, CvType.CV_64F);
			temp = Mat.zeros(1, 4, CvType.CV_64F);
			temp.put(0, 0, p2.get(2, 0)[0], p2.get(2, 1)[0], p2.get(2, 2)[0], p2.get(2, 3)[0]);
			Core.gemm(temp, X, 1, nullMatF, 0, p2xm);

			p2x1 = p2xm.get(0, 0)[0];

			// breaking point
			if (Math.abs(wi - p2x) <= EPSILON && Math.abs(wi1 - p2x1) <= EPSILON)
				break;

			wi = p2x;
			wi1 = p2x1;

			// Log.i("Tri", wi + " " + wi1 + " " + p2x + " " + p2x1);

			double arrMatA[][] = {
					{ (u.get(0, 0)[0] * p1.get(2, 0)[0] - p1.get(0, 0)[0]) / wi,
							(u.get(0, 0)[0] * p1.get(2, 1)[0] - p1.get(0, 1)[0]) / wi,
							(u.get(0, 0)[0] * p1.get(2, 2)[0] - p1.get(0, 2)[0]) / wi },

					{ (u.get(0, 1)[0] * p1.get(2, 0)[0] - p1.get(1, 0)[0]) / wi,
							(u.get(0, 1)[0] * p1.get(2, 1)[0] - p1.get(1, 1)[0]) / wi,
							(u.get(0, 1)[0] * p1.get(2, 2)[0] - p1.get(1, 2)[0]) / wi },

					{ (u1.get(0, 0)[0] * p2.get(2, 0)[0] - p2.get(0, 0)[0]) / wi1,
							(u1.get(0, 0)[0] * p2.get(2, 1)[0] - p2.get(0, 1)[0]) / wi1,
							(u1.get(0, 0)[0] * p2.get(2, 2)[0] - p2.get(0, 2)[0]) / wi1 },

					{ (u1.get(0, 1)[0] * p2.get(2, 0)[0] - p2.get(1, 0)[0]) / wi1,
							(u1.get(0, 1)[0] * p2.get(2, 1)[0] - p2.get(1, 1)[0]) / wi1,
							(u1.get(0, 1)[0] * p2.get(2, 2)[0] - p2.get(1, 2)[0]) / wi1 } };

			Mat A = Mat.zeros(4, 3, CvType.CV_64F);
			A.put(0, 0, arrMatA[0]);
			A.put(1, 0, arrMatA[1]);
			A.put(2, 0, arrMatA[2]);
			A.put(3, 0, arrMatA[3]);

			double arrMatB[][] = { { -((u.get(0, 0)[0] * p1.get(2, 3)[0] - p1.get(0, 3)[0])) / wi },
					{ -((u.get(0, 1)[0] * p1.get(2, 3)[0] - p1.get(1, 3)[0])) / wi },
					{ -((u1.get(0, 0)[0] * p2.get(2, 3)[0] - p2.get(0, 3)[0])) / wi1 },
					{ -((u1.get(0, 1)[0] * p2.get(2, 3)[0] - p2.get(1, 3)[0])) / wi1 } };

			Mat B = Mat.zeros(4, 1, CvType.CV_64F);
			B.put(0, 0, arrMatB[0]);
			B.put(1, 0, arrMatB[1]);
			B.put(2, 0, arrMatB[2]);
			B.put(3, 0, arrMatB[3]);

			x = Mat.zeros(0, 0, CvType.CV_64F);
			Core.solve(A, B, x, Core.DECOMP_SVD);
			x.push_back(Mat.ones(1, 1, CvType.CV_64F));

			// Log.i("Tri", x.dump());
			X = x.clone();
		}
		return X;
	}
}
