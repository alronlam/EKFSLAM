package dummies.features;

import java.util.ArrayList;
import java.util.List;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Size;

import commondata.PointDouble;

public class FeatureManager {

	private static final String TAG = "Feature Manager";

	// Optical flow fields
	private int frames = 0;
	private final int FRAME_INTERVAL = 3;
	private boolean framesReady = false;
	private List<Mat> images;

	private Mat checkpointImage;
	private MatOfPoint2f checkpointFeatures;
	private OpticalFlow opticalFlow;

	public static boolean get = false;
	int divr = 3, divc = 2;
	Mat flow;

	// Triangulation fields
	private Size imageSize;
	private Mat cameraMatrix, distCoeffs, Rot2, Rot1, T2, T1;
	private Mat R1, R2, P1, P2, Q;
	private Mat points4D;
	private Mat F, E, W;
	private Mat u, w, vt;
	private Mat nullMatF, tempMat;

	// DANGER Gets assigned in triangulatePoints(). Too lazy to return properly.
	private double reprojectionError = 0;

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

	static boolean firstImg = true;

	public FeatureUpdate getFeatureUpdate(Mat currentImage) {
		if (!framesReady) {
			Mat toAdd = new Mat();
			currentImage.copyTo(toAdd);
			images.add(toAdd);
			if (frames == FRAME_INTERVAL + 3) {
				images.get(0).copyTo(checkpointImage);
				images.remove(0);
				framesReady = true;
			}
			frames++;
			return null;
		}

		Mat nearImage = new Mat();
		images.get(0).copyTo(nearImage);
		Mat farImage = new Mat();
		currentImage.copyTo(farImage);

		// Mat checkpointSnippet = checkpointImage.submat(0,
		// checkpointImage.rows() / divr, 0, checkpointImage.cols() / divc);
		// Mat prevSnippet = nearImage.submat(0, nearImage.rows() / divr, 0,
		// nearImage.cols() / divc);
		// Mat currentSnippet = currentImage.submat(0, currentImage.rows() /
		// divr, 0, currentImage.cols() / divc);

		OpticalFlowResult opflowresult = opticalFlow.getFeatures(checkpointImage, nearImage, farImage,
				checkpointFeatures);
		opflowresult.getNearFeatures().copyTo(checkpointFeatures);

		MatOfPoint2f goodOld = opflowresult.getNearFeatures();
		MatOfPoint2f goodNew = opflowresult.getFarFeatures();

		// if (!get) {
		// // Log.i("FILE WRITE", "written " + points4D.width() + " points");
		// Log.i("FILE WRITE", "written ");
		// XYZConverter.writeRgbOFMatToXYZFile(flow, this.prevRgbMap.submat(0,
		// prevImage.rows() / divr, 0, prevImage.cols() / divc),
		// this.rgbMap.submat(0, currentImage.rows() / divr, 0,
		// currentImage.cols() / divc));// .writeRGBPointsToXYZFile(points4D,
		// rgbMap,div);
		//
		// // XYZConverter.writeGrayOFMatToXYZFile(flow, prevImage.submat(0,
		// prevImage.rows() / divr, 0, prevImage.cols() / divc),
		// // currentImage.submat(0, currentImage.rows() / divr, 0,
		// currentImage.cols() / divc));// .writeRGBPointsToXYZFile(points4D,
		// rgbMap,div);
		// get = true;
		// }

		// Triangulation

		if (!goodOld.empty() && !goodNew.empty()) {
			// SOLVING FOR THE ROTATION AND TRANSLATION MATRICES

			// GETTING THE FUNDAMENTAL MATRIX

			// There is a case that fundamental matrix is not found

			int tries = 0;

			do {
				// MAGIC NUMBERS
				switch (tries) {
				case 0:
					F = Calib3d.findFundamentalMat(goodOld, goodNew, Calib3d.FM_RANSAC, 1, 0.95);
					break;
				case 1:
					F = Calib3d.findFundamentalMat(goodOld, goodNew, Calib3d.FM_RANSAC, 2, 0.90);
					break;
				case 2:
					F = Calib3d.findFundamentalMat(goodOld, goodNew, Calib3d.FM_RANSAC, 3, 0.85);
					break;
				case 3:
					F = Calib3d.findFundamentalMat(goodOld, goodNew, Calib3d.FM_LMEDS, 3, 0.85);
					break;
				default:
					F = Calib3d.findFundamentalMat(goodOld, goodNew);
				}

				tries++;

				// F = Calib3d.findFundamentalMat(goodOld, goodNew);
				// F = Calib3d.findFundamentalMat(goodOld, goodNew,
				// Calib3d.FM_RANSAC, 3, 0.95);
				cameraMatrix = cameraMatrix.clone();

				tempMat = nullMatF.clone();
				E = nullMatF.clone();

				// GETTING THE ESSENTIAL MATRIX

				Core.gemm(cameraMatrix.t(), F, 1, nullMatF, 0, tempMat);
				Core.gemm(tempMat, cameraMatrix, 1, nullMatF, 0, E);

				if (Math.abs(Core.determinant(E)) > 1e-07)
					continue;

				W = Mat.zeros(3, 3, CvType.CV_64F);
				W.put(0, 0, 0, -1, 0);
				W.put(1, 0, 1, 0, 0);
				W.put(2, 0, 0, 0, 1);
				u = nullMatF.clone();
				w = nullMatF.clone();
				vt = nullMatF.clone();
				Core.SVDecomp(E, w, u, vt);

				// check if first and second singular values are the same (as
				// they should be)
				double singular_values_ratio = Math.abs(w.get(0, 0)[0]) / Math.abs(w.get(1, 0)[0]);
				if (singular_values_ratio > 1.0)
					singular_values_ratio = 1.0 / singular_values_ratio; // flip
																			// ratio
																			// to
																			// keep
																			// it
																			// [0,1]
				if (singular_values_ratio < 0.7) {
					continue;
				}

				Core.gemm(u, W, 1, nullMatF, 0, tempMat);
				Core.gemm(tempMat, vt, 1, nullMatF, 0, Rot1);
				T1 = u.col(2);

				Core.gemm(u, W.inv(), 1, nullMatF, 0, tempMat);
				Core.gemm(tempMat, vt, 1, nullMatF, 0, Rot2);
				T2 = u.col(2).mul(Mat.ones(u.col(2).size(), u.col(2).type()), -1);

				if (Core.determinant(R1) + 1.0 < 1e-09) {
					// according to
					// http://en.wikipedia.org/wiki/Essential_matrix#Showing_that_it_is_valid
					E.mul(Mat.ones(E.size(), E.type()), -1);

					Core.SVDecomp(E, w, u, vt);

					// check if first and second singular values are the same
					// (as they should be)
					singular_values_ratio = Math.abs(w.get(0, 0)[0]) / Math.abs(w.get(1, 0)[0]);
					if (singular_values_ratio > 1.0)
						singular_values_ratio = 1.0 / singular_values_ratio; // flip
																				// ratio
																				// to
																				// keep
																				// it
																				// [0,1]
					if (singular_values_ratio < 0.7) {
						continue;
					}

					Core.gemm(u, W, 1, nullMatF, 0, tempMat);
					Core.gemm(tempMat, vt, 1, nullMatF, 0, Rot1);
					T1 = u.col(2);

					Core.gemm(u, W.inv(), 1, nullMatF, 0, tempMat);
					Core.gemm(tempMat, vt, 1, nullMatF, 0, Rot2);
					T2 = u.col(2).mul(Mat.ones(u.col(2).size(), u.col(2).type()), -1);
				}

				if (Math.abs(Core.determinant(R1)) - 1.0 > 1e-07) {
					continue;
				}

				P1.put(0, 0, 1, 0, 0, 0);
				P1.put(1, 0, 0, 1, 0, 0);
				P1.put(2, 0, 0, 0, 1, 0);

				break;

			} while (true);

			while (!get) { // hack so we can use break

				// Combination 1
				P2.put(0, 0, Rot1.get(0, 0)[0], Rot1.get(0, 1)[0], Rot1.get(0, 2)[0], T1.get(0, 0)[0]);
				P2.put(1, 0, Rot1.get(1, 0)[0], Rot1.get(1, 1)[0], Rot1.get(1, 2)[0], T1.get(1, 0)[0]);
				P2.put(2, 0, Rot1.get(2, 0)[0], Rot1.get(2, 1)[0], Rot1.get(2, 2)[0], T1.get(2, 0)[0]);

				points4D = Mat.zeros(0, 4, CvType.CV_64F);
				points4D = triangulatePoints(goodOld, goodNew, cameraMatrix, P1, P2);

				// XYZConverter.writeAllToXYZFile(flow, prevImage, currentImage,
				// points4D, divr,divc, rgbMap, "tryC1.xyz");

				if (reprojectionError <= 100) {
					// if (testTriangulation(points4D, P2) && reprojectionError
					// <= 100) {
					break;
				}

				// Combination 2
				P2.put(0, 0, Rot1.get(0, 0)[0], Rot1.get(0, 1)[0], Rot1.get(0, 2)[0], T2.get(0, 0)[0]);
				P2.put(1, 0, Rot1.get(1, 0)[0], Rot1.get(1, 1)[0], Rot1.get(1, 2)[0], T2.get(1, 0)[0]);
				P2.put(2, 0, Rot1.get(2, 0)[0], Rot1.get(2, 1)[0], Rot1.get(2, 2)[0], T2.get(2, 0)[0]);

				points4D = Mat.zeros(0, 4, CvType.CV_64F);
				points4D = triangulatePoints(goodOld, goodNew, cameraMatrix, P1, P2);

				// XYZConverter.writeAllToXYZFile(flow, prevImage, currentImage,
				// points4D, divr, divc, rgbMap, "tryC2.xyz");

				if (reprojectionError <= 100) {
					// if (testTriangulation(points4D, P2) && reprojectionError
					// <= 100) {
					break;
				}

				// Combination 3
				P2.put(0, 0, Rot2.get(0, 0)[0], Rot2.get(0, 1)[0], Rot2.get(0, 2)[0], T1.get(0, 0)[0]);
				P2.put(1, 0, Rot2.get(1, 0)[0], Rot2.get(1, 1)[0], Rot2.get(1, 2)[0], T1.get(1, 0)[0]);
				P2.put(2, 0, Rot2.get(2, 0)[0], Rot2.get(2, 1)[0], Rot2.get(2, 2)[0], T1.get(2, 0)[0]);

				points4D = Mat.zeros(0, 4, CvType.CV_64F);
				points4D = triangulatePoints(goodOld, goodNew, cameraMatrix, P1, P2);

				// XYZConverter.writeAllToXYZFile(flow, prevImage, currentImage,
				// points4D, divr,divc, rgbMap, "tryC3.xyz");

				if (reprojectionError <= 100) {
					// if (testTriangulation(points4D, P2) && reprojectionError
					// <= 100) {
					break;
				}

				// Combination 4
				P2.put(0, 0, Rot2.get(0, 0)[0], Rot2.get(0, 1)[0], Rot2.get(0, 2)[0], T2.get(0, 0)[0]);
				P2.put(1, 0, Rot2.get(1, 0)[0], Rot2.get(1, 1)[0], Rot2.get(1, 2)[0], T2.get(1, 0)[0]);
				P2.put(2, 0, Rot2.get(2, 0)[0], Rot2.get(2, 1)[0], Rot2.get(2, 2)[0], T2.get(2, 0)[0]);

				points4D = Mat.zeros(0, 4, CvType.CV_64F);
				points4D = triangulatePoints(goodOld, goodNew, cameraMatrix, P1, P2);

				// XYZConverter.writeAllToXYZFile(flow, prevImage, currentImage,
				// points4D, divr,divc, prevRgbMap, "tryC4.xyz");

				if (reprojectionError <= 100) {
					// if (testTriangulation(points4D, P2) && reprojectionError
					// <= 100) {
					break;
				} else {
					break;
				}

				// Log.i("FILE WRITE", "written " + points4D.width() +
				// " points");
				// get = true;
			}

			// check RoT and T

			// Calib3d.triangulatePoints(P1,P2, goodOld, goodNew, points4D);
			// if (!get) {
			// Log.i("FILE WRITE", "written " + points4D.width() + " points");
			//
			// XYZConverter.writeRGBPointsToXYZFile(points4D, rgbMap, div);
			// get = true;
			// }
		}

		FeatureUpdate update = new FeatureUpdate();
		List<PointDouble> currentPoints = new ArrayList<>();
		List<PointDouble> newPoints = new ArrayList<>();

		int currentSize = (int) opflowresult.getCurrentSize();
		for (int i = 0; i < points4D.cols(); i++) {
			double w = points4D.get(3, i)[0];
			double x = points4D.get(0, i)[0] / w;
			double y = points4D.get(1, i)[0] / w;
			double z = points4D.get(2, i)[0] / w;

			PointDouble point = new PointDouble(y, z);

			if (i < currentSize) {
				currentPoints.add(point);
			} else {
				newPoints.add(point);
			}
		}

		update.setCurrentPoints(currentPoints);
		update.setBadPointsIndex(opflowresult.getBadPointsIndex());
		update.setNewPoints(newPoints);

		images.add(farImage);
		nearImage.copyTo(checkpointImage);
		images.remove(0);
		frames++;

		return update;
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

	private boolean testTriangulation(final Mat points4d, final Mat P) {
		final int TYPE = points4d.type();

		List<Point3> pcloud_pt3d = homogenizeToList(points4d); // CloudPointsToPoints(pcloud);
		Mat pcloud_pt3d_projected = new Mat(points4d.rows(), points4d.cols() - 1, TYPE);

		Mat P4x4 = Mat.eye(4, 4, TYPE);
		for (int i = 0; i < 12; i++) {
			int row = i / 4;
			int col = i % 4;
			P4x4.put(row, col, P.get(row, col));
		}

		// perspectiveTransform() requires Mat, but source uses a vector.
		Mat points4d32F = new Mat();
		points4d.convertTo(points4d32F, CvType.CV_32FC3);

		Mat pcloud_mat = new Mat();

		Calib3d.convertPointsFromHomogeneous(points4d32F, pcloud_mat);
		Core.perspectiveTransform(pcloud_mat, pcloud_pt3d_projected, P4x4);

		List<Integer> status = new ArrayList<>(pcloud_pt3d.size());
		for (int i = 0; i < pcloud_pt3d.size(); i++) {
			double homogenizedValue = pcloud_pt3d_projected.get(i, 2)[0] / pcloud_pt3d_projected.get(i, 3)[0]; // z
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

	private Mat triangulatePoints(MatOfPoint2f goodOld, MatOfPoint2f goodNew, Mat K, Mat p1, Mat p2) {
		Mat Kinv = K.inv(); // Retained k. I needed that ._.

		Mat points4D = Mat.zeros(goodNew.height(), 0, CvType.CV_64F);

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

			// x = iterativeLinearLSTriangulation(um1.t(), p1, um2.t(), p2);
			x = linearLSTriangulation(um1.t(), p1, um2.t(), p2);
			points4D.push_back(x.t());

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
		reprojectionError = reprojectionSum / goodOld.height();

		return points4D.t();
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
