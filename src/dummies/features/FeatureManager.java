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
import org.opencv.core.Core.MinMaxLocResult;
import org.opencv.features2d.DMatch;
import org.opencv.features2d.KeyPoint;
import org.opencv.utils.Converters;

import commondata.PointDouble;

public class FeatureManager {

	private static final String TAG = "Feature Manager";
	private static boolean DEBUG_MODE = false;
	
	// Optical flow fields
	private int frames = 0;
	private final int FRAME_INTERVAL = 0;
	private boolean framesReady = false;
	private List<Mat> images;

	private Mat checkpointImage;
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

	public static int STEP_VALID_UPDATE = 0;
	public static int STEP_IMAGE_CAPTURE = 1;
	public static int STEP_OPTICAL_FLOW = 2;
	public static int STEP_ESSENTIAL_MATRIX = 3;
	public static int STEP_TRIANGULATION = 4;

	public static int CURRENT_STEP;

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

	public FeatureUpdate getFeatureUpdate(Mat currentImage) {
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
			return null;
		}
		CURRENT_STEP = this.STEP_OPTICAL_FLOW;

		Mat nearImage = new Mat();
		images.get(0).copyTo(nearImage);
		Mat farImage = new Mat();
		currentImage.copyTo(farImage);
		images.add(farImage);
		images.remove(0);
		

		OpticalFlowResult opflowresult = opticalFlow.getFeatures(checkpointImage, nearImage, farImage, checkpointFeatures);

		MatOfPoint2f goodOld = opflowresult.getNearFeatures();
		MatOfPoint2f goodNew = opflowresult.getFarFeatures();

		points4D1 = new Mat();
		if (!goodOld.empty() && !goodNew.empty()) {
			// SOLVING FOR THE ROTATION AND TRANSLATION MATRICES


			List<KeyPoint> kpGoodOld = new ArrayList<KeyPoint>(), kpGoodNew = new ArrayList<KeyPoint>();

			kpGoodOld = convertMatOfPoint2fToListOfKeyPoints(goodOld);
			kpGoodNew = convertMatOfPoint2fToListOfKeyPoints(goodNew);

			CURRENT_STEP = this.STEP_ESSENTIAL_MATRIX;

			List<DMatch> matches = new ArrayList<>();

			// GETTING THE FUNDAMENTAL MATRIX
			F = getFundamentalMat(kpGoodOld, kpGoodNew, matches);

//			if(matches.size() < 100)
//			// not enough inliers
//			return null;
			
			tempMat = nullMatF.clone();
			E = nullMatF.clone();

			// GETTING THE ESSENTIAL MATRIX
			Core.gemm(cameraMatrix.t(), F, 1, nullMatF, 0, tempMat);
			Core.gemm(tempMat, cameraMatrix, 1, nullMatF, 0, E);

			if (Math.abs(Core.determinant(E)) > 1e-07) {
				if(this.DEBUG_MODE)
					System.out.println( "det(E) != 0 : " + Core.determinant(E));
				P2 = Mat.zeros(3, 4, CvType.CV_64F); // TODO: Double check the type
				return null;
			}

			if (!decomposeEtoRandT(E))
				return null;

			if (Core.determinant(R1) + 1.0 < 1e-09) {
				// according to http://en.wikipedia.org/wiki/Essential_matrix#Showing_that_it_is_valid
				
				if(this.DEBUG_MODE)
					System.out.println("det(R) == -1 ["+Core.determinant(R1)+"]: flip E's sign");
				// TODO: cout <<  << endl;

				E = E.mul(Mat.ones(E.size(), E.type()), -1);

				if (!decomposeEtoRandT(E))
					return null;
			}

			P1.put(0, 0, 1, 0, 0, 0);
			P1.put(1, 0, 0, 1, 0, 0);
			P1.put(2, 0, 0, 0, 1, 0);

			if (!checkCoherentRotation(Rot1)) {
				P2 = Mat.zeros(3, 4, CvType.CV_64F); // TODO: Double check the type
				return null;
			}

			CURRENT_STEP = this.STEP_TRIANGULATION;

			// Combination 1
			P2.put(0, 0, Rot1.get(0, 0)[0], Rot1.get(0, 1)[0], Rot1.get(0, 2)[0], T1.get(0, 0)[0]);
			P2.put(1, 0, Rot1.get(1, 0)[0], Rot1.get(1, 1)[0], Rot1.get(1, 2)[0], T1.get(1, 0)[0]);
			P2.put(2, 0, Rot1.get(2, 0)[0], Rot1.get(2, 1)[0], Rot1.get(2, 2)[0], T1.get(2, 0)[0]);

			points4D1 = triangulatePoints(goodOld, goodNew, cameraMatrix, P1, P2, true);
			points4D2 = triangulatePoints(goodNew, goodOld, cameraMatrix, P2, P1, false);

			if (reprojErr1 > 100 || reprojErr2 > 100) { // TODO: Test Triangulation, !testTriangulation(points4D1, P1) ||

				// Combination 2
				P2.put(0, 0, Rot1.get(0, 0)[0], Rot1.get(0, 1)[0], Rot1.get(0, 2)[0], T2.get(0, 0)[0]);
				P2.put(1, 0, Rot1.get(1, 0)[0], Rot1.get(1, 1)[0], Rot1.get(1, 2)[0], T2.get(1, 0)[0]);
				P2.put(2, 0, Rot1.get(2, 0)[0], Rot1.get(2, 1)[0], Rot1.get(2, 2)[0], T2.get(2, 0)[0]);

				points4D1 = triangulatePoints(goodOld, goodNew, cameraMatrix, P1, P2, true);
				points4D2 = triangulatePoints(goodNew, goodOld, cameraMatrix, P2, P1, false);

				if (reprojErr1 > 100 || reprojErr2 > 100) { // TODO: Test Triangulation
					if (!checkCoherentRotation(Rot2)) {
						P2 = Mat.zeros(3, 4, CvType.CV_64F); // TODO: Double check the type
						return null;
					}
					// Combination 3
					P2.put(0, 0, Rot2.get(0, 0)[0], Rot2.get(0, 1)[0], Rot2.get(0, 2)[0], T1.get(0, 0)[0]);
					P2.put(1, 0, Rot2.get(1, 0)[0], Rot2.get(1, 1)[0], Rot2.get(1, 2)[0], T1.get(1, 0)[0]);
					P2.put(2, 0, Rot2.get(2, 0)[0], Rot2.get(2, 1)[0], Rot2.get(2, 2)[0], T1.get(2, 0)[0]);

					points4D1 = triangulatePoints(goodOld, goodNew, cameraMatrix, P1, P2, true);
					points4D2 = triangulatePoints(goodNew, goodOld, cameraMatrix, P2, P1, false);

					if (reprojErr1 > 100 || reprojErr2 > 100) { // TODO: Test Triangulation

						// Combination 4
						P2.put(0, 0, Rot2.get(0, 0)[0], Rot2.get(0, 1)[0], Rot2.get(0, 2)[0], T2.get(0, 0)[0]);
						P2.put(1, 0, Rot2.get(1, 0)[0], Rot2.get(1, 1)[0], Rot2.get(1, 2)[0], T2.get(1, 0)[0]);
						P2.put(2, 0, Rot2.get(2, 0)[0], Rot2.get(2, 1)[0], Rot2.get(2, 2)[0], T2.get(2, 0)[0]);

						points4D1 = triangulatePoints(goodOld, goodNew, cameraMatrix, P1, P2, true);
						points4D2 = triangulatePoints(goodNew, goodOld, cameraMatrix, P2, P1, false);

						if (reprojErr1 > 100 || reprojErr2 > 100) { // TODO: Test Triangulation
							// Triangulation failed.
							return null;
						}
					}
				}
			}
		}

		// TODO: Check

		FeatureUpdate update = new FeatureUpdate();
		List<PointDouble> currentPoints = new ArrayList<>();
		List<PointDouble> newPoints = new ArrayList<>();
		int currentSize = (int) opflowresult.getCurrentSize();
		for (int i = 0; i < points4D1.cols(); i++) {
			double w = points4D1.get(3, i)[0];
			double x = points4D1.get(0, i)[0] / w;
			double y = points4D1.get(1, i)[0] / w;
			double z = points4D1.get(2, i)[0] / w;

			PointDouble point = new PointDouble(x, z);

			if (i < currentSize) {
				// System.out.println("Current: " + i);
				currentPoints.add(point);
			} else {
				// System.out.println("New: " + i);
				newPoints.add(point);
			}
		}

		update.setCurrentPoints(currentPoints);
		update.setBadPointsIndex(opflowresult.getBadPointsIndex());
		update.setNewPoints(newPoints);

		// Assignment of values for next cycle
		// Only gets caleld when nothing went wrong
		opflowresult.getNearFeatures().copyTo(checkpointFeatures);
		
		nearImage.copyTo(checkpointImage);
		frames++;

		System.out.println(update);
		CURRENT_STEP = this.STEP_VALID_UPDATE;
		
		return update;
	}

	private List<Point> KeyPointsToPoints(List<KeyPoint> kps) { // TODO: not even sure if it should be list of points or mat
		List<Point> ps = new ArrayList<>();

		for (KeyPoint kp : kps)
			ps.add(kp.pt);

		return ps;
	}

	private List<KeyPoint> PointsToKeyPoints(List<Point> ps) { // TODO: not even sure if it should be list of points or mat
		List<KeyPoint> kps = new ArrayList<>();

		for (Point p : ps)
			kps.add(new KeyPoint((float) p.x, (float) p.y, 1.0f)); // TODO: I assumed that the third parameter is size, but I'm not sure

		return kps;
	}

	private void GetAlignedPointsFromMatch(List<KeyPoint> imgpts1, List<KeyPoint> imgpts2, List<DMatch> matches, List<KeyPoint> pt_set1, List<KeyPoint> pt_set2) { // TODO: not even
																																									// // points or
																																									// mat
		// TODO: Implement
		for (int i = 0; i < matches.size(); ++i) {
			pt_set1.add(imgpts1.get(matches.get(i).queryIdx));
			pt_set2.add(imgpts2.get(matches.get(i).trainIdx));
		}
	}

	/** Another converter because Opencv doesn't like us **/
	// private MatOfPoint2f convertListOfPointsToMatOfPoint2f(List<Point> lp){
	// MatOfPoint2f mp = new MatOfPoint2f(lp.size(),1);
	//
	// //mp.p
	//
	// return null;
	// }

	private Mat convertMatOfPoint2fToMat(MatOfPoint2f mpf) {
		Mat mat = new Mat(mpf.rows(), mpf.cols() * mpf.channels(), CvType.CV_32F);

		for (int i = 0; i < mpf.rows(); ++i) {
			for (int j = 0; j < mpf.cols(); ++j) {
				mat.put(i, j*2, mpf.get(i, j)[0]);
				mat.put(i, j*2+1, mpf.get(i, j)[1]);
			}
		}
		return mat;
	}

	private Mat getFundamentalMat(List<KeyPoint> imgpts1, List<KeyPoint> imgpts2, List<DMatch> matches) {
		Mat status = new Mat();
		Mat imgpts1_good = new Mat(), imgpts2_good = new Mat();
		
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
		// TODO: double check types

		// MatOfPoint2f pts1Mat = (MatOfPoint2f) Converters.vector_Point2f_to_Mat(pts1);
		// MatOfPoint2f pts2Mat = (MatOfPoint2f) Converters.vector_Point2f_to_Mat(pts2);
		MatOfPoint2f pts1Mat = new MatOfPoint2f();
		MatOfPoint2f pts2Mat = new MatOfPoint2f();
		pts1Mat.fromList(pts1);
		pts2Mat.fromList(pts2);

		// Note: There is no minmaxIdx in java opencv
		MinMaxLocResult res = Core.minMaxLoc(convertMatOfPoint2fToMat(pts1Mat));

		// threshold from [Snavely07 4.1]
		F = Calib3d.findFundamentalMat(pts1Mat, pts2Mat, Calib3d.FM_RANSAC, 0.006 * res.maxVal, 0.99, status); 

		// TODO: Point Filtering
		// vector<DMatch> new_matches;
		// cout << "F keeping " << countNonZero(status) << " / " << status.size() << endl;
		// for (unsigned int i=0; i<status.size(); i++) {
		// if (status[i])
		// {
		// imgpts1_good.push_back(imgpts1_tmp[i]);
		// imgpts2_good.push_back(imgpts2_tmp[i]);
		//
		// new_matches.push_back(matches[i]);
		// }
		// }
		//
		// cout << matches.size() << " matches before, " << new_matches.size() << " new matches after Fundamental Matrix\n";
		// matches = new_matches; //keep only those points who survived the fundamental matrix
		
		return F;
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

		// check if first and second singular values are the same (as they should be)
		double singular_values_ratio = Math.abs(w.get(0, 0)[0]) / Math.abs(w.get(1, 0)[0]);
		if (singular_values_ratio > 1.0)
			singular_values_ratio = 1.0 / singular_values_ratio; // flip ratio to keep it [0,1]
		if (singular_values_ratio < 0.7) {
			if(this.DEBUG_MODE)
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
			if(this.DEBUG_MODE)
				System.out.println( "resulting rotation is not coherent");
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

	private Mat triangulatePoints(MatOfPoint2f goodOld, MatOfPoint2f goodNew, Mat K, Mat p1, Mat p2, boolean isFirst) {
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

			// x = iterativeLinearLSTriangulation(um1.t(), p1, um2.t(), p2);
			x = linearLSTriangulation(um1.t(), p1, um2.t(), p2);
			points4Dtemp.push_back(x.t());

			// BEGIN Reprojection error code
			Mat xPt_img = new Mat();
			Core.gemm(KP1, x, 1, nullMatF, 0, xPt_img); // Reproject
			Point xPt_img_ = new Point(xPt_img.get(0, 0)[0] / xPt_img.get(2, 0)[0], xPt_img.get(1, 0)[0] / xPt_img.get(2, 0)[0]); // (Triangulation.cpp:210)
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
				{ u.get(0, 0)[0] * p1.get(2, 0)[0] - p1.get(0, 0)[0], u.get(0, 0)[0] * p1.get(2, 1)[0] - p1.get(0, 1)[0], u.get(0, 0)[0] * p1.get(2, 2)[0] - p1.get(0, 2)[0] },

				{ u.get(0, 1)[0] * p1.get(2, 0)[0] - p1.get(1, 0)[0], u.get(0, 1)[0] * p1.get(2, 1)[0] - p1.get(1, 1)[0], u.get(0, 1)[0] * p1.get(2, 2)[0] - p1.get(1, 2)[0] },

				{ u1.get(0, 0)[0] * p2.get(2, 0)[0] - p2.get(0, 0)[0], u1.get(0, 0)[0] * p2.get(2, 1)[0] - p2.get(0, 1)[0], u1.get(0, 0)[0] * p2.get(2, 2)[0] - p2.get(0, 2)[0] },

				{ u1.get(0, 1)[0] * p2.get(2, 0)[0] - p2.get(1, 0)[0], u1.get(0, 1)[0] * p2.get(2, 1)[0] - p2.get(1, 1)[0], u1.get(0, 1)[0] * p2.get(2, 2)[0] - p2.get(1, 2)[0] } };

		Mat A = Mat.zeros(4, 3, CvType.CV_64F);
		A.put(0, 0, arrMatA[0]);
		A.put(1, 0, arrMatA[1]);
		A.put(2, 0, arrMatA[2]);
		A.put(3, 0, arrMatA[3]);

		double arrMatB[][] = { { -(u.get(0, 0)[0] * p1.get(2, 3)[0] - p1.get(0, 3)[0]) }, { -(u.get(0, 1)[0] * p1.get(2, 3)[0] - p1.get(1, 3)[0]) },
				{ -(u1.get(0, 0)[0] * p2.get(2, 3)[0] - p2.get(0, 3)[0]) }, { -(u1.get(0, 1)[0] * p2.get(2, 3)[0] - p2.get(1, 3)[0]) } };

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
					{ (u.get(0, 0)[0] * p1.get(2, 0)[0] - p1.get(0, 0)[0]) / wi, (u.get(0, 0)[0] * p1.get(2, 1)[0] - p1.get(0, 1)[0]) / wi,
							(u.get(0, 0)[0] * p1.get(2, 2)[0] - p1.get(0, 2)[0]) / wi },

					{ (u.get(0, 1)[0] * p1.get(2, 0)[0] - p1.get(1, 0)[0]) / wi, (u.get(0, 1)[0] * p1.get(2, 1)[0] - p1.get(1, 1)[0]) / wi,
							(u.get(0, 1)[0] * p1.get(2, 2)[0] - p1.get(1, 2)[0]) / wi },

					{ (u1.get(0, 0)[0] * p2.get(2, 0)[0] - p2.get(0, 0)[0]) / wi1, (u1.get(0, 0)[0] * p2.get(2, 1)[0] - p2.get(0, 1)[0]) / wi1,
							(u1.get(0, 0)[0] * p2.get(2, 2)[0] - p2.get(0, 2)[0]) / wi1 },

					{ (u1.get(0, 1)[0] * p2.get(2, 0)[0] - p2.get(1, 0)[0]) / wi1, (u1.get(0, 1)[0] * p2.get(2, 1)[0] - p2.get(1, 1)[0]) / wi1,
							(u1.get(0, 1)[0] * p2.get(2, 2)[0] - p2.get(1, 2)[0]) / wi1 } };

			Mat A = Mat.zeros(4, 3, CvType.CV_64F);
			A.put(0, 0, arrMatA[0]);
			A.put(1, 0, arrMatA[1]);
			A.put(2, 0, arrMatA[2]);
			A.put(3, 0, arrMatA[3]);

			double arrMatB[][] = { { -((u.get(0, 0)[0] * p1.get(2, 3)[0] - p1.get(0, 3)[0])) / wi }, { -((u.get(0, 1)[0] * p1.get(2, 3)[0] - p1.get(1, 3)[0])) / wi },
					{ -((u1.get(0, 0)[0] * p2.get(2, 3)[0] - p2.get(0, 3)[0])) / wi1 }, { -((u1.get(0, 1)[0] * p2.get(2, 3)[0] - p2.get(1, 3)[0])) / wi1 } };

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
