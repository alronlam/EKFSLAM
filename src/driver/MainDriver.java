package driver;

import idp.VINSIDPController;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.Mat;

import stepbasedins.INSController;
import stepbasedins.data.SensorEntry;
import util.FileLog;
import vins.DoubleIntegrationController;
import vins.VINSController;

import commondata.Constants;
import commondata.PointDouble;

import desktop.img.ImgLogReader;
import desktop.imu.IMULogReader;
import desktop.imu.IMUReadingsBatch;
import dummies.BreadcrumbDummiesController;
import dummies.ekf.EKFScalingCorrecter;
import dummies.features.FeatureData;
import dummies.features.FeatureManager;
import dummies.features.FeatureScaler;
import dummies.features.FeatureUpdate;

public class MainDriver {

	static {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
	}

	private static String logFolder = "results";
	private static String breadcrumbLogFileName = "breadcrumb.csv";
	private static String breadcrumbWithCimuHeadingLogFileName = "breadcrumbCimuHeading.csv";
	private static String breadcrumbWithCimuHeading15hzLogFileName = "breadcrumbCimuHeading15hz.csv";
	private static String insLogFileName = "ins.csv";
	private static String insCimuHeadingLogFileName = "insCimuHeading.csv";
	private static String vinsLogFileName = "vins.csv";
	private static String vins15hzLogFileName = "vins15hz.csv";
	private static String idpLogFileName = "vinsidp.csv";
	private static String doubleIntegrationLogFileName = "doubleintegration.csv";

	private static StringBuilder finalResultsStringBuilder = new StringBuilder();

	public static void main(String[] args) {
		System.out.println("init 1");
		String dataset = Constants.FOLDER_STRT1_SJ5_S4;
		String targetFolder = "data/" + dataset;
		boolean isDatasetAsync = false;
		if (Constants.ASYNC_DATASETS.contains(dataset)) {
			isDatasetAsync = true;
		}

		finalResultsStringBuilder.append(dataset);

		System.out.println("init 2");
		/* Load IMU Dataset */
		IMULogReader imuLogReader = new IMULogReader(targetFolder + "/imu");
		List<IMUReadingsBatch> imuDataset = imuLogReader.readSensorEntries();

		System.out.println("init 3");
		IMULogReader cimuLogReader = new IMULogReader(targetFolder + "/cimu");
		List<IMUReadingsBatch> cimuDataset = cimuLogReader.readSensorEntries();

		System.out.println("init 4");
		/* Load Images Dataset */
		ImgLogReader imgLogReader = new ImgLogReader(targetFolder + "/img");
		List<Mat> imgDataset = imgLogReader.readImages();

		System.out.println("init 5");
		/* Change IMU Dataset with Camera Heading */
		List<IMUReadingsBatch> imuDatasetWithCimuHeading = changeHeading(imuDataset, cimuDataset);

		runVINSAsync(cimuDataset, imgDataset, vinsLogFileName, false);

		runINS(imuDataset, imgDataset, insLogFileName);
		runINS(imuDatasetWithCimuHeading, imgDataset, insCimuHeadingLogFileName);
		runDoubleIntegration(cimuDataset, imgDataset);
		runVINSAsyncDummy(cimuDataset, imgDataset, vinsLogFileName, false);
		runVINSAsync(cimuDataset, imgDataset, vinsLogFileName, false);
		runVINSAsync(cimuDataset, imgDataset, vins15hzLogFileName, true);
		runBreadcrumbAsync(imuDatasetWithCimuHeading, imgDataset, breadcrumbWithCimuHeadingLogFileName, false);
		runBreadcrumbAsync(imuDatasetWithCimuHeading, imgDataset, breadcrumbWithCimuHeading15hzLogFileName, true);

		System.out.println(finalResultsStringBuilder.toString());
	}

	private static List<IMUReadingsBatch> changeHeading(List<IMUReadingsBatch> originalIMUDataset,
			List<IMUReadingsBatch> cimuDataset) {

		List<IMUReadingsBatch> newIMUDataset = new ArrayList<IMUReadingsBatch>();

		for (int i = 0; i < originalIMUDataset.size(); i++) {

			IMUReadingsBatch imuBatchCopy = originalIMUDataset.get(i).getCopy();

			ArrayList<SensorEntry> newEntries = imuBatchCopy.getEntries();
			ArrayList<SensorEntry> cimuEntries = cimuDataset.get(i).getEntries();

			for (int j = 0; j < cimuEntries.size(); j++) {
				newEntries.get(j).setOrient_x(cimuEntries.get(j).getOrient_x());
			}

			newIMUDataset.add(imuBatchCopy);
		}

		return newIMUDataset;

	}

	private static void runDoubleIntegration(List<IMUReadingsBatch> imuDataset, List<Mat> imgDataset) {
		DoubleIntegrationController doubleIntegration = new DoubleIntegrationController();

		/* Initialize the logs for all three techniques */
		FileLog doubleIntegrationLog = new FileLog(logFolder + "/" + doubleIntegrationLogFileName);
		doubleIntegrationLog.append(doubleIntegration.getDeviceCoords() + "\n");

		/* Their sizes may not match due to logging problems */
		int datasetSize = Math.min(imuDataset.size(), imgDataset.size());
		System.out.println("DATASET SIZE: IMU = " + imuDataset.size() + " and  IMG = " + imgDataset.size());

		for (int i = 0; i < datasetSize; i++) {

			System.out.println("\n\nTime Step " + (i + 1));

			/* IMU Predict */
			IMUReadingsBatch currIMUBatch = imuDataset.get(i);
			doubleIntegration.predict(currIMUBatch);
			// System.out.println("Finished predicting.");

			doubleIntegrationLog.append(doubleIntegration.getDeviceCoords() + "\n");
		}
		finalResultsStringBuilder.append("\r\n" + doubleIntegrationLogFileName + "\r\n");
		finalResultsStringBuilder.append("Total distance traveled " + doubleIntegration.getTotalDistanceTraveled()
				+ "\r\n");
		finalResultsStringBuilder.append("Total Displacement = "
				+ doubleIntegration.getDeviceCoords().computeDistanceTo(new PointDouble(0, 0)) + "\r\n");

		/* Log - Write to File */
		doubleIntegrationLog.writeToFile();

	}

	private static void runAltogether(List<IMUReadingsBatch> imuDataset, List<Mat> imgDataset) {
		/* Initialize the three controllers */
		BreadcrumbDummiesController breadcrumb = new BreadcrumbDummiesController();
		INSController ins = new INSController();
		VINSController vins = new VINSController();
		VINSIDPController vinsIDP = new VINSIDPController();

		/* Initialize the logs for all three techniques */
		FileLog breadcrumbLog = new FileLog(logFolder + "/" + breadcrumbLogFileName);
		FileLog insLog = new FileLog(logFolder + "/" + insLogFileName);
		FileLog vinsLog = new FileLog(logFolder + "/" + vinsLogFileName);
		FileLog vinsIDPLog = new FileLog(logFolder + "/" + idpLogFileName);

		breadcrumbLog.append(breadcrumb.getDeviceCoords() + "\n");
		insLog.append(ins.getDeviceCoords() + "\n");
		vinsLog.append(vins.getDeviceCoords() + "\n");
		vinsIDPLog.append(vinsIDP.getDeviceCoords() + "\n");

		FeatureManager featureManager = new FeatureManager();
		idp.features.FeatureManager featureManagerIDP = new idp.features.FeatureManager();

		/* Their sizes may not match due to logging problems */
		int datasetSize = Math.min(imuDataset.size(), imgDataset.size());
		System.out.println("DATASET SIZE: " + imuDataset.size() + " and " + imgDataset.size());

		double prevX = 0;
		double prevY = 0;
		for (int i = 0; i < datasetSize; i++) {

			System.out.println("\n\nTime Step " + (i + 1));

			// Get them fancy translations
			// is this even correct
			double transX = breadcrumb.getDeviceCoords().getX() - prevX;
			double transY = breadcrumb.getDeviceCoords().getY() - prevY;

			prevX = breadcrumb.getDeviceCoords().getX();
			prevY = breadcrumb.getDeviceCoords().getY();

			/* IMU Predict */
			IMUReadingsBatch currIMUBatch = imuDataset.get(i);
			breadcrumb.predict(currIMUBatch);
			ins.predict(currIMUBatch);
			// vins.predict(currIMUBatch);
			// vinsIDP.predict(currIMUBatch);
			System.out.println("Finished predicting.");
			/* Image Update */
			FeatureUpdate featureUpdate = featureManager.getFeatureUpdate(imgDataset.get(i), transX, transY,
					vins.getDeviceCoords());
			breadcrumb.update(featureUpdate);
			// vins.update(featureUpdate);

			// idp.features.FeatureUpdate idpFeatureUpdate =
			// featureManagerIDP.getFeatureUpdate(imgDataset.get(i));
			// vinsIDP.update(idpFeatureUpdate);

			System.out.println("Finished updating.");
			/* Update the logs per controller */
			breadcrumbLog.append(breadcrumb.getDeviceCoords() + "\n");
			insLog.append(ins.getDeviceCoords() + "\n");
			vinsLog.append(vins.getDeviceCoords() + "\n");
			vinsIDPLog.append(vinsIDP.getDeviceCoords() + "\n");
		}
		System.out.println(ins.totalStepsDetected);
		breadcrumbLog.writeToFile();
		insLog.writeToFile();
		vinsLog.writeToFile();
		vinsIDPLog.writeToFile();
	}

	private static void runIDP(List<IMUReadingsBatch> imuDataset, List<Mat> imgDataset) {
		/* Initialize the contrller and manager */
		VINSIDPController vinsIDP = new VINSIDPController();
		idp.features.FeatureManager featureManagerIDP = new idp.features.FeatureManager();

		/* Initialize the logs for all three techniques */
		FileLog vinsIDPLog = new FileLog(logFolder + "/" + idpLogFileName);
		vinsIDPLog.append(vinsIDP.getDeviceCoords() + "\n");

		/* Their sizes may not match due to logging problems */
		int datasetSize = Math.min(imuDataset.size(), imgDataset.size());
		System.out.println("DATASET SIZE: IMU = " + imuDataset.size() + " and  IMG = " + imgDataset.size());

		for (int i = 0; i < datasetSize; i++) {

			System.out.println("\n\nTime Step " + (i + 1));

			/* IMU Predict */
			IMUReadingsBatch currIMUBatch = imuDataset.get(i);
			vinsIDP.predict(currIMUBatch);
			// System.out.println("Finished predicting.");

			/* Image Update */
			idp.features.FeatureUpdate idpFeatureUpdate = featureManagerIDP.getFeatureUpdate(imgDataset.get(i));
			vinsIDP.update(idpFeatureUpdate);

			// System.out.println("Finished updating.");

			/* Update the logs per controller */
			// vinsIDPLog.append((vinsIDP.getDeviceCoords().getX() / 100000) +
			// ","
			// + (vinsIDP.getDeviceCoords().getY() / 100000) + "\n");
			vinsIDPLog.append(vinsIDP.getDeviceCoords() + "\n");
		}

		finalResultsStringBuilder.append("Total Distance Traveled = " + vinsIDP.getTotalDistanceTraveled() + "\r\n");
		finalResultsStringBuilder.append("Total Displacement = "
				+ vinsIDP.getDeviceCoords().computeDistanceTo(new PointDouble(0, 0)) + "\r\n");

		/* Log - Write to File */
		vinsIDPLog.writeToFile();
	}

	private static void runBreadcrumbDummies(List<IMUReadingsBatch> imuDataset, List<Mat> imgDataset, String logFileName) {
		/* Initialize the controller and manager */
		BreadcrumbDummiesController breadcrumb = new BreadcrumbDummiesController();
		FeatureManager featureManager = new FeatureManager();

		/* Initialize the logs */
		FileLog breadcrumbLog = new FileLog(logFolder + "/" + logFileName);
		breadcrumbLog.append(breadcrumb.getDeviceCoords() + "\n");

		/* Their sizes may not match due to logging problems */
		int datasetSize = Math.min(imuDataset.size(), imgDataset.size());
		System.out.println("DATASET SIZE: IMU = " + imuDataset.size() + " and  IMG = " + imgDataset.size());

		double prevX = 0;
		double prevY = 0;
		PointDouble prevPoint = new PointDouble(Double.MAX_VALUE, Double.MAX_VALUE);
		for (int i = 0; i < datasetSize; i++) {

			System.out.println("Time Step " + (i + 1));

			// Get them fancy translations
			// is this even correct
			double transX = breadcrumb.getDeviceCoords().getX() - prevX;
			double transY = breadcrumb.getDeviceCoords().getY() - prevY;

			prevX = breadcrumb.getDeviceCoords().getX();
			prevY = breadcrumb.getDeviceCoords().getY();

			/* IMU Predict */
			IMUReadingsBatch currIMUBatch = imuDataset.get(i);
			breadcrumb.predict(currIMUBatch);

			PointDouble predictResult = breadcrumb.getDeviceCoords();
			// System.out.println("Finished predicting.");

			/* Image Update */
			if (prevPoint.getX() != predictResult.getX() || prevPoint.getY() != predictResult.getY()) {
				FeatureUpdate featureUpdate = featureManager.getFeatureUpdate(imgDataset.get(i), transX, transY,
						breadcrumb.getDeviceCoords());
				breadcrumb.update(featureUpdate);
				prevPoint = predictResult;
			}
			// System.out.println("Finished updating.");

			System.out.println(breadcrumb.getDeviceCoords() + "\n");

			PointDouble deviceCoords = breadcrumb.getDeviceCoords();

			EKFScalingCorrecter.getEKFScalingResultCorrecter().updateCoords(deviceCoords, predictResult);

			/* Update the logs */
			// breadcrumbLog.append(breadcrumb.getDeviceCoords() + "\n");
		}

		breadcrumbLog.append(EKFScalingCorrecter.getEKFScalingResultCorrecter().getCorrectedPositionsAsString());

		finalResultsStringBuilder.append("Total steps detected " + breadcrumb.totalStepsDetected + "\r\n");
		// finalResultsStringBuilder.append("Total distance traveled " +
		// breadcrumb.getTotalDistanceTraveled() + "\r\n");
		// finalResultsStringBuilder.append("Total Displacement = " +
		// breadcrumb.getDeviceCoords().computeDistanceTo(new PointDouble(0, 0))
		// + "\r\n");

		finalResultsStringBuilder.append("Total distance traveled "
				+ EKFScalingCorrecter.getEKFScalingResultCorrecter().getTotalDistanceTraveled() + "\r\n");
		finalResultsStringBuilder.append("Total Displacement = "
				+ EKFScalingCorrecter.getEKFScalingResultCorrecter().getFinalPosition()
						.computeDistanceTo(new PointDouble(0, 0)) + "\r\n");

		/* Log - Write to File */
		breadcrumbLog.writeToFile();
	}

	/* Based on runBreadcrumbDummies */
	private static void runBreadcrumbAsync(List<IMUReadingsBatch> imuDataset, List<Mat> imgDataset, String logFileName,
			boolean isAsync) {
		resetFeatureRelatedStaticVars();
		System.out.println("init 1");
		/* Initialize the controller and manager */
		BreadcrumbDummiesController breadcrumb = new BreadcrumbDummiesController();
		FeatureManager featureManager = new FeatureManager();

		/* Initialize the logs */
		FileLog breadcrumbLog = new FileLog(logFolder + "/" + logFileName);
		breadcrumbLog.append(breadcrumb.getDeviceCoords() + "\n");

		System.out.println("DATASET SIZE: IMU = " + imuDataset.size() + " and  IMG = " + imgDataset.size());

		double prevX = 0;
		double prevY = 0;
		PointDouble prevPoint = new PointDouble(Double.MAX_VALUE, Double.MAX_VALUE);

		System.out.println("init 2");
		int imuIndex = 0;
		int imgIndex = 0;
		int elapsedTime = Constants.MS_IMU_DURATION;
		int timeStep = 0;
		while (true) {
			if (imuIndex >= imuDataset.size() || imgIndex >= imgDataset.size()) {
				break;
			}

			StringBuilder sb = new StringBuilder();
			if (elapsedTime >= Constants.MS_IMU_DURATION) {
				System.out.println("\nTime Step " + (timeStep + 1));
				sb.append("Feature Update. ");
				sb.append("img: " + imgIndex + " ");
				sb.append("imu: " + imuIndex + " ");

				// Get them fancy translations
				// is this even correct
				double transX = breadcrumb.getDeviceCoords().getX() - prevX;
				double transY = breadcrumb.getDeviceCoords().getY() - prevY;

				prevX = breadcrumb.getDeviceCoords().getX();
				prevY = breadcrumb.getDeviceCoords().getY();

				/* IMU Predict */
				IMUReadingsBatch currIMUBatch = imuDataset.get(imuIndex);
				breadcrumb.predict(currIMUBatch);

				PointDouble predictResult = breadcrumb.getDeviceCoords();
				// System.out.println("Finished predicting.");

				/* Image Update */
				if (prevPoint.getX() != predictResult.getX() || prevPoint.getY() != predictResult.getY()) {
					FeatureUpdate featureUpdate = featureManager.getAsyncFeatureUpdate(imgDataset.get(imgIndex),
							transX, transY, breadcrumb.getDeviceCoords());

					breadcrumb.update(featureUpdate);
					prevPoint = predictResult;
				}
				// System.out.println("Finished updating.");

				System.out.println(breadcrumb.getDeviceCoords() + "");

				PointDouble deviceCoords = breadcrumb.getDeviceCoords();

				EKFScalingCorrecter.getEKFScalingResultCorrecter().updateCoords(deviceCoords, predictResult);

				imuIndex++;
				elapsedTime = elapsedTime % Constants.MS_IMU_DURATION;
				sb.append(elapsedTime + "ms");
				/* Update the logs */
				// breadcrumbLog.append(breadcrumb.getDeviceCoords() + "\n");
				System.out.println(sb.toString());

			} else {
				// sb.append("Image Flow. ");
				// sb.append("img: " + imgIndex + " ");
				// sb.append(elapsedTime + "ms");
				// System.out.println(sb.toString());
				if (isAsync)
					featureManager.flowImage(imgDataset.get(imgIndex));
			}

			imgIndex++;
			if (isAsync)
				elapsedTime += Constants.MS_IMG_DURATION;
			else
				elapsedTime += Constants.MS_IMU_DURATION;
			timeStep++;
		}

		breadcrumbLog.append(EKFScalingCorrecter.getEKFScalingResultCorrecter().getCorrectedPositionsAsString());

		finalResultsStringBuilder.append("\r\n" + logFileName + "\r\n");
		finalResultsStringBuilder.append("Total steps detected " + breadcrumb.totalStepsDetected + "\r\n");
		// finalResultsStringBuilder.append("Total distance traveled " +
		// breadcrumb.getTotalDistanceTraveled() + "\r\n");
		// finalResultsStringBuilder.append("Total Displacement = " +
		// breadcrumb.getDeviceCoords().computeDistanceTo(new PointDouble(0, 0))
		// + "\r\n");

		finalResultsStringBuilder.append("Total distance traveled "
				+ EKFScalingCorrecter.getEKFScalingResultCorrecter().getTotalDistanceTraveled() + "\r\n");
		finalResultsStringBuilder.append("Total Displacement = "
				+ EKFScalingCorrecter.getEKFScalingResultCorrecter().getFinalPosition()
						.computeDistanceTo(new PointDouble(0, 0)) + "\r\n");

		/* Log - Write to File */
		breadcrumbLog.writeToFile();
	}

	private static void resetFeatureRelatedStaticVars() {
		FeatureScaler.resetInstance();
		EKFScalingCorrecter.resetInstance();
		FeatureData.resetCameraPositions();
	}

	private static void runVINSAsync(List<IMUReadingsBatch> imuDataset, List<Mat> imgDataset, String logFileName,
			boolean isAsync) {
		resetFeatureRelatedStaticVars();
		System.out.println("init 1");
		/* Initialize the controller and manager */
		VINSController vins = new VINSController();
		FeatureManager featureManager = new FeatureManager();

		/* Initialize the logs */
		FileLog vinsLog = new FileLog(logFolder + "/" + logFileName);
		vinsLog.append(vins.getDeviceCoords() + "\n");

		System.out.println("DATASET SIZE: IMU = " + imuDataset.size() + " and  IMG = " + imgDataset.size());

		int state[] = new int[6];
		int valid[] = new int[4];
		double prevX = 0;
		double prevY = 0;
		PointDouble prevPoint = new PointDouble(Double.MAX_VALUE, Double.MAX_VALUE);

		System.out.println("init 2");
		int imuIndex = 0;
		int imgIndex = 0;
		int elapsedTime = Constants.MS_IMU_DURATION;
		int timeStep = 0;
		while (true) {
			if (imuIndex >= imuDataset.size() || imgIndex >= imgDataset.size()) {
				break;
			}

			StringBuilder sb = new StringBuilder();
			if (elapsedTime >= Constants.MS_IMU_DURATION) {
				System.out.println("\nTime Step " + (timeStep + 1));
				sb.append("Feature Update. ");
				sb.append("img: " + imgIndex + " ");
				sb.append("imu: " + imuIndex + " ");

				// Get them fancy translations
				// is this even correct
				double transX = vins.getDeviceCoords().getX() - prevX;
				double transY = vins.getDeviceCoords().getY() - prevY;

				prevX = vins.getDeviceCoords().getX();
				prevY = vins.getDeviceCoords().getY();

				/* IMU Predict */
				IMUReadingsBatch currIMUBatch = imuDataset.get(imuIndex);
				vins.predict(currIMUBatch);
				PointDouble predictResult = vins.getDeviceCoords();

				/* Image Update */
				if (prevPoint.getX() != predictResult.getX() || prevPoint.getY() != predictResult.getY()) {
					FeatureUpdate featureUpdate = featureManager.getAsyncFeatureUpdate(imgDataset.get(imgIndex),
							transX, transY, vins.getDeviceCoords());
					valid[(FeatureManager.VALID_ROTATION == FeatureManager.ROT_1 ? 0 : 2)
							+ (FeatureManager.VALID_TRANSLATION == FeatureManager.TRAN_1 ? 0 : 1)]++;
					state[FeatureManager.CURRENT_STEP]++;
					state[5]++;

					vins.update(featureUpdate);
				}
				// System.out.println("Finished updating.");

				// System.out.println("Finished updating.");

				PointDouble deviceCoords = vins.getDeviceCoords();

				EKFScalingCorrecter.getEKFScalingResultCorrecter().updateCoords(deviceCoords, predictResult);

				imuIndex++;
				elapsedTime = elapsedTime % Constants.MS_IMU_DURATION;
				sb.append(elapsedTime + "ms");
				/* Update the logs */
				// breadcrumbLog.append(breadcrumb.getDeviceCoords() + "\n");
				System.out.println(sb.toString());

			} else {
				// sb.append("Image Flow. ");
				// sb.append("img: " + imgIndex + " ");
				// sb.append(elapsedTime + "ms");
				// System.out.println(sb.toString());
				if (isAsync)
					featureManager.flowImage(imgDataset.get(imgIndex));
			}

			imgIndex++;
			if (isAsync)
				elapsedTime += Constants.MS_IMG_DURATION;
			else
				elapsedTime += Constants.MS_IMU_DURATION;
			timeStep++;
		}

		vinsLog.append(EKFScalingCorrecter.getEKFScalingResultCorrecter().getCorrectedPositionsAsString());

		System.out.println("Rot 1 & Tran 1: " + valid[0]);
		System.out.println("Rot 1 & Tran 2: " + valid[1]);
		System.out.println("Rot 2 & Tran 1: " + valid[2]);
		System.out.println("Rot 1 & Tran 2: " + valid[3]);
		System.out.println("Success/Processed: " + state[0] + "/" + (state[5] - state[1]));
		System.out.println("Initial Delay: " + state[1]);
		System.out.println("Failed due to optical flow: " + state[2]);
		System.out.println("Failed due to essential matrix: " + state[3]);
		System.out.println("Failed due to triangulation: " + state[4]);
		System.out.println("Success/Processed: " + state[0] + "/" + (state[5] - state[1]));
		System.out.println("Failed/Processed: " + (state[2] + state[3] + state[4]) + "/" + (state[5] - state[1]));
		System.out.printf("Success Rate: %.3f%%\n", state[0] * 100.0 / (state[5] - state[1]));

		finalResultsStringBuilder.append("\r\n" + logFileName + "\r\n");
		finalResultsStringBuilder.append("Total distance traveled "
				+ EKFScalingCorrecter.getEKFScalingResultCorrecter().getTotalDistanceTraveled() + "\r\n");
		finalResultsStringBuilder.append("Total Displacement = "
				+ EKFScalingCorrecter.getEKFScalingResultCorrecter().getFinalPosition()
						.computeDistanceTo(new PointDouble(0, 0)) + "\r\n");

		// finalResultsStringBuilder.append("Total distance traveled " +
		// vins.getTotalDistanceTraveled() + "\r\n");
		// finalResultsStringBuilder.append("Total Displacement = " +
		// vins.getDeviceCoords().computeDistanceTo(new PointDouble(0, 0)) +
		// "\r\n");

		/* Log - Write to File */
		vinsLog.writeToFile();
	}

	// this is just to avoid the wrong results of the first run
	private static void runVINSAsyncDummy(List<IMUReadingsBatch> imuDataset, List<Mat> imgDataset, String logFileName,
			boolean isAsync) {
		resetFeatureRelatedStaticVars();
		System.out.println("init 1");
		/* Initialize the controller and manager */
		VINSController vins = new VINSController();
		FeatureManager featureManager = new FeatureManager();

		/* Initialize the logs */
		FileLog vinsLog = new FileLog(logFolder + "/" + logFileName);
		vinsLog.append(vins.getDeviceCoords() + "\n");

		System.out.println("DATASET SIZE: IMU = " + imuDataset.size() + " and  IMG = " + imgDataset.size());

		int state[] = new int[6];
		int valid[] = new int[4];
		double prevX = 0;
		double prevY = 0;
		PointDouble prevPoint = new PointDouble(Double.MAX_VALUE, Double.MAX_VALUE);

		System.out.println("init 2");
		int imuIndex = 0;
		int imgIndex = 0;
		int elapsedTime = Constants.MS_IMU_DURATION;
		int timeStep = 0;

		StringBuilder sb = new StringBuilder();
		if (elapsedTime >= Constants.MS_IMU_DURATION) {
			System.out.println("\nTime Step " + (timeStep + 1));
			sb.append("Feature Update. ");
			sb.append("img: " + imgIndex + " ");
			sb.append("imu: " + imuIndex + " ");

			// Get them fancy translations
			// is this even correct
			double transX = vins.getDeviceCoords().getX() - prevX;
			double transY = vins.getDeviceCoords().getY() - prevY;

			prevX = vins.getDeviceCoords().getX();
			prevY = vins.getDeviceCoords().getY();

			/* IMU Predict */
			IMUReadingsBatch currIMUBatch = imuDataset.get(imuIndex);
			vins.predict(currIMUBatch);
			PointDouble predictResult = vins.getDeviceCoords();

			/* Image Update */
			if (prevPoint.getX() != predictResult.getX() || prevPoint.getY() != predictResult.getY()) {
				FeatureUpdate featureUpdate = featureManager.getAsyncFeatureUpdate(imgDataset.get(imgIndex), transX,
						transY, vins.getDeviceCoords());
				valid[(FeatureManager.VALID_ROTATION == FeatureManager.ROT_1 ? 0 : 2)
						+ (FeatureManager.VALID_TRANSLATION == FeatureManager.TRAN_1 ? 0 : 1)]++;
				state[FeatureManager.CURRENT_STEP]++;
				state[5]++;

				vins.update(featureUpdate);
			}

			PointDouble deviceCoords = vins.getDeviceCoords();

			EKFScalingCorrecter.getEKFScalingResultCorrecter().updateCoords(deviceCoords, predictResult);

			imuIndex++;
			elapsedTime = elapsedTime % Constants.MS_IMU_DURATION;
			sb.append(elapsedTime + "ms");
			/* Update the logs */
			// breadcrumbLog.append(breadcrumb.getDeviceCoords() + "\n");
			System.out.println(sb.toString());

		} else {

			if (isAsync)
				featureManager.flowImage(imgDataset.get(imgIndex));

			imgIndex++;
			if (isAsync)
				elapsedTime += Constants.MS_IMG_DURATION;
			else
				elapsedTime += Constants.MS_IMU_DURATION;
			timeStep++;
		}

	}

	private static void runVINS(List<IMUReadingsBatch> imuDataset, List<Mat> imgDataset) {
		/* Initialize the controller and manager */
		VINSController vins = new VINSController();
		FeatureManager featureManager = new FeatureManager();

		/* Initialize the logs */
		FileLog vinsLog = new FileLog(logFolder + "/" + vinsLogFileName);
		vinsLog.append(vins.getDeviceCoords() + "\n");

		/* Their sizes may not match due to logging problems */
		int datasetSize = Math.min(imuDataset.size(), imgDataset.size());
		System.out.println("DATASET SIZE: IMU = " + imuDataset.size() + " and  IMG = " + imgDataset.size());

		int state[] = new int[6];
		int valid[] = new int[4];
		double prevX = 0;
		double prevY = 0;
		for (int i = 0; i < datasetSize; i++) {

			System.out.println("\nTime Step " + (i + 1));

			// Get them fancy translations
			// is this even correct
			double transX = vins.getDeviceCoords().getX() - prevX;
			double transY = vins.getDeviceCoords().getY() - prevY;

			prevX = vins.getDeviceCoords().getX();
			prevY = vins.getDeviceCoords().getY();

			/* IMU Predict */
			IMUReadingsBatch currIMUBatch = imuDataset.get(i);
			// System.out.println(vins.getDeviceCoords());
			vins.predict(currIMUBatch);

			PointDouble predictResult = vins.getDeviceCoords();
			// System.out.println(vins.getDeviceCoords());

			/* Image Update */
			FeatureUpdate featureUpdate = featureManager.getFeatureUpdate(imgDataset.get(i), transX, transY,
					vins.getDeviceCoords());
			valid[(FeatureManager.VALID_ROTATION == FeatureManager.ROT_1 ? 0 : 2)
					+ (FeatureManager.VALID_TRANSLATION == FeatureManager.TRAN_1 ? 0 : 1)]++;
			state[FeatureManager.CURRENT_STEP]++;
			state[5]++;

			vins.update(featureUpdate);
			// System.out.println("Finished updating.");

			PointDouble deviceCoords = vins.getDeviceCoords();

			EKFScalingCorrecter.getEKFScalingResultCorrecter().updateCoords(deviceCoords, predictResult);

			/* Update the logs */
			// vinsLog.append(vins.getDeviceCoords() + "\n");
		}

		vinsLog.append(EKFScalingCorrecter.getEKFScalingResultCorrecter().getCorrectedPositionsAsString());

		System.out.println("Rot 1 & Tran 1: " + valid[0]);
		System.out.println("Rot 1 & Tran 2: " + valid[1]);
		System.out.println("Rot 2 & Tran 1: " + valid[2]);
		System.out.println("Rot 1 & Tran 2: " + valid[3]);
		System.out.println("Success/Processed: " + state[0] + "/" + (state[5] - state[1]));
		System.out.println("Dataset Size: " + datasetSize);
		System.out.println("Initial Delay: " + state[1]);
		System.out.println("Failed due to optical flow: " + state[2]);
		System.out.println("Failed due to essential matrix: " + state[3]);
		System.out.println("Failed due to triangulation: " + state[4]);
		System.out.println("Success/Processed: " + state[0] + "/" + (state[5] - state[1]));
		System.out.println("Failed/Processed: " + (state[2] + state[3] + state[4]) + "/" + (state[5] - state[1]));
		System.out.printf("Success Rate: %.3f%%\n", state[0] * 100.0 / (state[5] - state[1]));

		finalResultsStringBuilder.append("Total distance traveled "
				+ EKFScalingCorrecter.getEKFScalingResultCorrecter().getTotalDistanceTraveled() + "\r\n");
		finalResultsStringBuilder.append("Total Displacement = "
				+ EKFScalingCorrecter.getEKFScalingResultCorrecter().getFinalPosition()
						.computeDistanceTo(new PointDouble(0, 0)) + "\r\n");

		// finalResultsStringBuilder.append("Total distance traveled " +
		// vins.getTotalDistanceTraveled() + "\r\n");
		// finalResultsStringBuilder.append("Total Displacement = " +
		// vins.getDeviceCoords().computeDistanceTo(new PointDouble(0, 0)) +
		// "\r\n");

		/* Log - Write to File */
		vinsLog.writeToFile();
	}

	private static void runINS(List<IMUReadingsBatch> imuDataset, List<Mat> imgDataset, String logFileName) {
		/* Initialize the controller */
		INSController ins = new INSController();

		/* Initialize the logs */
		FileLog insLog = new FileLog(logFolder + "/" + logFileName);
		insLog.append(ins.getDeviceCoords() + "\n");

		/* Their sizes may not match due to logging problems */
		int datasetSize = Math.min(imuDataset.size(), imgDataset.size());
		System.out.println("DATASET SIZE: IMU = " + imuDataset.size() + " and  IMG = " + imgDataset.size());
		for (int i = 0; i < datasetSize; i++) {

			// System.out.println("\n\nTime Step " + (i + 1));

			/* IMU Predict */
			IMUReadingsBatch currIMUBatch = imuDataset.get(i);
			ins.predict(currIMUBatch);

			/* Update the logs */
			insLog.append(ins.getDeviceCoords() + "\n");

		}
		finalResultsStringBuilder.append("\r\n" + logFileName + "\r\n");
		finalResultsStringBuilder.append("Total steps detected: " + ins.totalStepsDetected + "\r\n");
		finalResultsStringBuilder.append("Total distance traveled: " + ins.totalDistanceTraveled + "\r\n");
		finalResultsStringBuilder.append("Total Displacement = "
				+ ins.getDeviceCoords().computeDistanceTo(new PointDouble(0, 0)) + "\r\n");

		/* Log - Write to File */
		insLog.writeToFile();
	}
}
