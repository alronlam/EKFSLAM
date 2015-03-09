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

	private static StringBuilder finalResultsStringBuilder;

	public static void main(String[] args) {

		ArrayList<String> datasets = getDatasetsToRun();

		for (String datasetName : datasets) {

			String targetFolder = "data/" + datasetName;
			// boolean isDatasetAsync = false;
			// if (Constants.ASYNC_DATASETS.contains(datasetName)) {
			// isDatasetAsync = true;
			// }
			finalResultsStringBuilder = new StringBuilder();
			finalResultsStringBuilder.append(datasetName + "\r\n");

			/* Load IMU Dataset */
			IMULogReader imuLogReader = new IMULogReader(targetFolder + "/imu");
			List<IMUReadingsBatch> imuDataset = imuLogReader.readSensorEntries();

			IMULogReader cimuLogReader = new IMULogReader(targetFolder + "/cimu");
			List<IMUReadingsBatch> cimuDataset = cimuLogReader.readSensorEntries();

			/* Load Images Dataset */
			ImgLogReader imgLogReader = new ImgLogReader(targetFolder + "/img");
			List<Mat> imgDataset = imgLogReader.readImages();

			/* Change IMU Dataset with Camera Heading */
			List<IMUReadingsBatch> imuDatasetWithCimuHeading = changeHeading(imuDataset, cimuDataset);

			// runINS(imuDataset, imgDataset, datasetName, insLogFileName);
			// runINS(imuDatasetWithCimuHeading, imgDataset, datasetName,
			// insCimuHeadingLogFileName);
			// runDoubleIntegration(cimuDataset, imgDataset, datasetName,
			// doubleIntegrationLogFileName);
			// runVINSAsync(cimuDataset, imgDataset, datasetName,
			// vinsLogFileName,
			// false);
			// runVINSAsync(cimuDataset, imgDataset, datasetName,
			// vins15hzLogFileName, true);
			// runBreadcrumbAsync(imuDatasetWithCimuHeading, imgDataset,
			// datasetName, breadcrumbWithCimuHeadingLogFileName,
			// false);
			// runBreadcrumbAsync(imuDatasetWithCimuHeading, imgDataset,
			// datasetName,
			// breadcrumbWithCimuHeading15hzLogFileName, true);
			runAltogether(imuDataset, imuDatasetWithCimuHeading, cimuDataset, imgDataset, datasetName);
			System.out.println(finalResultsStringBuilder.toString());
		}
	}

	private static ArrayList<String> getDatasetsToRun() {
		ArrayList<String> datasets = new ArrayList<String>();
		// datasets.add(Constants.FOLDER_RECT1_MIGUEL2_S3);
		// datasets.add(Constants.FOLDER_RECT1_MIGUEL3_S3);
		// datasets.add(Constants.FOLDER_RECT1_MIGUEL3_S4);
		// datasets.add(Constants.FOLDER_RECT1_MIGUEL4_S4);
		// datasets.add(Constants.FOLDER_RECT2_MIGUEL4_S4);
		// datasets.add(Constants.FOLDER_STRT1_SJ5_S4);
		// datasets.add(Constants.FOLDER_STRT1_SJ6_S4_MAR3);
		// datasets.add(Constants.FOLDER_STRT2_SJ6_S4);
		// datasets.add(Constants.FOLDER_STRT2_SJ6_S4_MAR3);

		// datasets.add(Constants.FOLDER_RECT1_MIGUEL_S4_ALRON_MAR5);
		// datasets.add(Constants.FOLDER_RECT2_MIGUEL_S4_ALRON_MAR5);
		// datasets.add(Constants.FOLDER_STRT1_SJ6_PARTIAL_S4_MAR5_BLACK_CAM);
		// datasets.add(Constants.FOLDER_STRT2_SJ6_PARTIAL_S4_MAR5);
		datasets.add(Constants.FOLDER_RECT1_MIGUEL_S4_IVAN_MAR5);
		// datasets.add(Constants.FOLDER_RECT2_MIGUEL_S4_IVAN_MAR5);

		// datasets.add(Constants.FOLDER_RECT1_MIGUEL_S4_IVAN_MAR6);
		// datasets.add(Constants.FOLDER_RECT2_MIGUEL_S4_IVAN_MAR6);
		// datasets.add(Constants.FOLDER_STRT1_SJ6_S4_MAR6);
		// datasets.add(Constants.FOLDER_STRT1_LS1_S3CAM_IVAN_MAR6);
		// datasets.add(Constants.FOLDER_STRT2_LS1_S3CAM_IVAN_MAR6);

		return datasets;
	}

	private static void runAltogether(List<IMUReadingsBatch> imuDataset, List<IMUReadingsBatch> imuDatasetWithCimuHeading, List<IMUReadingsBatch> cimuDataset,
			List<Mat> imgDataset, String datasetName) {
//		runINS(imuDataset, imgDataset, datasetName, insLogFileName);
//		runINS(imuDatasetWithCimuHeading, imgDataset, datasetName, insCimuHeadingLogFileName);
//		runDoubleIntegration(cimuDataset, imgDataset, datasetName, doubleIntegrationLogFileName);
		runVINSAsync(cimuDataset, imgDataset, datasetName, vinsLogFileName, false);
		runVINSAsync(cimuDataset, imgDataset, datasetName, vins15hzLogFileName, true);
//		runBreadcrumbAsync(imuDatasetWithCimuHeading, imgDataset, datasetName, breadcrumbWithCimuHeadingLogFileName, false);
//		runBreadcrumbAsync(imuDatasetWithCimuHeading, imgDataset, datasetName, breadcrumbWithCimuHeading15hzLogFileName, true);

		FileLog finalResultsLog = new FileLog(logFolder + "/" + datasetName + "/" + datasetName + ".txt");
		finalResultsLog.append(finalResultsStringBuilder.toString());
		finalResultsLog.writeToFile();
	}

	private static List<IMUReadingsBatch> changeHeading(List<IMUReadingsBatch> originalIMUDataset, List<IMUReadingsBatch> cimuDataset) {

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

	private static void resetFeatureRelatedStaticVars() {
		FeatureScaler.resetInstance();
		EKFScalingCorrecter.resetInstance();
		FeatureData.resetCameraPositions();
	}

	private static void runINS(List<IMUReadingsBatch> imuDataset, List<Mat> imgDataset, String datasetName, String logFileName) {
		/* Initialize the controller */
		INSController ins = new INSController();

		/* Initialize the logs */
		FileLog insLog, summaryLog, speedLog;
		String genericFileName = logFolder + "/" + datasetName + "/" + logFileName;
		insLog = new FileLog(genericFileName);
		summaryLog = new FileLog(genericFileName.split("[.]")[0] + "_summary.txt");
		speedLog = new FileLog(genericFileName.split("[.]")[0] + "_timeElapsed.csv");

		insLog.append(ins.getDeviceCoords() + "\n");

		/* Speed recording variables */
		long startTime, endTime, loopDuration, maxTime, totalTime;
		List<Long> loopDurationList;

		loopDurationList = new ArrayList<>();
		totalTime = 0;
		maxTime = 0;

		/* Their sizes may not match due to logging problems */
		int datasetSize = Math.min(imuDataset.size(), imgDataset.size());
		System.out.println("DATASET SIZE: IMU = " + imuDataset.size() + " and  IMG = " + imgDataset.size());

		for (int i = 0; i < datasetSize; i++) {
			System.out.println("\n\nTime Step " + (i + 1));

			startTime = System.currentTimeMillis();

			/* IMU Predict */
			IMUReadingsBatch currIMUBatch = imuDataset.get(i);
			ins.predict(currIMUBatch);

			/* Update the logs */
			insLog.append(ins.getDeviceCoords() + "\n");

			/* Measure time elapsed */
			endTime = System.currentTimeMillis();

			loopDuration = endTime - startTime;

			if (maxTime < loopDuration)
				maxTime = loopDuration;
			loopDurationList.add(loopDuration);
			totalTime += loopDuration;
		}
		summaryLog.append(logFileName + "\r\n\r\n");
		summaryLog.append("Total steps detected: " + ins.totalStepsDetected + " steps\r\n");
		summaryLog.append("Total distance traveled: " + ins.totalDistanceTraveled + " m\r\n");
		summaryLog.append("Total Displacement = " + ins.getDeviceCoords().computeDistanceTo(new PointDouble(0, 0)) + " m\r\n\r\n");

		for (Long loopTime : loopDurationList) {
			speedLog.append(loopTime + "\n");
		}
		summaryLog.append("Total Time Elapsed: " + totalTime + " ms\r\n");
		summaryLog.append("Average Time: " + String.format("%.3f", (totalTime / (double) loopDurationList.size())) + " ms\r\n");
		summaryLog.append("Max Time: " + maxTime + " ms\r\n");

		/* Log - Write to File */
		insLog.writeToFile();
		summaryLog.writeToFile();
		speedLog.writeToFile();
	}

	private static void runDoubleIntegration(List<IMUReadingsBatch> imuDataset, List<Mat> imgDataset, String datasetName, String logFileName) {
		DoubleIntegrationController doubleIntegration = new DoubleIntegrationController();

		/* Initialize the logs for all three techniques */
		FileLog doubleIntegrationLog, summaryLog, speedLog;
		String genericFileName = logFolder + "/" + datasetName + "/" + logFileName;
		doubleIntegrationLog = new FileLog(genericFileName);
		summaryLog = new FileLog(genericFileName.split("[.]")[0] + "_summary.txt");
		speedLog = new FileLog(genericFileName.split("[.]")[0] + "_timeElapsed.csv");

		doubleIntegrationLog.append(doubleIntegration.getDeviceCoords() + "\n");

		/* Speed recording variables */
		long startTime, endTime, loopDuration, maxTime, totalTime;
		List<Long> loopDurationList;

		loopDurationList = new ArrayList<>();
		totalTime = 0;
		maxTime = 0;

		/* Their sizes may not match due to logging problems */
		int datasetSize = Math.min(imuDataset.size(), imgDataset.size());
		System.out.println("DATASET SIZE: IMU = " + imuDataset.size() + " and  IMG = " + imgDataset.size());

		for (int i = 0; i < datasetSize; i++) {
			System.out.println("\n\nTime Step " + (i + 1));

			startTime = System.currentTimeMillis();

			/* IMU Predict */
			IMUReadingsBatch currIMUBatch = imuDataset.get(i);
			doubleIntegration.predict(currIMUBatch);
			// System.out.println("Finished predicting.");

			doubleIntegrationLog.append(doubleIntegration.getDeviceCoords() + "\n");

			/* Measure time elapsed */
			endTime = System.currentTimeMillis();

			loopDuration = endTime - startTime;

			if (maxTime < loopDuration)
				maxTime = loopDuration;
			loopDurationList.add(loopDuration);
			totalTime += loopDuration;
		}
		summaryLog.append(logFileName + "\r\n\r\n");
		summaryLog.append("Total distance traveled " + doubleIntegration.getTotalDistanceTraveled() + " m\r\n");
		summaryLog.append("Total Displacement = " + doubleIntegration.getDeviceCoords().computeDistanceTo(new PointDouble(0, 0)) + " m\r\n\r\n");

		for (Long loopTime : loopDurationList) {
			speedLog.append(loopTime + "\n");
		}
		summaryLog.append("Total Time Elapsed: " + totalTime + " ms\r\n");
		summaryLog.append("Average Time: " + String.format("%.3f", (totalTime / (double) loopDurationList.size())) + " ms\r\n");
		summaryLog.append("Max Time: " + maxTime + " ms\r\n");

		/* Log - Write to File */
		doubleIntegrationLog.writeToFile();
		summaryLog.writeToFile();
		speedLog.writeToFile();
	}

	private static void runVINSAsync(List<IMUReadingsBatch> imuDataset, List<Mat> imgDataset, String datasetName, String logFileName, boolean isAsync) {
		resetFeatureRelatedStaticVars();
		System.out.println("init 1");
		/* Initialize the controller and manager */
		VINSController vins = new VINSController();
		FeatureManager featureManager = new FeatureManager();

		/* Initialize the logs for all three techniques */
		FileLog vinsLog, summaryLog, speedLog;
		String genericFileName = logFolder + "/" + datasetName + "/" + logFileName;
		vinsLog = new FileLog(genericFileName);
		summaryLog = new FileLog(genericFileName.split("[.]")[0] + "_summary.txt");
		speedLog = new FileLog(genericFileName.split("[.]")[0] + "_timeElapsed.csv");

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

		/* Speed recording variables */
		long startTime, endTime, loopDuration, maxTime, totalTime;
		int featureCount, maxFeatureCount, totalFeatureCount;
		List<Long> loopDurationList;
		List<Integer> featureCountList;

		loopDurationList = new ArrayList<>();
		featureCountList = new ArrayList<>();
		totalTime = 0;
		maxTime = 0;
		totalFeatureCount = 0;
		maxFeatureCount = 0;

		while (true) {
			if (imuIndex >= imuDataset.size() || imgIndex >= imgDataset.size()) {
				break;
			}

			StringBuilder sb = new StringBuilder();

			if (elapsedTime >= Constants.MS_IMU_DURATION) {
				System.out.println("\nTime Step " + (timeStep + 1));

				startTime = System.currentTimeMillis();

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
					FeatureUpdate featureUpdate = featureManager.getAsyncFeatureUpdate(imgDataset.get(imgIndex), transX, transY, vins.getDeviceCoords());
					valid[(FeatureManager.VALID_ROTATION == FeatureManager.ROT_1 ? 0 : 2) + (FeatureManager.VALID_TRANSLATION == FeatureManager.TRAN_1 ? 0 : 1)]++;
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

				/* Measure time elapsed */
				endTime = System.currentTimeMillis();

				loopDuration = endTime - startTime;
				featureCount = vins.getCurrentFeatureCount();

				if (maxTime < loopDuration)
					maxTime = loopDuration;
				if (maxFeatureCount < featureCount)
					maxFeatureCount = featureCount;
				loopDurationList.add(loopDuration);
				featureCountList.add(featureCount);
				totalTime += loopDuration;
				totalFeatureCount += featureCount;
			} else {
				// sb.append("Image Flow. ");
				// sb.append("img: " + imgIndex + " ");
				// sb.append(elapsedTime + "ms");
				// System.out.println(sb.toString());
				if (isAsync)
					featureManager.flowImage(imgDataset.get(imgIndex));
			}

			imgIndex++;

			elapsedTime += Constants.MS_IMG_DURATION;

			timeStep++;
		}
		vinsLog.append(EKFScalingCorrecter.getEKFScalingResultCorrecter().getCorrectedPositionsAsString());

		summaryLog.append("Rot 1 & Tran 1: " + valid[0] + "\r\n");
		summaryLog.append("Rot 1 & Tran 2: " + valid[1] + "\r\n");
		summaryLog.append("Rot 2 & Tran 1: " + valid[2] + "\r\n");
		summaryLog.append("Rot 1 & Tran 2: " + valid[3] + "\r\n");
		summaryLog.append("Success/Processed: " + state[0] + "/" + (state[5] - state[1]) + "\r\n");
		summaryLog.append("Initial Delay: " + state[1] + "\r\n");
		summaryLog.append("Failed due to optical flow: " + state[2] + "\r\n");
		summaryLog.append("Failed due to essential matrix: " + state[3] + "\r\n");
		summaryLog.append("Failed due to triangulation: " + state[4] + "\r\n");
		summaryLog.append("Success/Processed: " + state[0] + "/" + (state[5] - state[1]) + "\r\n");
		summaryLog.append("Failed/Processed: " + (state[2] + state[3] + state[4]) + "/" + (state[5] - state[1]) + "\r\n");
		summaryLog.append("Success Rate: " + String.format("%.3f", (state[0] * 100.0 / (state[5] - state[1]))) + "\r\n");
		summaryLog.append("\r\n");

		summaryLog.append(logFileName + "\r\n\r\n");
		summaryLog.append("Total distance traveled " + EKFScalingCorrecter.getEKFScalingResultCorrecter().getTotalDistanceTraveled() + " m\r\n");
		summaryLog.append("Total Displacement = " + EKFScalingCorrecter.getEKFScalingResultCorrecter().getFinalPosition().computeDistanceTo(new PointDouble(0, 0)) + " m\r\n\r\n");

		for (int i = 0; i < loopDurationList.size(); ++i) {
			speedLog.append(loopDurationList.get(i) + "," + featureCountList.get(i) + "\n");
		}
		summaryLog.append("Total Time Elapsed: " + totalTime + " ms\r\n");
		summaryLog.append("Average Time: " + String.format("%.3f", (totalTime / (double) loopDurationList.size())) + " ms\r\n");
		summaryLog.append("Max Time: " + maxTime + " ms\r\n\r\n");

		summaryLog.append("Average Feature Count: " + String.format("%.3f", (totalFeatureCount / (double) featureCountList.size())) + " features\r\n");
		summaryLog.append("Max Feature Count: " + maxFeatureCount + " features\r\n");

		// finalResultsStringBuilder.append("Total distance traveled " +
		// vins.getTotalDistanceTraveled() + "\r\n");
		// finalResultsStringBuilder.append("Total Displacement = " +
		// vins.getDeviceCoords().computeDistanceTo(new PointDouble(0, 0)) +
		// "\r\n");

		/* Log - Write to File */
		vinsLog.writeToFile();
		summaryLog.writeToFile();
		speedLog.writeToFile();
	}

	/* Based on runBreadcrumbDummies */
	private static void runBreadcrumbAsync(List<IMUReadingsBatch> imuDataset, List<Mat> imgDataset, String datasetName, String logFileName, boolean isAsync) {
		resetFeatureRelatedStaticVars();

		/* Initialize the controller and manager */
		BreadcrumbDummiesController breadcrumb = new BreadcrumbDummiesController();
		FeatureManager featureManager = new FeatureManager();

		/* Initialize the logs */
		FileLog breadcrumbLog, summaryLog, speedLog;
		String genericFileName = logFolder + "/" + datasetName + "/" + logFileName;
		breadcrumbLog = new FileLog(genericFileName);
		summaryLog = new FileLog(genericFileName.split("[.]")[0] + "_summary.txt");
		speedLog = new FileLog(genericFileName.split("[.]")[0] + "_timeElapsed.csv");

		breadcrumbLog.append(breadcrumb.getDeviceCoords() + "\n");

		System.out.println("DATASET SIZE: IMU = " + imuDataset.size() + " and  IMG = " + imgDataset.size());

		/* Visual correction Success variables */
		int state[] = new int[6];
		int valid[] = new int[4];

		double prevX = 0;
		double prevY = 0;
		PointDouble prevPoint = new PointDouble(Double.MAX_VALUE, Double.MAX_VALUE);

		int imuIndex = 0;
		int imgIndex = 0;
		int elapsedTime = Constants.MS_IMU_DURATION;
		int timeStep = 0;

		/* Speed recording variables */
		long startTime, endTime, loopDuration, maxTime, totalTime;
		int featureCount, maxFeatureCount, totalFeatureCount;
		List<Long> loopDurationList;
		List<Integer> featureCountList;

		loopDurationList = new ArrayList<>();
		featureCountList = new ArrayList<>();
		totalTime = 0;
		maxTime = 0;
		totalFeatureCount = 0;
		maxFeatureCount = 0;

		while (true) {
			if (imuIndex >= imuDataset.size() || imgIndex >= imgDataset.size()) {
				break;
			}

			StringBuilder sb = new StringBuilder();
			if (elapsedTime >= Constants.MS_IMU_DURATION) {
				startTime = System.currentTimeMillis();

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
					FeatureUpdate featureUpdate = featureManager.getAsyncFeatureUpdate(imgDataset.get(imgIndex), transX, transY, breadcrumb.getDeviceCoords());
					valid[(FeatureManager.VALID_ROTATION == FeatureManager.ROT_1 ? 0 : 2) + (FeatureManager.VALID_TRANSLATION == FeatureManager.TRAN_1 ? 0 : 1)]++;
					state[FeatureManager.CURRENT_STEP]++;
					state[5]++;

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

				/* Measure time elapsed */
				endTime = System.currentTimeMillis();

				loopDuration = endTime - startTime;
				featureCount = breadcrumb.getCurrentFeatureCount();

				if (maxTime < loopDuration)
					maxTime = loopDuration;
				if (maxFeatureCount < featureCount)
					maxFeatureCount = featureCount;
				loopDurationList.add(loopDuration);
				featureCountList.add(featureCount);
				totalTime += loopDuration;
				totalFeatureCount += featureCount;
			} else {
				if (isAsync)
					featureManager.flowImage(imgDataset.get(imgIndex));
			}

			imgIndex++;

			elapsedTime += Constants.MS_IMG_DURATION;

			timeStep++;
		}

		breadcrumbLog.append(EKFScalingCorrecter.getEKFScalingResultCorrecter().getCorrectedPositionsAsString());

		summaryLog.append("Rot 1 & Tran 1: " + valid[0] + "\r\n");
		summaryLog.append("Rot 1 & Tran 2: " + valid[1] + "\r\n");
		summaryLog.append("Rot 2 & Tran 1: " + valid[2] + "\r\n");
		summaryLog.append("Rot 1 & Tran 2: " + valid[3] + "\r\n");
		summaryLog.append("Success/Processed: " + state[0] + "/" + (state[5] - state[1]) + "\r\n");
		summaryLog.append("Initial Delay: " + state[1] + "\r\n");
		summaryLog.append("Failed due to optical flow: " + state[2] + "\r\n");
		summaryLog.append("Failed due to essential matrix: " + state[3] + "\r\n");
		summaryLog.append("Failed due to triangulation: " + state[4] + "\r\n");
		summaryLog.append("Success/Processed: " + state[0] + "/" + (state[5] - state[1]) + "\r\n");
		summaryLog.append("Failed/Processed: " + (state[2] + state[3] + state[4]) + "/" + (state[5] - state[1]) + "\r\n");
		summaryLog.append("Success Rate: " + String.format("%.3f", (state[0] * 100.0 / (state[5] - state[1]))) + "\r\n");
		summaryLog.append("\r\n");

		summaryLog.append(logFileName + "\r\n\r\n");
		summaryLog.append("Total steps detected " + breadcrumb.totalStepsDetected + " steps\r\n");
		// finalResultsStringBuilder.append("Total distance traveled " +
		// breadcrumb.getTotalDistanceTraveled() + "\r\n");
		// finalResultsStringBuilder.append("Total Displacement = "
		// + breadcrumb.getDeviceCoords().computeDistanceTo(new PointDouble(0,
		// 0)) + "\r\n");

		summaryLog.append("Total distance traveled " + EKFScalingCorrecter.getEKFScalingResultCorrecter().getTotalDistanceTraveled() + " m\r\n");
		summaryLog.append("Total Displacement = " + EKFScalingCorrecter.getEKFScalingResultCorrecter().getFinalPosition().computeDistanceTo(new PointDouble(0, 0)) + " m\r\n\r\n");

		for (int i = 0; i < loopDurationList.size(); ++i) {
			speedLog.append(loopDurationList.get(i) + "," + featureCountList.get(i) + "\n");
		}
		summaryLog.append("Total Time Elapsed: " + totalTime + " ms\r\n");
		summaryLog.append("Average Time: " + String.format("%.3f", (totalTime / (double) loopDurationList.size())) + " ms\r\n");
		summaryLog.append("Max Time: " + maxTime + " ms\r\n\r\n");

		summaryLog.append("Average Feature Count: " + String.format("%.3f", (totalFeatureCount / (double) featureCountList.size())) + " features\r\n");
		summaryLog.append("Max Feature Count: " + maxFeatureCount + " features\r\n");

		/* Log - Write to File */
		breadcrumbLog.writeToFile();
		summaryLog.writeToFile();
		speedLog.writeToFile();
	}

	/** Depreciated **/
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
		finalResultsStringBuilder.append("Total Displacement = " + vinsIDP.getDeviceCoords().computeDistanceTo(new PointDouble(0, 0)) + "\r\n");

		/* Log - Write to File */
		vinsIDPLog.writeToFile();
	}

	/** Depreciated **/
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
				FeatureUpdate featureUpdate = featureManager.getFeatureUpdate(imgDataset.get(i), transX, transY, breadcrumb.getDeviceCoords());
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

		finalResultsStringBuilder.append("Total distance traveled " + EKFScalingCorrecter.getEKFScalingResultCorrecter().getTotalDistanceTraveled() + "\r\n");
		finalResultsStringBuilder.append("Total Displacement = " + EKFScalingCorrecter.getEKFScalingResultCorrecter().getFinalPosition().computeDistanceTo(new PointDouble(0, 0))
				+ "\r\n");

		/* Log - Write to File */
		breadcrumbLog.writeToFile();
	}

	/** Depreciated **/
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
			FeatureUpdate featureUpdate = featureManager.getFeatureUpdate(imgDataset.get(i), transX, transY, vins.getDeviceCoords());
			valid[(FeatureManager.VALID_ROTATION == FeatureManager.ROT_1 ? 0 : 2) + (FeatureManager.VALID_TRANSLATION == FeatureManager.TRAN_1 ? 0 : 1)]++;
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

		finalResultsStringBuilder.append("Total distance traveled " + EKFScalingCorrecter.getEKFScalingResultCorrecter().getTotalDistanceTraveled() + "\r\n");
		finalResultsStringBuilder.append("Total Displacement = " + EKFScalingCorrecter.getEKFScalingResultCorrecter().getFinalPosition().computeDistanceTo(new PointDouble(0, 0))
				+ "\r\n");

		// finalResultsStringBuilder.append("Total distance traveled " +
		// vins.getTotalDistanceTraveled() + "\r\n");
		// finalResultsStringBuilder.append("Total Displacement = " +
		// vins.getDeviceCoords().computeDistanceTo(new PointDouble(0, 0)) +
		// "\r\n");

		/* Log - Write to File */
		vinsLog.writeToFile();
	}

	/** Depreciated **/
	private static void runVINSDummy(List<IMUReadingsBatch> imuDataset, List<Mat> imgDataset, String logFileName, boolean isAsync) {
		resetFeatureRelatedStaticVars();

		/* Initialize the controller and manager */
		VINSController vins = new VINSController();
		FeatureManager featureManager = new FeatureManager();

		/* Initialize the logs */
		FileLog vinsLog = new FileLog(logFolder + "/" + logFileName);
		vinsLog.append(vins.getDeviceCoords() + "\n");

		int state[] = new int[6];
		int valid[] = new int[4];
		double prevX = 0;
		double prevY = 0;
		PointDouble prevPoint = new PointDouble(Double.MAX_VALUE, Double.MAX_VALUE);

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
					FeatureUpdate featureUpdate = featureManager.getAsyncFeatureUpdate(imgDataset.get(imgIndex), transX, transY, vins.getDeviceCoords());
					valid[(FeatureManager.VALID_ROTATION == FeatureManager.ROT_1 ? 0 : 2) + (FeatureManager.VALID_TRANSLATION == FeatureManager.TRAN_1 ? 0 : 1)]++;
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

	}

}
