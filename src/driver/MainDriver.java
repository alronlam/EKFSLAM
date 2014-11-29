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
import dummies.features.FeatureManager;
import dummies.features.FeatureUpdate;

public class MainDriver {

	static {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
	}

	private static String logFolder = "results";
	private static String breadcrumbLogFileName = "breadcrumb.csv";
	private static String breadcrumbWithCimuHeadingLogFileName = "breadcrumbCimuHeading.csv";
	private static String insLogFileName = "ins.csv";
	private static String insCimuHeadingLogFileName = "insCimuHeading.csv";
	private static String vinsLogFileName = "vins.csv";
	private static String idpLogFileName = "vinsidp.csv";
	private static String doubleIntegrationLogFileName = "doubleintegration.csv";

	private static StringBuilder finalResultsStringBuilder = new StringBuilder();

	public static void main(String[] args) {

		String targetFolder = "data/" + Constants.FOLDER_LS_STRAIGHT;

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

		// runDoubleIntegration(cimuDataset, imgDataset);
		runVINS(cimuDataset, imgDataset);
		// runINS(imuDataset, imgDataset, insLogFileName);
		// runINS(imuDatasetWithCimuHeading, imgDataset, insCimuHeadingLogFileName);
		// runBreadcrumbDummies(imuDataset, imgDataset, breadcrumbLogFileName);
		// runBreadcrumbDummies(imuDatasetWithCimuHeading, imgDataset, breadcrumbWithCimuHeadingLogFileName);
		// runIDP(cimuDataset, imgDataset);
		// runAltogether(imuDataset, imgDataset);

		System.out.println(finalResultsStringBuilder.toString());
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

		finalResultsStringBuilder.append("Total distance traveled " + doubleIntegration.getTotalDistanceTraveled() + "\r\n");
		finalResultsStringBuilder.append("Total Displacement = " + doubleIntegration.getDeviceCoords().computeDistanceTo(new PointDouble(0, 0)) + "\r\n");

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
		for (int i = 0; i < datasetSize; i++) {

			System.out.println("\n\nTime Step " + (i + 1));

			/* IMU Predict */
			IMUReadingsBatch currIMUBatch = imuDataset.get(i);
			breadcrumb.predict(currIMUBatch);
			ins.predict(currIMUBatch);
			// vins.predict(currIMUBatch);
			// vinsIDP.predict(currIMUBatch);
			System.out.println("Finished predicting.");
			/* Image Update */
			FeatureUpdate featureUpdate = featureManager.getFeatureUpdate(imgDataset.get(i));
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

		finalResultsStringBuilder.append("Total Displacement = " + vinsIDP.getDeviceCoords().computeDistanceTo(new PointDouble(0, 0)) + "\r\n");

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

		for (int i = 0; i < datasetSize; i++) {

			System.out.println("Time Step " + (i + 1));

			/* IMU Predict */
			IMUReadingsBatch currIMUBatch = imuDataset.get(i);
			breadcrumb.predict(currIMUBatch);
			// System.out.println("Finished predicting.");

			/* Image Update */
			FeatureUpdate featureUpdate = featureManager.getFeatureUpdate(imgDataset.get(i));
			breadcrumb.update(featureUpdate);
			// System.out.println("Finished updating.");

			System.out.println(breadcrumb.getDeviceCoords() + "\n");

			/* Update the logs */
			breadcrumbLog.append(breadcrumb.getDeviceCoords() + "\n");
		}

		finalResultsStringBuilder.append("Total steps detected " + breadcrumb.totalStepsDetected + "\r\n");
		finalResultsStringBuilder.append("Total distance traveled " + breadcrumb.getTotalDistanceTraveled() + "\r\n");
		finalResultsStringBuilder.append("Total Displacement = " + breadcrumb.getDeviceCoords().computeDistanceTo(new PointDouble(0, 0)) + "\r\n");

		/* Log - Write to File */
		breadcrumbLog.writeToFile();
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
		for (int i = 0; i < datasetSize; i++) {

			System.out.println("\n\nTime Step " + (i + 1));

			/* IMU Predict */
			IMUReadingsBatch currIMUBatch = imuDataset.get(i);
			vins.predict(currIMUBatch);
			// System.out.println("Finished predicting.");

			/* Image Update */
			FeatureUpdate featureUpdate = featureManager.getFeatureUpdate(imgDataset.get(i));
			state[FeatureManager.CURRENT_STEP]++;
			state[5]++;
			System.out.println("Dataset Size: " + datasetSize );
			System.out.println("Initial Delay: " + state[1]);
			System.out.println("Failed due to optical flow: " + state[2]);
			System.out.println("Failed due to essential matrix: " + state[3]);
			System.out.println("Failed due to triangulation: " + state[4]);
			System.out.println("Success/Processed: " + state[0] +"/"+ (state[5]-state[1]));
			System.out.println("Failed/Processed: " + (state[2] + state[3] + state[4]) +"/"+ (state[5]-state[1]) );
			System.out.printf("Success Rate: %.3f%%\n", state[0] * 100.0 / (datasetSize-state[1]) );
			

			vins.update(featureUpdate);
			System.out.println("Finished updating.");

			/* Update the logs */
			vinsLog.append(vins.getDeviceCoords() + "\n");
		}
		
		finalResultsStringBuilder.append("Total distance traveled " + vins.getTotalDistanceTraveled() + "\r\n");
		finalResultsStringBuilder.append("Total Displacement = " + vins.getDeviceCoords().computeDistanceTo(new PointDouble(0, 0)) + "\r\n");

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

		finalResultsStringBuilder.append("Total steps detected: " + ins.totalStepsDetected + "\r\n");
		finalResultsStringBuilder.append("Total distance traveled: " + ins.totalDistanceTraveled + "\r\n");
		finalResultsStringBuilder.append("Total Displacement = " + ins.getDeviceCoords().computeDistanceTo(new PointDouble(0, 0)) + "\r\n");

		/* Log - Write to File */
		insLog.writeToFile();
	}
}
