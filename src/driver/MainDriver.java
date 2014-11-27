package driver;

import idp.VINSIDPController;

import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.Mat;

import stepbasedins.INSController;
import util.FileLog;
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
	private static String insLogFileName = "ins.csv";
	private static String vinsLogFileName = "vins.csv";
	private static String idpLogFileName = "vinsidp.csv";

	public static void main(String[] args) {

		String targetFolder = "data/" + Constants.FOLDER_MIGUEL_STRAIGHT;

		/* Load IMU Dataset */
		IMULogReader imuLogReader = new IMULogReader(targetFolder + "/imu");
		List<IMUReadingsBatch> imuDataset = imuLogReader.readSensorEntries();

		IMULogReader cimuLogReader = new IMULogReader(targetFolder + "/cimu");
		List<IMUReadingsBatch> cimuDataset = imuLogReader.readSensorEntries();

		/* Load Images Dataset */
		ImgLogReader imgLogReader = new ImgLogReader(targetFolder + "/img");
		List<Mat> imgDataset = imgLogReader.readImages();

<<<<<<< HEAD
		runINS(imuDataset, imgDataset);
		runVINS(cimuDataset, imgDataset);
		runBreadcrumbDummies(imuDataset, imgDataset);
		// runIDP(cimuDataset, imgDataset);
=======
		// runINS(imuDataset, imgDataset);
		// runVINS(imuDataset, imgDataset);
		// runBreadcrumbDummies(imuDataset, imgDataset);
		runIDP(imuDataset, imgDataset);
>>>>>>> 613f4d3cd9b56ec26e04f822f1b0c25ef45e8a2c
		// runAltogether(imuDataset, imgDataset);
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
			vinsIDPLog.append((vinsIDP.getDeviceCoords().getX() / 100000) + ","
					+ (vinsIDP.getDeviceCoords().getY() / 100000) + "\n");
			// vinsIDPLog.append(vinsIDP.getDeviceCoords() + "\n");
		}

		System.out
				.println("Total Displacement = " + vinsIDP.getDeviceCoords().computeDistanceTo(new PointDouble(0, 0)));

		/* Log - Write to File */
		vinsIDPLog.writeToFile();
	}

	private static void runBreadcrumbDummies(List<IMUReadingsBatch> imuDataset, List<Mat> imgDataset) {
		/* Initialize the controller and manager */
		BreadcrumbDummiesController breadcrumb = new BreadcrumbDummiesController();
		FeatureManager featureManager = new FeatureManager();

		/* Initialize the logs */
		FileLog breadcrumbLog = new FileLog(logFolder + "/" + breadcrumbLogFileName);
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

		System.out.println("Total steps detected " + breadcrumb.totalStepsDetected);
		System.out.println("Total distance traveled " + breadcrumb.totalDistanceTraveled);
		System.out.println("Total Displacement = "
				+ breadcrumb.getDeviceCoords().computeDistanceTo(new PointDouble(0, 0)));

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

		for (int i = 0; i < datasetSize; i++) {

			System.out.println("\n\nTime Step " + (i + 1));

			/* IMU Predict */
			IMUReadingsBatch currIMUBatch = imuDataset.get(i);
			vins.predict(currIMUBatch);
			// System.out.println("Finished predicting.");

			/* Image Update */
			FeatureUpdate featureUpdate = featureManager.getFeatureUpdate(imgDataset.get(i));
			vins.update(featureUpdate);
			System.out.println("Finished updating.");

			/* Update the logs */
			vinsLog.append(vins.getDeviceCoords() + "\n");
		}
		System.out.println("Total Displacement = " + vins.getDeviceCoords().computeDistanceTo(new PointDouble(0, 0)));

		/* Log - Write to File */
		vinsLog.writeToFile();
	}

	private static void runINS(List<IMUReadingsBatch> imuDataset, List<Mat> imgDataset) {
		/* Initialize the controller */
		INSController ins = new INSController();

		/* Initialize the logs */
		FileLog insLog = new FileLog(logFolder + "/" + insLogFileName);
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

		System.out.println("Total steps detected: " + ins.totalStepsDetected);
		System.out.println("Total distance traveled: " + ins.totalDistanceTraveled);
		System.out.println("Total Displacement = " + ins.getDeviceCoords().computeDistanceTo(new PointDouble(0, 0)));

		/* Log - Write to File */
		insLog.writeToFile();
	}
}
