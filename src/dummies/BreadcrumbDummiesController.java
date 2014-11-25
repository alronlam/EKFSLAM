package dummies;

import java.util.Collections;
import java.util.List;

import stepbasedins.controller.StepBasedINSController;
import stepbasedins.data.BatchProcessingResults;
import util.FileLog;

import commondata.PointDouble;

import desktop.imu.IMUReadingsBatch;
import dummies.ekf.EKF;
import dummies.features.FeatureUpdate;

public class BreadcrumbDummiesController {

	private EKF ekf;
	private StepBasedINSController ins;
	private int featureUpdateNullCount = 0;
	private int updateCount = 0;
	public int totalStepsDetected;
	public double totalDistanceTraveled = 0;

	public BreadcrumbDummiesController() {
		this.ekf = new EKF();
		this.ins = new StepBasedINSController();
	}

	public PointDouble getDeviceCoords() {
		return ekf.getDeviceCoords();
	}

	public void predict(IMUReadingsBatch imuBatch) {
		BatchProcessingResults result = this.ins.processSensorEntryBatch(imuBatch.getEntries());
		totalStepsDetected += result.getDetectedSteps();
		totalDistanceTraveled += result.getStrideLength();
		ekf.predictFromINS(result.getStrideLength(), Math.toRadians(result.getHeadingAngle()));
	}

	public void update(FeatureUpdate featureUpdate) {

		updateCount++;

		FileLog pointsLog = new FileLog("logs/" + String.format("%5d", updateCount) + ".csv");

		if (featureUpdate != null) {
			featureUpdateNullCount = 0;

			List<Integer> toDelete = featureUpdate.getBadPointsIndex();
			List<PointDouble> toUpdate = featureUpdate.getCurrentPoints();
			List<PointDouble> toAdd = featureUpdate.getNewPoints();

			System.out.println("To Delete:" + toDelete.size());
			System.out.println("To Update:" + toUpdate.size());
			System.out.println("To Add:" + toAdd.size());

			pointsLog.append("Update Points:\r\n");
			for (PointDouble update : toUpdate)
				pointsLog.append(update.toString() + "\r\n");

			pointsLog.append("\r\nAdd Points:\r\n");
			for (PointDouble add : toAdd)
				pointsLog.append(add.toString() + "\r\n");

			pointsLog.writeToFile();

			/* Delete features that disappeared */
			Collections.reverse(toDelete);
			for (Integer index : toDelete)
				ekf.deleteFeature(index);

			/* Update using re-observed features */
			for (int i = 0; i < toUpdate.size(); i++) {
				PointDouble currXY = toUpdate.get(i);
				ekf.updateFromReobservedFeatureCoords(i, currXY.getX(), currXY.getY());
			}

			/* Add new features */
			for (PointDouble featpos : toAdd)
				ekf.addFeature(featpos.getX(), featpos.getY());
		} else {
			featureUpdateNullCount++;
			if (featureUpdateNullCount == 3)
				ekf.deleteAllFeatures();
		}

		System.out.println("Coords are now: " + ekf.getDeviceCoords());
	}
}
