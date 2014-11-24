package dummies;

import java.util.Collections;
import java.util.List;

import stepbasedins.controller.StepBasedINSController;
import stepbasedins.data.BatchProcessingResults;

import commondata.PointDouble;

import desktop.imu.IMUReadingsBatch;
import dummies.ekf.EKF;
import dummies.features.FeatureUpdate;

public class BreadcrumbDummiesController {

	private EKF ekf;
	private StepBasedINSController ins;
	private int featureUpdateNullCount = 0;

	public BreadcrumbDummiesController() {
		this.ekf = new EKF();
		this.ins = new StepBasedINSController();
	}

	public PointDouble getDeviceCoords() {
		return ekf.getDeviceCoords();
	}

	public void predict(IMUReadingsBatch imuBatch) {
		BatchProcessingResults result = this.ins.processSensorEntryBatch(imuBatch.getEntries());

		ekf.predictFromINS(result.getStrideLength(), Math.toRadians(result.getHeadingAngle()));
	}

	public void update(FeatureUpdate featureUpdate) {

		if (featureUpdate != null) {
			featureUpdateNullCount = 0;
			/* Delete features that disappeared */
			List<Integer> toDelete = featureUpdate.getBadPointsIndex();
			System.out.println("To Delete:" + toDelete.size());
			Collections.reverse(toDelete);
			for (Integer index : toDelete)
				ekf.deleteFeature(index);

			/* Update using re-observed features */
			List<PointDouble> toUpdate = featureUpdate.getCurrentPoints();

			System.out.println("To Update:" + toUpdate.size());
			for (int i = 0; i < toUpdate.size(); i++) {
				PointDouble currXY = toUpdate.get(i);
				ekf.updateFromReobservedFeatureCoords(i, currXY.getX(), currXY.getY());
			}

			/* Add new features */
			List<PointDouble> toAdd = featureUpdate.getNewPoints();
			System.out.println("To Add:" + toAdd.size());

			for (PointDouble featpos : toAdd)
				ekf.addFeature(featpos.getX(), featpos.getY());
		} else {
			featureUpdateNullCount++;
			if (featureUpdateNullCount == 3)
				ekf.deleteAllFeatures();
		}
	}
}
