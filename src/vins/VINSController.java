package vins;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import stepbasedins.data.SensorEntry;
import vins.motionestimation.IntegrateMotionEstimation;
import vins.motionestimation.MotionEstimation;

import commondata.DevicePose;
import commondata.PointDouble;

import desktop.imu.IMUReadingsBatch;
import dummies.ekf.EKF;
import dummies.ekf.EKFController;
import dummies.features.FeatureUpdate;

public class VINSController implements EKFController {

	private EKF ekf;
	private MotionEstimation motionEstimator;
	private int featureUpdateNullCount = 0;

	private List<PointDouble> coordinates;

	public VINSController() {
		this.ekf = new EKF();
		this.motionEstimator = new IntegrateMotionEstimation();

		this.coordinates = new ArrayList<PointDouble>();
	}

	public void predict(IMUReadingsBatch batch) {
		ArrayList<SensorEntry> sensorEntries = batch.getEntries();
		for (SensorEntry se : sensorEntries)
			motionEstimator.inputData(se);
		try {
			DevicePose devicePose = motionEstimator.getHeadingAndDisplacement();

			ekf.predictFromINS(devicePose.getXYDistance(), devicePose.getHeadingRadians());
			coordinates.add(ekf.getDeviceCoords());
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	public void update(FeatureUpdate featureUpdate) {

		if (featureUpdate != null) {

			featureUpdateNullCount = 0;

			/* Delete features that disappeared */
			List<Integer> toDelete = featureUpdate.getBadPointsIndex();
			Collections.reverse(toDelete);
			for (Integer index : toDelete)
				ekf.deleteFeature(index);

			/* Update using re-observed features */
			List<PointDouble> toUpdate = featureUpdate.getCurrentPoints();
			for (int i = 0; i < toUpdate.size(); i++) {
				PointDouble currXY = toUpdate.get(i);
				ekf.updateFromReobservedFeatureCoords(i, currXY.getX(), currXY.getY());
			}

			coordinates.set(coordinates.size() - 1, ekf.getDeviceCoords());
			/* Add new features */
			List<PointDouble> toAdd = featureUpdate.getNewPoints();
			for (PointDouble featpos : toAdd)
				ekf.addFeature(featpos.getX(), featpos.getY());
		} else {
			featureUpdateNullCount++;
			// if (featureUpdateNullCount == 3)
			// ekf.deleteAllFeatures();
		}
	}

	public PointDouble getDeviceCoords() {
		return ekf.getDeviceCoords();
	}

	public double getCurrentHeading() {
		return ekf.getHeadingDegrees();
	}

	public double getTotalDistanceTraveled() {
		double distance = 0;

		for (int i = 1; i < coordinates.size(); i++) {
			distance += coordinates.get(i - 1).computeDistanceTo(coordinates.get(i));
		}

		return distance;
	}

	public int getCurrentFeatureCount() {
		return ekf.getFeatureCount();
	}

	@Override
	public PointDouble getFeaturePos(int index) {
		return ekf.getFeatureCoordsFromStateVector(index);
	}
}
