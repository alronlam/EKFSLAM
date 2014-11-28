package vins;

import java.util.ArrayList;
import java.util.List;

import stepbasedins.data.SensorEntry;
import vins.motionestimation.IntegrateMotionEstimation;
import vins.motionestimation.MotionEstimation;

import commondata.DevicePose;
import commondata.PointDouble;

import desktop.imu.IMUReadingsBatch;
import dummies.ekf.EKF;
import dummies.features.FeatureUpdate;

public class DoubleIntegrationController {

	private EKF ekf;
	private MotionEstimation motionEstimator;

	private List<PointDouble> coordinates;

	public DoubleIntegrationController() {
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

	}

	public PointDouble getDeviceCoords() {
		return ekf.getDeviceCoords();
	}

	public double getTotalDistanceTraveled() {
		double distance = 0;

		for (int i = 1; i < coordinates.size(); i++) {
			distance += coordinates.get(i - 1).computeDistanceTo(coordinates.get(i));
		}

		return distance;
	}

}
