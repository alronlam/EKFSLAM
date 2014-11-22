package driver;

import imu.IMUReadingsBatch;

import java.util.ArrayList;

import stepbasedins.data.SensorEntry;
import vins.motionestimation.IntegrateMotionEstimation;
import vins.motionestimation.MotionEstimation;
import ekf.DevicePose;
import ekf.EKF;
import ekf.PointDouble;
import features.FeatureUpdate;

public class VINSController {

	private EKF ekf;
	private MotionEstimation motionEstimator;

	public VINSController() {
		this.ekf = new EKF();
		this.motionEstimator = new IntegrateMotionEstimation();
	}

	public void predict(IMUReadingsBatch batch) {
		ArrayList<SensorEntry> sensorEntries = batch.getEntries();
		for (SensorEntry se : sensorEntries)
			motionEstimator.inputData(se);
		try {
			DevicePose devicePose = motionEstimator.getHeadingAndDisplacement();

			ekf.predictFromINS(devicePose.getXYDistance(), devicePose.getHeadingRadians());

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

}
