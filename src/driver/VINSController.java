package driver;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import desktop.imu.IMUReadingsBatch;
import dummies.ekf.DevicePose;
import dummies.ekf.EKF;
import dummies.ekf.PointDouble;
import dummies.features.FeatureUpdate;
import stepbasedins.data.SensorEntry;
import vins.motionestimation.IntegrateMotionEstimation;
import vins.motionestimation.MotionEstimation;

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

		/* Add new features */
		List<PointDouble> toAdd = featureUpdate.getNewPoints();
		for (PointDouble featpos : toAdd)
			ekf.addFeature(featpos.getX(), featpos.getY());
	}

	public PointDouble getDeviceCoords() {
		return ekf.getDeviceCoords();
	}

}
