package idp;

import commondata.PointDouble;

import idp.ekf.EKF;
import desktop.imu.IMUReadingsBatch;
import dummies.features.FeatureUpdate;

public class VINSIDPController {

	private EKF ekf;

	public VINSIDPController() {
		this.ekf = new EKF();

	}

	public PointDouble getDeviceCoords() {
		return ekf.getDeviceCoords();
	}

	public void predict(IMUReadingsBatch imuBatch) {
	}

	public void update(FeatureUpdate featureUpdate) {
	}

}
