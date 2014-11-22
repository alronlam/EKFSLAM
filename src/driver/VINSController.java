package driver;

import imu.IMUReadingsBatch;
import ekf.EKF;
import ekf.PointDouble;
import features.FeatureUpdate;

public class VINSController {

	private EKF ekf;

	public VINSController() {
		this.ekf = new EKF();
	}

	public void predict(IMUReadingsBatch batch) {

	}

	public void update(FeatureUpdate featureUpdate) {

	}

	public PointDouble getDeviceCoords() {
		return ekf.getDeviceCoords();
	}

}
