package driver;

import imu.IMUReadingsBatch;
import ekf.EKF;
import ekf.PointDouble;
import features.FeatureUpdate;

public class BreadcrumbController {

	private EKF ekf;

	public BreadcrumbController() {
		this.ekf = new EKF();
	}

	public PointDouble getDeviceCoords() {
		return ekf.getDeviceCoords();
	}

	public void predict(IMUReadingsBatch batch) {
		
	}

	public void update(FeatureUpdate featureUpdate) {

	}

}
