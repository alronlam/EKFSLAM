package driver;

import imu.IMUReadingsBatch;
import ekf.EKF;
import ekf.PointDouble;

public class INSController {

	private EKF ekf;

	public void predict(IMUReadingsBatch imuBatch) {

	}

	public PointDouble getDeviceCoords() {
		return ekf.getDeviceCoords();
	}

}
