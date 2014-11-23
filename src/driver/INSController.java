package driver;

import desktop.imu.IMUReadingsBatch;
import dummies.ekf.EKF;
import dummies.ekf.PointDouble;
import stepbasedins.controller.StepBasedINSController;
import stepbasedins.data.BatchProcessingResults;

public class INSController {

	private EKF ekf;
	private StepBasedINSController ins;

	public INSController() {
		this.ekf = new EKF();
		this.ins = new StepBasedINSController();
	}

	public void predict(IMUReadingsBatch imuBatch) {
		BatchProcessingResults result = this.ins.processSensorEntryBatch(imuBatch.getEntries());

		ekf.predictFromINS(result.getStrideLength(), Math.toRadians(result.getHeadingAngle()));
	}

	public PointDouble getDeviceCoords() {
		return ekf.getDeviceCoords();
	}

}
