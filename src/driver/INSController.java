package driver;

import desktop.imu.IMUReadingsBatch;
import stepbasedins.controller.StepBasedINSController;
import stepbasedins.data.BatchProcessingResults;
import ekf.EKF;
import ekf.PointDouble;

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
