package stepbasedins;

import stepbasedins.controller.StepBasedINSController;
import stepbasedins.data.BatchProcessingResults;

import commondata.PointDouble;

import desktop.imu.IMUReadingsBatch;
import dummies.ekf.EKF;

public class INSController {

	private EKF ekf;
	private StepBasedINSController ins;

	public int totalStepsDetected;

	public INSController() {
		this.ekf = new EKF();
		this.ins = new StepBasedINSController();
	}

	public void predict(IMUReadingsBatch imuBatch) {
		BatchProcessingResults result = this.ins.processSensorEntryBatch(imuBatch.getEntries());
		totalStepsDetected += result.getDetectedSteps();

		// System.out.println(result.getStrideLength() + " " +
		// result.getHeadingAngle() + "deg");
		ekf.predictFromINS(result.getStrideLength(), Math.toRadians(result.getHeadingAngle()));
	}

	public PointDouble getDeviceCoords() {
		return ekf.getDeviceCoords();
	}

}
