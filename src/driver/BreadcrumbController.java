package driver;

import imu.IMUReadingsBatch;
import stepbasedins.controller.StepBasedINSController;
import stepbasedins.data.BatchProcessingResults;
import ekf.EKF;
import ekf.PointDouble;
import features.FeatureUpdate;

public class BreadcrumbController {

	private EKF ekf;
	private StepBasedINSController ins;

	public BreadcrumbController() {
		this.ekf = new EKF();
		this.ins = new StepBasedINSController();
	}

	public PointDouble getDeviceCoords() {
		return ekf.getDeviceCoords();
	}

	public void predict(IMUReadingsBatch imuBatch) {
		BatchProcessingResults result = this.ins.processSensorEntryBatch(imuBatch.getEntries());

		ekf.predictFromINS(result.getStrideLength(), Math.toRadians(result.getHeadingAngle()));
	}

	public void update(FeatureUpdate featureUpdate) {

	}

}
