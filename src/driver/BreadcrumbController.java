package driver;

import imu.IMUReadingsBatch;

import java.util.Collections;
import java.util.List;

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

}
