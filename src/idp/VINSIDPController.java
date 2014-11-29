package idp;

import idp.ekf.Camera;
import idp.ekf.EKF;
import idp.ekf.PointTriple;
import idp.features.FeatureUpdate;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import commondata.Constants;
import commondata.PointDouble;

import desktop.imu.IMUReadingsBatch;

public class VINSIDPController {

	private EKF ekf;
	private Random random;
	private Camera camera;
	private int featureUpdateNullCount = 0;

	private List<PointDouble> coordinates;

	public VINSIDPController() {
		this.ekf = new EKF();
		random = new Random();
		camera = new Camera();

		this.coordinates = new ArrayList<PointDouble>();
	}

	public PointDouble getDeviceCoords() {
		return ekf.getDeviceCoords();
	}

	public void predict(IMUReadingsBatch imuBatch) {
		double vxP = random.nextGaussian();
		double vyP = random.nextGaussian();
		double vzP = random.nextGaussian();
		PointTriple vP = new PointTriple(vxP, vyP, vzP);

		double wxP = random.nextGaussian();
		double wyP = random.nextGaussian();
		double wzP = random.nextGaussian();
		PointTriple wP = new PointTriple(wxP, wyP, wzP);

		ekf.predict(vP, wP, Constants.MS_OVERALL_CYCLE_FREQUENCY / 1000.0);
		coordinates.add(ekf.getDeviceCoords());
	}

	public void update(FeatureUpdate featureUpdate) {

		// if (featureUpdate != null) {
		featureUpdateNullCount = 0;
		/* Delete features that disappeared */
		List<Integer> toDelete = featureUpdate.getBadPointsIndex();

		// System.out.println("To Delete:" + toDelete.size());
		Collections.reverse(toDelete);
		for (Integer index : toDelete)
			ekf.deleteFeature(index);

		/* Update using re-observed features */
		List<PointDouble> toUpdate = featureUpdate.getCurrentPoints();

		// System.out.println("To Update:" + toUpdate.size());
		for (int i = 0; i < toUpdate.size(); i++) {
			PointDouble currXY = toUpdate.get(i);
			ekf.updateFromReobservedFeatureThroughImageCoords(i, currXY.getX(), currXY.getY());
		}

		/* Add new features */
		List<PointDouble> toAdd = featureUpdate.getNewPoints();

		// System.out.println("To Add:" + toAdd.size());
		for (PointDouble featpos : toAdd)
			ekf.addFeature((int) featpos.getX(), (int) featpos.getY(), camera);

		coordinates.set(coordinates.size() - 1, ekf.getDeviceCoords());
		// } else {
		// // featureUpdateNullCount++;
		// // if (featureUpdateNullCount == 3)
		// // ekf.deleteAllFeatures();
		// }
	}

	public double getTotalDistanceTraveled() {
		double distance = 0;

		for (int i = 1; i < coordinates.size(); i++) {
			distance += coordinates.get(i - 1).computeDistanceTo(coordinates.get(i));
		}

		return distance;
	}

}
