package dummies.ekf;

import java.util.ArrayList;
import java.util.List;

import commondata.PointDouble;

public class EKFScalingCorrecter {

	private List<PointDouble> cameraPositionList;
	private List<PointDouble> correctedCameraPositionList;
	private PointDouble offset;

	public static EKFScalingCorrecter ekfScalingCorrecter = new EKFScalingCorrecter();

	private EKFScalingCorrecter() {
		cameraPositionList = new ArrayList<>();
		correctedCameraPositionList = new ArrayList<>();
		offset = new PointDouble(0, 0);
	}

	public static void resetInstance() {
		ekfScalingCorrecter = new EKFScalingCorrecter();
	}

	public static EKFScalingCorrecter getEKFScalingResultCorrecter() {
		return ekfScalingCorrecter;
	}

	public void updateCoords(PointDouble deviceCoords, PointDouble predictResult) {

		if (correctedCameraPositionList.size() < 2) {
			correctedCameraPositionList.add(deviceCoords);
			return;
		}

		deviceCoords.setX(deviceCoords.getX() - offset.getX());
		deviceCoords.setY(deviceCoords.getY() - offset.getY());
		predictResult.setX(predictResult.getX() - offset.getX());
		predictResult.setY(predictResult.getY() - offset.getY());

		double dist = correctedCameraPositionList.get(correctedCameraPositionList.size() - 1).computeDistanceTo(deviceCoords);

		if ((dist < .7) || (dist > -0.0000001 && dist < 0.0000001)) {
			// valid point

			correctedCameraPositionList.add(deviceCoords);
		} else {
			// need to correct

			offset.setX(offset.getX() + deviceCoords.getX() - predictResult.getX());
			offset.setY(offset.getY() + deviceCoords.getY() - predictResult.getY());

			// predictResult.setX(predictResult.getX()+1000);
			correctedCameraPositionList.add(predictResult);
		}
	}

	public String getCorrectedPositionsAsString() {
		StringBuilder sb = new StringBuilder();
		for (PointDouble cameraPosition : correctedCameraPositionList) {
			sb.append(cameraPosition);
			sb.append("\n");
		}
		return sb.toString();
	}

	public PointDouble getFinalPosition() {
		int finalPositionIndex = correctedCameraPositionList.size() - 1;
		PointDouble finalPosition;
		if (correctedCameraPositionList.size() > 0) {
			finalPosition = correctedCameraPositionList.get(finalPositionIndex);
			return finalPosition;
		} else
			return new PointDouble(0, 0);
	}

	public double getTotalDistanceTraveled() {
		double totalDistanceTraveled;
		totalDistanceTraveled = 0;

		totalDistanceTraveled += correctedCameraPositionList.get(0).computeDistanceTo(new PointDouble(0, 0));

		for (int i = 1; i < correctedCameraPositionList.size() - 1; ++i)
			totalDistanceTraveled += correctedCameraPositionList.get(i).computeDistanceTo(correctedCameraPositionList.get(i + 1));

		return totalDistanceTraveled;
	}

}
