package dummies.features;

import java.util.ArrayList;
import java.util.List;

import commondata.PointDouble;

public class FeatureData {

	// temporarily all public

	// these two are paired with each other
	public int index;
	public List<PointDouble> relativePositionList;
	public static List<PointDouble> cameraPositionList = new ArrayList<>();

	public List<PointDouble> metricPositionList;
	public PointDouble estimatedPosition;

	public static int run = 0;

	public FeatureData() {
		relativePositionList = new ArrayList<>();
		metricPositionList = new ArrayList<>();
		index = run++;
	}

	public static void resetCameraPositions() {
		cameraPositionList.clear();
	}

	public String toString() {
		return "" + index;
	}

	public FeatureData(PointDouble relativePosition) {
		this();
		relativePositionList.add(relativePosition);
	}

	// public void addToRelativePositionList(PointDouble relativePosition) {
	// relativePositionList.add(relativePosition);
	// }
	//
	// public void removeFromLists(int i) {
	// relativePositionList.remove(i);
	// metricPositionList.remove(i);
	// }

	public static void addToCameraPositionList(PointDouble cameraPosition) {
		cameraPositionList.add(cameraPosition);
	}

	public PointDouble getEstimatedPosition(PointDouble relativePosition, PointDouble cameraPosition, PointDouble predictedPosition) {
		if (relativePositionList.size() < 1) {
			relativePositionList.add(relativePosition);
			return null;
		}
		if (relativePositionList.size() == 1) {

			// Create metric position based on new relative position and camera
			// position
			// given two rays, find the intersection
			// solve for the lines
			// y = mx + b

			// Given:

			PointDouble P0, P1; // camera positions
			PointDouble F0, F1; // feature points w/ relative scale

			P0 = cameraPosition;
			F0 = relativePosition;

			// translate relative position to fit global coordinate system
			// F0.setX(P0.getX() + F0.getX());
			// F0.setY(P0.getY() + F0.getY());

			int index = relativePositionList.size() - 1;

			P1 = relativePositionList.get(index);
			F1 = cameraPositionList.get(index);

			// translate relative position to fit global coordinate system
			// F1.setX(P1.getX() + F1.getX());
			// F1.setY(P1.getY() + F1.getY());

			// Get Slope and y-intercept

			double m0 = (P0.getY() - F0.getY()) / (P0.getX() - F0.getX());
			double m1 = (P1.getY() - F1.getY()) / (P1.getX() - F1.getX());

			double b0 = P0.getY() - m0 * P0.getX();
			double b1 = P1.getY() - m1 * P1.getX();

			// Get the intersection of the two lines

			double x = (b1 - b0) / (m0 - m1);
			double y = (b0 * m1 - b1 * m0) / (m1 - m0);

			PointDouble metricPosition = new PointDouble(x, y);

			// Add new metric position to list
//			this.metricPositionList.add(metricPosition);

			// for (int i = 0; i < relativePositionList.size(); ++i) {
			// PointDouble P0, P1; // camera positions
			// PointDouble F0, F1; // feature points w/ relative scale
			//
			// P0 = cameraPosition;
			// F0 = relativePosition;
			//
			// // translate relative position to fit global coordinate system
			// F0.setX(P0.getX() + F0.getX());
			// F0.setY(P0.getY() + F0.getY());
			//
			// P1 = relativePositionList.get(i);
			// F1 = cameraPositionList.get(i);
			//
			// // translate relative position to fit global coordinate system
			// F1.setX(P1.getX() + F1.getX());
			// F1.setY(P1.getY() + F1.getY());
			//
			// // Get Slope and y-intercept
			//
			// double m0 = (P0.getY() - F0.getY()) / (P0.getX() - F0.getX());
			// double m1 = (P1.getY() - F1.getY()) / (P1.getX() - F1.getX());
			//
			// double b0 = P0.getY() - m0 * P0.getX();
			// double b1 = P1.getY() - m1 * P1.getX();
			//
			// // Get the intersection of the two lines
			//
			// double x = (b1 - b0) / (m0 - m1);
			// double y = (b0 * m1 - b1 * m0) / (m1 - m0);
			//
			// PointDouble metricPosition = new PointDouble(x, y);
			//
			// // Add new metric position to list
			// this.metricPositionList.add(metricPosition);
			// }
			//
			// if (this.index == 0)
			// System.out.println(metricPositionList);
//
//			updateEstimatedPosition();
//
//			// Add relative position to list
//			this.relativePositionList.add(relativePosition);

			return metricPosition;
		} else if (relativePositionList.size() > 1) {
			// Create metric position based on new relative position and known predicted feature position
			// given feature position which forms a line orthogonal to the new relative position's ray
			// solve for the intersection
			// solve for the lines
			// y = mx + b

			// Given:

			PointDouble P0, P1; // camera positions
			PointDouble F0, F1; // feature points w/ relative scale

			P0 = cameraPosition;
			F0 = relativePosition;

			// translate relative position to fit global coordinate system
			// F0.setX(P0.getX() + F0.getX());
			// F0.setY(P0.getY() + F0.getY());

			int index = relativePositionList.size() - 1;

			F1 = cameraPositionList.get(index);

			// translate relative position to fit global coordinate system
			// F1.setX(P1.getX() + F1.getX());
			// F1.setY(P1.getY() + F1.getY());

			// Get Slope and y-intercept

			double m0 = (P0.getY() - F0.getY()) / (P0.getX() - F0.getX());
			double b0 = P0.getY() - m0 * P0.getX();

			// Get perpendicular line via y - y0 = m * (x - x0)
			double m1 = -(P0.getX() - F0.getX()) / (P0.getY() - F0.getY());
			double b1 = -m1 * F1.getX() + F1.getY();

			// Get the intersection of the two lines

			double x = (b1 - b0) / (m0 - m1);
			double y = (b0 * m1 - b1 * m0) / (m1 - m0);

			PointDouble metricPosition = new PointDouble(x, y);

			// Add new metric position to list
//			this.metricPositionList.add(metricPosition);

//			updateEstimatedPosition();

			// Add relative position to list
//			this.relativePositionList.add(relativePosition);

			return metricPosition;
		}
		return null;
	}

//	private void updateEstimatedPosition() {
//
//		double sumnumx = 0;
//		double sumnumy = 0;
//		double sumdenx = 0;
//		double sumdeny = 0;
//
//		// System.out.println(metricPositionList.size());
//		if (metricPositionList.size() == 1)
//			this.estimatedPosition = metricPositionList.get(0);
//		else {
//			// Mean
//			// double sumx = 0, sumy = 0, count = metricPositionList.size();
//			// for (PointDouble featurePosition : this.metricPositionList) {
//			// sumx += featurePosition.getX();
//			// sumy += featurePosition.getY();
//			// }
//			//
//			// this.estimatedPosition = new PointDouble(sumx / count, sumy /
//			// count);
//
//			// Weighted Mean
//			for (int i = 0; i < metricPositionList.size(); ++i) {
//				PointDouble curr = metricPositionList.get(i);
//
//				double weightx = 0;
//				double weighty = 0;
//				for (int j = 0; j < metricPositionList.size(); ++j) {
//					weightx += Math.abs(curr.getX() - metricPositionList.get(j).getX());
//					weighty += Math.abs(curr.getY() - metricPositionList.get(j).getY());
//				}
//				// weightx *= weightx;
//				// weighty *= weighty;
//				// System.out.print("(" + weightx + ", " + weighty + ") ");
//
//				sumnumx += weightx * curr.getX();
//				sumnumy += weighty * curr.getY();
//
//				sumdenx += weightx;
//				sumdeny += weighty;
//			}
//			// System.out.println();
//			// System.out.println();
//
//			this.estimatedPosition = new PointDouble(sumnumx / sumdenx, sumnumy / sumdeny);
//		}
//	}

	public PointDouble getSavedEstimatedPosition() {
		return estimatedPosition;
	}
}
