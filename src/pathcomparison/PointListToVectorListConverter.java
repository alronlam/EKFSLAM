package pathcomparison;

import java.util.ArrayList;

import commondata.PointDouble;

public class PointListToVectorListConverter {

	public static ArrayList<Vector> convertPointList(ArrayList<PointDouble> pointList) {

		ArrayList<Vector> vectorList = new ArrayList<Vector>();

		for (int i = 1; i < pointList.size(); i++) {
			PointDouble prevPoint = pointList.get(i - 1);
			PointDouble currPoint = pointList.get(i);

			Vector currVector = prevPoint.calculateVectorTo(currPoint);
			vectorList.add(currVector);
		}

		return vectorList;

	}

}
