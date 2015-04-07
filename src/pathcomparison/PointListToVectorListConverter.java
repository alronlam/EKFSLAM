package pathcomparison;

import java.util.ArrayList;

import commondata.PointDouble;

public class PointListToVectorListConverter {

	public static ArrayList<Vector> convertPointList(ArrayList<PointDouble> pointList) {

		ArrayList<Vector> vectorList = new ArrayList<Vector>();
		int limit = pointList.size();

		for (int i = 2; i < limit;) {
			PointDouble prevPoint = pointList.get(i - 2);
			PointDouble currPoint = pointList.get(i);

			Vector currVector = prevPoint.calculateVectorTo(currPoint);
			vectorList.add(currVector);

			i += 2;
			if (i >= limit && i < limit + 1)
				i = limit - 1;
		}

		return vectorList;

	}

}
