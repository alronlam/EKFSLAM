package pathcomparison;

import java.util.ArrayList;

import commondata.PointDouble;

public class PathComparator {

	public static double compare(ArrayList<PointDouble> set1, ArrayList<PointDouble> set2) {
		double totalDistance = 0;

		int limit = Math.min(set1.size(), set2.size());

		for (int i = 0; i < limit; i++) {
			PointDouble p1 = set1.get(i);
			PointDouble p2 = set2.get(i);
			double currDistance = p1.computeDistanceTo(p2);
			totalDistance += currDistance;
		}

		return totalDistance / limit;

	}

}
