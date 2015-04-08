package pathcomparison;

import java.util.ArrayList;

import commondata.PointDouble;

public class PathComparator {

	public static double compare(ArrayList<PointDouble> set1, ArrayList<PointDouble> set2) {

		int limit = Math.min(set1.size(), set2.size());

		return getTotalDistanceError(set1, set2) / limit;
	}

	public static double getTotalDistanceError(ArrayList<PointDouble> set1, ArrayList<PointDouble> set2) {
		double totalDistance = 0;
		int limit = Math.min(set1.size(), set2.size());

		for (int i = 0; i < limit; i++) {
			PointDouble p1 = set1.get(i);
			PointDouble p2 = set2.get(i);
			double currDistance = p1.computeDistanceTo(p2);
			totalDistance += currDistance;

			System.out.println(currDistance);

		}

		return totalDistance;
	}

	public static ArrayList<Double> getErrorList(ArrayList<PointDouble> set1, ArrayList<PointDouble> set2) {
		ArrayList<Double> errorList = new ArrayList<Double>();

		int limit = Math.min(set1.size(), set2.size());

		for (int i = 0; i < limit; i++) {
			PointDouble p1 = set1.get(i);
			PointDouble p2 = set2.get(i);
			double currDistance = p1.computeDistanceTo(p2);
			errorList.add(currDistance);
		}

		return errorList;
	}
}
