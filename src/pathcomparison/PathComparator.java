package pathcomparison;

import java.util.ArrayList;

import org.opencv.core.Point;

public class PathComparator {

	public static double compare(ArrayList<Point> set1, ArrayList<Point> set2) {
		double totalDistance = 0;

		int limit = Math.min(set1.size(), set2.size());

		for (int i = 0; i < limit; i++) {
			Point p1 = set1.get(i);
			Point p2 = set2.get(i);
			double currDistance = calculateDistance(p1, p2);
			totalDistance += currDistance;
		}

		return totalDistance / limit;

	}

	public static double calculateDistance(Point p1, Point p2) {
		return Math.sqrt(Math.pow(p1.x - p2.x, 2) + Math.pow(p1.y - p2.y, 2));
	}

}
