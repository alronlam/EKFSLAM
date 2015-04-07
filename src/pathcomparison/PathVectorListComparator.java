package pathcomparison;

import java.util.ArrayList;

public class PathVectorListComparator {

	public static ArrayList<Double> getDistanceErrorList(ArrayList<Vector> actualList, ArrayList<Vector> correctList) {

		ArrayList<Double> distanceErrorList = new ArrayList<Double>();

		int limit = Math.min(actualList.size(), correctList.size());
		for (int i = 0; i < limit; i++) {
			distanceErrorList.add(actualList.get(i).getMagnitude() - correctList.get(i).getMagnitude());
		}

		return distanceErrorList;
	}

	public static ArrayList<Double> getHeadingErrorList(ArrayList<Vector> actualList, ArrayList<Vector> correctList) {

		ArrayList<Double> headingErrorList = new ArrayList<Double>();

		int limit = Math.min(actualList.size(), correctList.size());

		for (int i = 0; i < limit; i++) {

			double diff = actualList.get(i).getDirection() - correctList.get(i).getDirection();
			diff %= Math.PI;
			diff = Math.toDegrees(diff);

			headingErrorList.add(diff);
		}

		return headingErrorList;
	}
}
