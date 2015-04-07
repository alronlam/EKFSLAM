package pathcomparison;

import java.util.ArrayList;

import commondata.Constants;
import commondata.PointDouble;

public class PathGenerator {

	// method stub
	public static ArrayList<PointDouble> generate(String datasetName, int timeSteps) {

		if (Constants.SJ_PARTIAL_DATASETS.contains(datasetName)) {

			if (datasetName.contains("2 Pass")) {
				ArrayList<PointDouble> path = new ArrayList<PointDouble>();
				path.addAll(generateLinePath(new PointDouble(0, 0), new PointDouble(-32.626379824325326,
						73.28004888480123), timeSteps / 2));
				if (timeSteps % 2 == 0)
					path.addAll(generateLinePath(new PointDouble(-32.626379824325326, 73.28004888480123),
							new PointDouble(0, 0), timeSteps / 2));
				else
					path.addAll(generateLinePath(new PointDouble(-32.626379824325326, 73.28004888480123),
							new PointDouble(0, 0), timeSteps / 2 + 1));

				return path;

			} else {
				PointDouble start = new PointDouble(0, 0);
				PointDouble end = new PointDouble(-32.626379824325326, 73.28004888480123);
				return generateLinePath(start, end, timeSteps);
			}

		}
		if (Constants.SJ_DATASETS.contains(datasetName)) {
			if (datasetName.contains("2 Pass")) {
				ArrayList<PointDouble> path = new ArrayList<PointDouble>();
				path.addAll(generateLinePath(new PointDouble(0, 0), new PointDouble(-40.317769744888714,
						90.55519348882281), timeSteps / 2));
				if (timeSteps % 2 == 0)
					path.addAll(generateLinePath(new PointDouble(-40.317769744888714, 90.55519348882281),
							new PointDouble(0, 0), timeSteps / 2));
				else
					path.addAll(generateLinePath(new PointDouble(-40.317769744888714, 90.55519348882281),
							new PointDouble(0, 0), timeSteps / 2 + 1));

				return path;

			} else {
				PointDouble start = new PointDouble(0, 0);
				PointDouble end = new PointDouble(-40.317769744888714, 90.55519348882281);
				return generateLinePath(start, end, timeSteps);
			}
		}
		if (Constants.LS_DATASETS.contains(datasetName)) {
			if (datasetName.contains("2 Pass")) {
				ArrayList<PointDouble> path = new ArrayList<PointDouble>();
				path.addAll(generateLinePath(new PointDouble(0, 0), new PointDouble(45.7619397124583,
						-102.78299943936902), timeSteps / 2));
				if (timeSteps % 2 == 0)
					path.addAll(generateLinePath(new PointDouble(45.7619397124583, -102.78299943936902),
							new PointDouble(0, 0), timeSteps / 2));
				else
					path.addAll(generateLinePath(new PointDouble(45.7619397124583, -102.78299943936902),
							new PointDouble(0, 0), timeSteps / 2 + 1));

				return path;

			} else {
				PointDouble start = new PointDouble(0, 0);
				PointDouble end = new PointDouble(45.7619397124583, -102.78299943936902);
				return generateLinePath(start, end, timeSteps);
			}
		}
		if (Constants.MIGUEL_DATASETS.contains(datasetName)) {
			PointDouble v1 = new PointDouble(0, 0);
			PointDouble v2 = new PointDouble(-24.867919469670156, -14.357499999999998);
			PointDouble v3 = new PointDouble(-21.367919469670156, -20.419677826491068);
			PointDouble v4 = new PointDouble(3.4999999999999996, -6.062177826491071);

			if (datasetName.contains("2 Pass")) {

				ArrayList<PointDouble> path = new ArrayList<PointDouble>();

				path.addAll(generateRectangularPath(v1, v2, v3, v4, timeSteps / 2));
				if (timeSteps % 2 == 0)
					path.addAll(generateRectangularPath(v1, v2, v3, v4, timeSteps / 2));
				else
					path.addAll(generateRectangularPath(v1, v2, v3, v4, timeSteps / 2 + 1));

				return path;
			} else {

				return generateRectangularPath(v1, v2, v3, v4, timeSteps);
			}
		}

		return new ArrayList<PointDouble>();
	}

	public static ArrayList<PointDouble> generateLinePath(PointDouble start, PointDouble end, int timeSteps) {

		ArrayList<PointDouble> path = new ArrayList<PointDouble>();

		double incrementX = (end.getX() - start.getX()) / timeSteps;
		double incrementY = (end.getY() - start.getY()) / timeSteps;

		for (int i = 0; i < timeSteps; i++) {
			double newX = start.getX() + incrementX * i;
			double newY = start.getY() + incrementY * i;
			path.add(new PointDouble(newX, newY));

			if (i == timeSteps - 1)
				if (path.get(i).computeDistanceTo(end) != 0) {
					path.remove(i);
					path.add(end.getCopy());
				}
		}

		return path;

	}

	public static ArrayList<PointDouble> generateRectangularPath(PointDouble v1, PointDouble v2, PointDouble v3,
			PointDouble v4, int timeSteps) {

		ArrayList<PointDouble> path = new ArrayList<PointDouble>();

		int[] timeStepsForEachLeg = new int[4];
		for (int i = 0; i < timeStepsForEachLeg.length; i++) {
			timeStepsForEachLeg[i] = timeSteps / 4;
		}

		for (int remainder = timeSteps % 4, i = 0; remainder > 0; remainder--, i++) {
			timeStepsForEachLeg[i]++;
		}

		path.addAll(generateLinePath(v1, v2, timeStepsForEachLeg[0]));
		path.addAll(generateLinePath(v2, v3, timeStepsForEachLeg[1]));
		path.addAll(generateLinePath(v3, v4, timeStepsForEachLeg[2]));
		path.addAll(generateLinePath(v4, v1, timeStepsForEachLeg[3]));

		return path;
	}

}
