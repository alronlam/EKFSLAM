package pathcomparison;

import java.util.ArrayList;

import commondata.PointDouble;

public class PathManager {

	private ArrayList<PointDouble> points;
	private ArrayList<PointDouble> correctPoints;

	public PathManager(String datasetName) {
		correctPoints = PathGenerator.generate(datasetName);
		points = new ArrayList<PointDouble>();
	}

	public void addPoint(PointDouble p) {
		points.add(p);
	}

	public double getAccuracy() {
		return PathComparator.compare(points, correctPoints);
	}

}
