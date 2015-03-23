package pathcomparison;

import java.util.ArrayList;

import commondata.PointDouble;

public class PathManager {

	private ArrayList<PointDouble> points;
	private String datasetName;

	public PathManager(String datasetName) {
		this.datasetName = datasetName;
		points = new ArrayList<PointDouble>();
	}

	public void addPoint(PointDouble p) {
		points.add(p);
	}

	public double getAccuracy() {

		ArrayList<PointDouble> correctPoints = PathGenerator.generate(datasetName, points.size());
		return PathComparator.compare(points, correctPoints);
	}

}
