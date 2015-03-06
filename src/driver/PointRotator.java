package driver;

import java.util.ArrayList;

import org.opencv.core.Point;

public class PointRotator {

	public static void main(String[] args) {

		double angle = Math.toRadians(30);
		ArrayList<Point> inputPoints = initInputPoints();
		ArrayList<Point> rotatedPoints = rotatePoints(inputPoints, angle);
		String result = generateOutputString(rotatedPoints);
		System.out.println(result);
	}

	private static ArrayList<Point> initInputPoints() {
		ArrayList<Point> points = new ArrayList<Point>();
		points.add(new Point(0, 0));
		points.add(new Point(-28.715, 0));
		points.add(new Point(-28.715, -7));
		points.add(new Point(0, -7));
		return points;
	}

	private static ArrayList<Point> rotatePoints(ArrayList<Point> inputPoints, double angle) {
		ArrayList<Point> rotatedPoints = new ArrayList<Point>();
		for (Point point : inputPoints) {
			double newX = point.x * Math.cos(angle) - point.y * Math.sin(angle);
			double newY = point.x * Math.sin(angle) + point.y * Math.cos(angle);
			Point rotatedPoint = new Point(newX, newY);
			rotatedPoints.add(rotatedPoint);
		}
		return rotatedPoints;
	}

	private static String generateOutputString(ArrayList<Point> rotatedPoints) {
		StringBuilder x_sb = new StringBuilder();
		StringBuilder y_sb = new StringBuilder();

		x_sb.append("miguel4_x = [");
		y_sb.append("miguel4_y = [");

		for (int i = 0; i < rotatedPoints.size(); i++) {
			Point point = rotatedPoints.get(i);
			if (i > 0) {
				x_sb.append(", ");
				y_sb.append(", ");
			}
			x_sb.append(point.x);
			y_sb.append(point.y);
		}

		if (rotatedPoints.size() > 0) {
			x_sb.append(", " + rotatedPoints.get(0).x);
			y_sb.append(", " + rotatedPoints.get(0).y);
		}

		x_sb.append("];");
		y_sb.append("];");

		return x_sb.toString() + "\r\n" + y_sb.toString();

	}
}
