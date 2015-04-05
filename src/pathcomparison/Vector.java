package pathcomparison;

public class Vector {

	private double magnitude;
	private double direction; // radians

	public Vector(double magnitude, double direction) {
		this.magnitude = magnitude;
		this.direction = direction;
	}

	public double getMagnitude() {
		return magnitude;
	}

	public void setMagnitude(double magnitude) {
		this.magnitude = magnitude;
	}

	public double getDirection() {
		return direction;
	}

	public void setDirection(double direction) {
		this.direction = direction;
	}

}
