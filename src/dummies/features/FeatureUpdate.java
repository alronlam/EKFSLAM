package dummies.features;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import commondata.PointDouble;

public class FeatureUpdate {
	private List<Integer> badPointsIndex;
	private List<PointDouble> currentPoints;
	private List<PointDouble> newPoints;

	public FeatureUpdate() {
		badPointsIndex = new ArrayList<>();
		currentPoints = new ArrayList<>();
		newPoints = new ArrayList<>();
	}

	/** selectively makes a copy of feature update **/
	public FeatureUpdate makeCopy(List<Integer> badPointsIndex, List<PointDouble> currentPoints, List<PointDouble> newPoints) {
		FeatureUpdate copy = new FeatureUpdate();

		// bad points index
		{
			List<Integer> temp = new ArrayList<>();

			if (badPointsIndex == null)
				Collections.copy(temp, this.badPointsIndex);
			else
				Collections.copy(temp, badPointsIndex);
			copy.setBadPointsIndex(temp);
		}

		// current points
		{
			List<PointDouble> temp = new ArrayList<>();

			if (badPointsIndex == null)
				Collections.copy(temp, this.currentPoints);
			else
				Collections.copy(temp, currentPoints);
			copy.setCurrentPoints(temp);
		}

		// new points
		{
			List<PointDouble> temp = new ArrayList<>();

			if (badPointsIndex == null)
				Collections.copy(temp, this.newPoints);
			else
				Collections.copy(temp, newPoints);
			copy.setNewPoints(temp);
		}

		return copy;
	}

	public List<Integer> getBadPointsIndex() {
		return badPointsIndex;
	}

	public List<PointDouble> getCurrentPoints() {
		return currentPoints;
	}

	public List<PointDouble> getNewPoints() {
		return newPoints;
	}

	void setBadPointsIndex(List<Integer> badPointsIndex) {
		this.badPointsIndex = badPointsIndex;
	}

	void setCurrentPoints(List<PointDouble> currentPoints) {
		this.currentPoints = currentPoints;
	}

	void setNewPoints(List<PointDouble> newPoints) {
		this.newPoints = newPoints;
	}

	public String toString() {
		System.out.println("Bad: " + badPointsIndex.size());
		System.out.println("Cur: " + currentPoints.size());
		System.out.println("New: " + newPoints.size());
		return "";
	}
}
