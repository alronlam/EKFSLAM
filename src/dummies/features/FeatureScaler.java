package dummies.features;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import commondata.PointDouble;

public class FeatureScaler {

	private static FeatureUpdate previousUpdate;
	private static List<FeatureData> featureList;
	private static boolean canReturnUpdate;
	private static int scaledCurrEndIndex, scaledNewEndIndex, unscaledEndIndex;

	public static FeatureScaler featureScaler = new FeatureScaler();

	private FeatureScaler() {
		previousUpdate = null;
		featureList = new ArrayList<>();
		canReturnUpdate = false;
		scaledCurrEndIndex = 0;
		scaledNewEndIndex = 0;
		unscaledEndIndex = 0;
	}

	public static FeatureScaler getFeatureScaler() {
		return featureScaler;
	}

	// Queue:
	// | Scaled Curr | Scaled New | Unscaled |
	// unscaledEndIndex == size - 1

	// Bad Indices = All Bad Indices < scaledNewEndIndex
	// Current Points = All Feature Points < scaledCurrEndIndex
	// New Points = scaledCurrEndIndex < All Feature Points < scaledNewEndIndex

	public FeatureUpdate getScaledFeatureUpdate(FeatureUpdate relativeFeatureUpdate, PointDouble cameraPosition) {

		System.out.println("\ngetScaledFeatureUpdate called: " + relativeFeatureUpdate);
		if (relativeFeatureUpdate == null) {
			// System.out.println("Null Feature Update");
			// previousUpdate = null;
			//
			// featureList.clear();
			// scaledCurrEndIndex = 0;
			// scaledNewEndIndex = 0;
			// unscaledEndIndex = 0;
			//
			// canReturnUpdate = false;

			return null;
		} else if (previousUpdate == null) {
			System.out.println("\nInitial Delay");
			previousUpdate = relativeFeatureUpdate;

			// temp
			FeatureData temp = null;

			for (PointDouble relativePosition : relativeFeatureUpdate.getNewPoints()) {
				temp = new FeatureData(relativePosition);
				featureList.add(temp);
				unscaledEndIndex++;
			}

			FeatureData.addToCameraPositionList(cameraPosition);

			canReturnUpdate = true;

			System.out.println(featureList.size() + " " + temp.relativePositionList.size() + " " + temp.cameraPositionList.size());
			System.out.println("0 -> " + this.scaledCurrEndIndex + " -> " + this.scaledNewEndIndex + " -> " + this.unscaledEndIndex);
			System.out.println("End Initial Delay");
			return null;
		} else {
			FeatureUpdate scaledFeatureUpdate = new FeatureUpdate();

			// Bad Scaled Points Indices

			List<Integer> badScaledPointsIndices = new ArrayList<>();

			List<Integer> toDelete = relativeFeatureUpdate.getBadPointsIndex();
			Collections.reverse(toDelete);
			for (Integer index : toDelete) {
				if (index >= scaledNewEndIndex) {
					// removing unscaled feature
					this.unscaledEndIndex--;

					featureList.remove((int) index);
				} else {
					// removing scaled feature

					this.unscaledEndIndex--;
					scaledNewEndIndex--;
					if (index < scaledCurrEndIndex)
						scaledCurrEndIndex--;

					badScaledPointsIndices.add(0, index);
					featureList.remove((int) index);
				}
			}

			// System.out.println("\nBad Scaled Points:\n" + featureList.size() + " " + toDelete.size());
			// System.out.println("\nBad Scaled Points:\n" + featureList.size() + " " + toDelete);
			// System.out.println(badScaledPointsIndices.size());
			// System.out.println("0 -> " + this.scaledCurrEndIndex + " -> " + this.scaledNewEndIndex + " -> " + this.unscaledEndIndex);

			// Current Points & New Points

			List<PointDouble> currentPoints = new ArrayList<>();
			List<PointDouble> newPoints = new ArrayList<>();
			List<PointDouble> relativeFeatures = relativeFeatureUpdate.getCurrentPoints();

			scaledCurrEndIndex = scaledNewEndIndex;
			scaledNewEndIndex = featureList.size();
			
			for (int i = 0; i < relativeFeatures.size(); ++i) {
				if (i < scaledCurrEndIndex) {
					currentPoints.add(featureList.get(i).getEstimatedPosition(relativeFeatures.get(i), cameraPosition));
				} else {
					newPoints.add(featureList.get(i).getEstimatedPosition(relativeFeatures.get(i), cameraPosition));
				}

			}

			FeatureData.addToCameraPositionList(cameraPosition);
			System.out.println("Camera Positions: " + FeatureData.cameraPositionList.size());
			System.out.println("Current Camera Position: " + cameraPosition);
			if (featureList.size() > 0) {
				System.out.println("Metric Position of First, MidPoint, and Last Feature: ");
				for (int i = 0; i < Math.min(5, featureList.size()); ++i)
					System.out.println("feature #" + featureList.get(i) + ": " + featureList.get(i).getSavedEstimatedPosition());
				// System.out.println("feature #" + featureList.get(0) + ": " + featureList.get(0).getSavedEstimatedPosition());
				// System.out.println("feature #" + featureList.get((int) (featureList.size() / 2)) + ": "
				// + featureList.get((int) (featureList.size() / 2)).getSavedEstimatedPosition());
				// System.out.println("feature #" + featureList.get(featureList.size() - 1) + ": " + featureList.get(featureList.size() - 1).getSavedEstimatedPosition());
			}

			// System.out.println("\n" + featureList.size());
			// System.out.println(currentPoints.size());
			// System.out.println(newPoints.size());
			// System.out.println("0 -> " + this.scaledCurrEndIndex + " -> " + this.scaledNewEndIndex + " -> " + this.unscaledEndIndex);

			// Unscaled Points

			// temp
			FeatureData temp = null;

			for (PointDouble relativePosition : relativeFeatureUpdate.getNewPoints()) {
				temp = new FeatureData(relativePosition);
				featureList.add(temp);
				this.unscaledEndIndex++;
			}

			scaledFeatureUpdate.setBadPointsIndex(badScaledPointsIndices);
			scaledFeatureUpdate.setCurrentPoints(currentPoints);
			scaledFeatureUpdate.setNewPoints(newPoints);

			System.out.println("\nScaled Feature Update");
			System.out.println("0 -> " + this.scaledCurrEndIndex + " -> " + this.scaledNewEndIndex + " -> " + this.unscaledEndIndex);
			System.out.println(scaledFeatureUpdate.getCurrentPoints().size() + scaledFeatureUpdate.getBadPointsIndex().size());
			System.out.println(scaledFeatureUpdate.getCurrentPoints().size() + scaledFeatureUpdate.getNewPoints().size());
			// System.out.println(scaledFeatureUpdate);
			// System.out.println(badScaledPointsIndices);
			// System.out.println(featureList.size());
			// System.out.println(featureList);

			return scaledFeatureUpdate;
		}
	}
}
