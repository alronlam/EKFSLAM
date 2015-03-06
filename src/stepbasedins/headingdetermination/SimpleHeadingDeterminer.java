package stepbasedins.headingdetermination;

import java.util.ArrayList;

import stepbasedins.data.SensorEntry;

public class SimpleHeadingDeterminer implements HeadingDeterminer {

	@Override
	public double getHeading(ArrayList<SensorEntry> batch) {
		double avg = HeadingDeterminerUtil.calculateAvgSensorEntry(batch);
		return avg;
	}

}
