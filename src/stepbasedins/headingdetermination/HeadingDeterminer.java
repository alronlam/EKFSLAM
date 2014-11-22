package stepbasedins.headingdetermination;

import java.util.ArrayList;

import stepbasedins.data.SensorEntry;

public interface HeadingDeterminer {
	public double getHeading(ArrayList<SensorEntry> batch);
}
