package imu;

import java.util.ArrayList;
import java.util.List;

import stepbasedins.data.SensorEntry;

public class IMUReadingsBatch {

	private List<SensorEntry> entries;

	public IMUReadingsBatch() {
		entries = new ArrayList<SensorEntry>();
	}

	public void addEntry(SensorEntry se) {
		entries.add(se);
	}

}
