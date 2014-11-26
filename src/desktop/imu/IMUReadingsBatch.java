package desktop.imu;

import java.util.ArrayList;

import stepbasedins.data.SensorEntry;

public class IMUReadingsBatch {

	private ArrayList<SensorEntry> entries;

	public IMUReadingsBatch() {
		entries = new ArrayList<SensorEntry>();
	}

	public void addEntry(SensorEntry se) {
		entries.add(se);
	}

	public void setEntries(ArrayList<SensorEntry> entries) {
		this.entries = entries;
	}

	@SuppressWarnings("unchecked")
	public ArrayList<SensorEntry> getEntries() {
		return (ArrayList<SensorEntry>) entries.clone();
	}
}
