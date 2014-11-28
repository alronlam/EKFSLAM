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

	public IMUReadingsBatch getCopy() {
		IMUReadingsBatch copy = new IMUReadingsBatch();

		ArrayList<SensorEntry> entriesCopy = new ArrayList<SensorEntry>();
		for (SensorEntry se : this.entries) {
			entriesCopy.add(se.getCopy());
		}

		copy.setEntries(entriesCopy);

		return copy;
	}
}
