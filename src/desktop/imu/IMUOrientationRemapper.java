package desktop.imu;

import java.util.List;

import stepbasedins.data.SensorEntry;

public class IMUOrientationRemapper {

	// method stub
	public List<SensorEntry> remap(List<SensorEntry> entries) {

		if (entries.size() == 0)
			return entries;

		double initialHeadingDegrees = entries.get(0).getOrient_x();

		for (SensorEntry entry : entries) {

			double origOrientZ = entry.getOrient_z();

			double remappedOrientX = 0; // = remap();
			double remappedOrientY = 0;
			double remappedOrientZ = 0;

			entry.setOrient_x(remappedOrientX);
			entry.setOrient_y(remappedOrientY);
			entry.setOrient_z(remappedOrientZ);

		}

		return entries;
	}
}
