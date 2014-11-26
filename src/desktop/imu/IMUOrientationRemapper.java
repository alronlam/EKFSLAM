package desktop.imu;

import java.util.ArrayList;

import stepbasedins.data.SensorEntry;

public class IMUOrientationRemapper {

	// method stub
	public static ArrayList<SensorEntry> remap(ArrayList<SensorEntry> entries) {

		if (entries.size() == 0)
			return entries;

		double initialHeadingDegrees = entries.get(0).getOrient_x();

		for (SensorEntry entry : entries) {

			// System.out.println("Before remapping: " + entry.getOrient_x());
			double remappedOrientX = Remap.mod360(Remap.mod360(-entry.getOrient_x()) + 90);
			double remappedOrientY = 0;
			double remappedOrientZ = 0;
			// System.out.println("After remapping: " + remappedOrientX);
			entry.setOrient_x(remappedOrientX);
			// entry.setOrient_y(remappedOrientY);
			// entry.setOrient_z(remappedOrientZ);

		}

		return entries;
	}
}
