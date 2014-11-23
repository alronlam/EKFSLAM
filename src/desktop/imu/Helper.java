package desktop.imu;

import stepbasedins.data.SensorEntry;

public class Helper {

	public static Double[] convertToDouble(String[] tokens) {
		Double[] doubleArr = new Double[tokens.length - 1];

		for (int i = 0; i < tokens.length - 1; i++) {
			doubleArr[i] = Double.parseDouble(tokens[i]);
		}

		return doubleArr;
	}

	public static SensorEntry createSensorEntryFromStringTokens(String[] tokens) {
		Double[] features = Helper.convertToDouble(tokens);
		SensorEntry currEntry = new SensorEntry();

		if (tokens.length == 10) {
			currEntry.setAcc_x(features[0]);
			currEntry.setAcc_y(features[1]);
			currEntry.setAcc_z(features[2]);

			currEntry.setGyro_x(features[3]);
			currEntry.setGyro_y(features[4]);
			currEntry.setGyro_z(features[5]);

			currEntry.setOrient_x(features[6]);
			currEntry.setOrient_y(features[7]);
			currEntry.setOrient_z(features[8]);

			currEntry.setTimeRecorded(Double.valueOf(tokens[tokens.length - 1]).longValue());

		}

		currEntry.buildSensorList();

		return currEntry;
	}
}
