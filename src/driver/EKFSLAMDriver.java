package driver;

import imu.IMULogReader;
import imu.IMUReadingsBatch;

import java.util.List;

public class EKFSLAMDriver {

	public static void main(String[] args) {

		IMULogReader imuLogReader = new IMULogReader("data/imu");

		List<IMUReadingsBatch> imuDataset = imuLogReader.readSensorEntries();

	}
}
