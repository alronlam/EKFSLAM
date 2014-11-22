package driver;

import features.FeatureUpdate;
import imu.IMULogReader;
import imu.IMUReadingsBatch;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.List;

public class MainDriver {

	public static void main(String[] args) {

		/* Initialize the three controllers */
		BreadcrumbController breadcrumb = new BreadcrumbController();
		INSController ins = new INSController();
		VINSController vins = new VINSController();

		/* Initialize the logs for all three techniques */
		StringBuilder breadcrumbLog = new StringBuilder();
		StringBuilder insLog = new StringBuilder();
		StringBuilder vinsLog = new StringBuilder();

		/* Load IMU Dataset */
		IMULogReader imuLogReader = new IMULogReader("data/imu");
		List<IMUReadingsBatch> imuDataset = imuLogReader.readSensorEntries();

		/* Load Images Dataset */

		/* Make sure their sizes match */

		int datasetSize = imuDataset.size();

		for (int i = 0; i < datasetSize; i++) {

			/* IMU Predict */
			IMUReadingsBatch currIMUBatch = imuDataset.get(i);
			breadcrumb.predict(currIMUBatch);
			ins.predict(currIMUBatch);
			vins.predict(currIMUBatch);

			/* Image Update */
			FeatureUpdate featureUpdate = null; // change this soon
			breadcrumb.update(featureUpdate);
			vins.update(featureUpdate);

			/* Update the logs per controller */
			breadcrumbLog.append(breadcrumb.getDeviceCoords() + "\n");
			insLog.append(ins.getDeviceCoords() + "\n");
			vinsLog.append(vins.getDeviceCoords() + "\n");
		}

		String folder = "results/";
		writeToFile(folder + "breadcrumb.csv", breadcrumbLog.toString());
		writeToFile(folder + "ins.csv", insLog.toString());
		writeToFile(folder + "vins.csv", vinsLog.toString());
	}

	private static void writeToFile(String targetFilePath, String toWrite) {
		try {
			FileWriter fw = new FileWriter(new File(targetFilePath));
			fw.write(toWrite);
			fw.close();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}
