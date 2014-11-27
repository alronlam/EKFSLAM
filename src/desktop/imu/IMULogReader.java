package desktop.imu;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

import stepbasedins.data.SensorEntry;

public class IMULogReader {

	private File targetFolder;

	public IMULogReader(String folderLocation) {
		this.targetFolder = new File(folderLocation);
	}

	public List<IMUReadingsBatch> readSensorEntries() {

		Scanner scanner;
		try {

			File[] csvFiles = this.targetFolder.listFiles(); // assuming it's
																// all .csv
			List<IMUReadingsBatch> dataset = new ArrayList<IMUReadingsBatch>();

			for (File csvFile : csvFiles) {
				scanner = new Scanner(csvFile);

				// Ignore the header file
				scanner.nextLine();

				IMUReadingsBatch currBatch = new IMUReadingsBatch();
				while (scanner.hasNext()) {
					String currLine = scanner.nextLine();
					String[] tokens = currLine.split(",");
					SensorEntry currEntry = Helper.createSensorEntryFromStringTokens(tokens);
					currBatch.addEntry(currEntry);
				}
				currBatch.setEntries(IMUOrientationRemapper.remap(currBatch.getEntries()));
				dataset.add(currBatch);
				scanner.close();
			}

			return dataset;

		} catch (FileNotFoundException e) {
			e.printStackTrace();
		}

		return new ArrayList<IMUReadingsBatch>();
	}
}
