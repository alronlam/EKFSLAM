package pathcomparison;

import java.util.ArrayList;

import util.FileLog;

public class ComparisonResultWriter {

	public static void writeToCSVFile(String name, ArrayList<Double> distanceList, ArrayList<Double> headingList,
			ArrayList<Double> positionList) {

		FileLog fileLog = new FileLog(name);

		int limit = Math.min(positionList.size(), Math.min(distanceList.size(), headingList.size()));

		for (int i = 0; i < limit; i++) {
			fileLog.append(distanceList.get(i) + "," + headingList.get(i) + "," + positionList.get(i) + "\r\n");
		}

		fileLog.writeToFile();

	}
}
