package pathcomparison;

import java.util.ArrayList;

import util.FileLog;

public class ComparisonResultWriter {

	public static void writeToCSVFile(String name, ArrayList<Double> list) {

		FileLog fileLog = new FileLog(name);

		for (int i = 0; i < list.size(); i++) {
			fileLog.append(list.get(i) + "\r\n");
		}

		fileLog.writeToFile();
	}

	public static void writeToCSVFile(String name, ArrayList<Double> distanceList, ArrayList<Double> headingList) {

		FileLog fileLog = new FileLog(name);

		int limit = Math.min(distanceList.size(), headingList.size());

		for (int i = 0; i < limit; i++) {
			fileLog.append(distanceList.get(i) + "," + headingList.get(i) + "\r\n");
		}

		fileLog.writeToFile();

	}

}
