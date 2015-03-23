package pathcomparison;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.Scanner;

import commondata.PointDouble;

public class CSVReader {

	public static ArrayList<PointDouble> readCSVPoints(File targetFile) {
		// path is assumed to point to a csv file

		try {
			Scanner scanner = new Scanner(targetFile);
			ArrayList<PointDouble> points = new ArrayList<PointDouble>();

			while (scanner.hasNext()) {
				String[] tokens = scanner.nextLine().split(",");

				double currX = Double.parseDouble(tokens[0]);
				double currY = Double.parseDouble(tokens[1]);

				points.add(new PointDouble(currX, currY));

			}
			return points;

		} catch (FileNotFoundException e) {
			e.printStackTrace();
			return new ArrayList<PointDouble>();
		}

	}

	public static ArrayList<File> getCSVFilesFromFolder(File targetFolder) {
		return filterCSVFiles(targetFolder.listFiles());
	}

	public static ArrayList<File> filterCSVFiles(File[] files) {
		ArrayList<File> csvFiles = new ArrayList<File>();
		for (File file : files) {
			if (file.getPath().contains(".csv"))
				csvFiles.add(file);
		}
		return csvFiles;
	}
}
