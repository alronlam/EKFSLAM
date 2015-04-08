package pathcomparison;

import java.io.File;
import java.util.ArrayList;

import util.FileLog;
import util.MathHelper;

import commondata.PointDouble;

public class CSVVectorComparatorDriver {

	public static void main(String[] args) {
		// for each folder in the csvcompare folder,
		// print folder name, and determine its correct dataset
		// read all csv files:
		// count number of points, generate correct path.
		// print csv file and its accuracy

		String FOLDER_CSV_RESULT_POINTS = "csv_result_points";
		String SUB_FOLDER = "result_errors";

		ArrayList<File> folders = CSVReader.getFolders(FOLDER_CSV_RESULT_POINTS);

		for (File datasetFolder : folders) {
			FileLog fileLog = new FileLog(datasetFolder + "/" + SUB_FOLDER + "/distance_heading_position_error.txt");
			System.out.println("Folder: " + datasetFolder);
			ArrayList<File> csvFiles = CSVReader.getCSVFilesFromFolder(datasetFolder);

			for (File csvFile : csvFiles) {

				System.out.println(csvFile.getName());

				// The estimated and correct path points
				ArrayList<PointDouble> estimatedPathPoints = CSVReader.readCSVPoints(csvFile);
				ArrayList<PointDouble> correctPathPoints = PathGenerator.generate(datasetFolder.getName(),
						estimatedPathPoints.size());

				// The estimated and correct path vectors (distance and heading)
				ArrayList<Vector> estimatedPathVectors = PointListToVectorListConverter
						.convertPointList(estimatedPathPoints);
				ArrayList<Vector> correctPathVectors = PointListToVectorListConverter
						.convertPointList(correctPathPoints);

				// The errors in distance, heading, and position, per 350ms
				ArrayList<Double> distanceErrorList = PathVectorListComparator.getDistanceErrorList(
						estimatedPathVectors, correctPathVectors);
				ArrayList<Double> headingErrorList = PathVectorListComparator.getHeadingErrorList(estimatedPathVectors,
						correctPathVectors);
				ArrayList<Double> positionErrorList = PathComparator.getErrorList(estimatedPathPoints,
						correctPathPoints);

				ComparisonResultWriter.writeToCSVFile(datasetFolder + "/" + SUB_FOLDER + "/" + csvFile.getName(),
						distanceErrorList, headingErrorList, positionErrorList);

				fileLog.append("\r\n" + csvFile.getName() + "\r\n");
				fileLog.append("Avg Distance Error: " + MathHelper.getAverage(distanceErrorList) + "\r\n");
				fileLog.append("Avg Heading Error (Degrees): " + MathHelper.getAverage(headingErrorList) + "\r\n");
				fileLog.append("Total Position Error: " + MathHelper.getSum(positionErrorList) + "\r\n");
				fileLog.append("Avg Position Error: " + MathHelper.getAverage(positionErrorList) + "\r\n");
			}

			fileLog.writeToFile();

			System.out.println();

		}
	}
}
