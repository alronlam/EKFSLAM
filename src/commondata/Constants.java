package commondata;

import java.util.ArrayList;
import java.util.List;

public class Constants {

	// Bluetooth related
	public static final int REQUEST_ENABLE_BT = 1;
	public static final String SERVER_DEVICE_NAME = "SERVER";

	// Bluetooth Message related
	public static final byte SIGNAL_SERVER_START_MSG = 0;
	public static final byte SIGNAL_CAMERA_TO_TAKE_IMAGE = 2;
	public static final byte SIGNAL_SERVER_STOP_MSG = 3;

	// Time related
	public static final int MS_OVERALL_CYCLE_FREQUENCY = 350;
	public static final int MS_INS_SAMPLING_FREQUENCY = 10;
	public static final int MS_FREQUENCY_FOR_CAMERA_CAPTURE = 350;

	public static final int MS_IMG_DURATION = 66;
	public static final int MS_IMU_DURATION = 350;

	// Log related
	public static final String FOLDER_STRAIGHT_7M = "straight7m";
	public static final String FOLDER_STANDING_30S = "standing30s";
	public static final String FOLDER_MICOHOUSE_36M = "micohouse36m";
	public static final String FOLDER_GOX_49M = "dataset_48.6m_45s_80_straight";
	public static final String FOLDER_MIGUEL_STRAIGHT = "miguel_straight_30_19.8m";
	public static final String FOLDER_MIGUEL_RECTANGLE = "miguel_rect_136_90.3m";
	public static final String FOLDER_MIGUEL_RECTANGLE_ALRON = "miguel_alron_rect_93.6m_149";
	public static final String FOLDER_LS_STRAIGHT = "LS Straight";
	public static final String FOLDER_YUCH_LOBBY_RECTANGLE = "Yuch Big Rectangle";
	public static final String FOLDER_YUCH_SMALLER_RECTANGLE_ALRON = "Yuch Smaller Rectangle - Alron";
	public static final String FOLDER_YUCH_SMALLER_RECTANGLE_IVAN = "Yuch Smaller Rectangle - Ivan";
	public static final String FOLDER_TriTest = "TriTest";

	// Feb 2015 Datasets

	/* Miguel Datasets */
	public static final String FOLDER_RECT1_MIGUEL2_S3 = "Ivan Miguel 2nd Floor S3 Feb 23";
	public static final String FOLDER_RECT1_MIGUEL3_S3 = "Ivan Miguel 3rd Floor S3 Feb 23";
	public static final String FOLDER_RECT1_MIGUEL3_S4 = "Ivan Miguel 3rd Floor S4 Feb 24";
	public static final String FOLDER_RECT1_MIGUEL4_S4 = "Ivan Miguel 4th Floor 1 Pass S4 Mar 3";
	public static final String FOLDER_RECT2_MIGUEL4_S4 = "Ivan Miguel 4th Floor 2 Pass S4 Mar 3";

	public static final String FOLDER_RECT1_MIGUEL_S4_ALRON_MAR5 = "Alron Miguel 4th Floor 1 Pass Mar 5 - 122 Steps";
	public static final String FOLDER_RECT2_MIGUEL_S4_ALRON_MAR5 = "Alron Miguel 4th Floor 2 Pass Mar 5 - 246 Steps (2nd Run)";
	public static final String FOLDER_RECT1_MIGUEL_S4_IVAN_MAR5 = "Ivan Miguel 4th Floor 1 Pass Mar 5 - 110 Steps";
	public static final String FOLDER_RECT2_MIGUEL_S4_IVAN_MAR5 = "Ivan Miguel 4th Floor 2 Pass Mar 5 - 213 Steps";
	public static final String FOLDER_RECT1_MIGUEL_S4_IVAN_MAR6 = "Ivan Miguel 4th Floor 1 Pass Mar 6 - 110 Steps S3cam";
	public static final String FOLDER_RECT2_MIGUEL_S4_IVAN_MAR6 = "Ivan Miguel 4th Floor 2 Pass Mar 6 - 214 Steps S3cam";

	public static final String FOLDER_RECT1_MIGUEL_S4_IVAN_MAR27_1 = "Ivan Miguel 4th Floor 1 Pass Mar 27 Trial 1 - 109 Steps";
	public static final String FOLDER_RECT1_MIGUEL_S4_IVAN_MAR27_2 = "Ivan Miguel 4th Floor 1 Pass Mar 27 Trial 2 - 106 Steps";
	public static final String FOLDER_RECT2_MIGUEL_S4_IVAN_MAR24_1 = "Ivan Miguel 4th Floor 2 Pass Mar 24 Trial 1 - 203 Steps";
	public static final String FOLDER_RECT2_MIGUEL_S4_IVAN_MAR24_2 = "Ivan Miguel 4th Floor 2 Pass Mar 24 Trial 2 - 204 Steps";

	/* SJ Datasets */
	public static final String FOLDER_STRT1_SJ5_S4 = "Ivan SJ 5th Floor S4 Feb 26";
	public static final String FOLDER_STRT2_SJ6_S4 = "Ivan SJ 6th Floor S4 Feb 26";
	public static final String FOLDER_STRT1_SJ6_S4_MAR3 = "Ivan SJ 6th Floor 1 Pass Mar S4 3";
	public static final String FOLDER_STRT2_SJ6_S4_MAR3 = "Ivan SJ 6th Floor 2 Pass Mar S4 3";

	public static final String FOLDER_STRT1_SJ6_PARTIAL_S4_MAR5_BLACK_CAM = "Ivan SJ 6th Floor 1 Pass Mar 5 - 122 Steps Partial [Black Camera]";
	public static final String FOLDER_STRT2_SJ6_PARTIAL_S4_MAR5 = "Ivan SJ 6th Floor 2 Pass Mar 5 - 244 Steps Partial";
	public static final String FOLDER_STRT1_SJ6_S4_MAR6 = "Ivan SJ 6th Floor 1 Pass Mar 6 - 147 Steps S3cam";

	public static final String FOLDER_STRT1_SJ6_IVAN_MAR27_1 = "Ivan SJ 6th Floor 1 Pass Mar 27 Trial 1 - 137 Steps";
	public static final String FOLDER_STRT1_SJ6_IVAN_MAR27_2 = "Ivan SJ 6th Floor 1 Pass Mar 27 Trial 2 - 138 Steps";
	public static final String FOLDER_STRT2_SJ6_IVAN_MAR24 = "Ivan SJ 6th Floor 2 Pass Mar 24 Trial 1 - 282 Steps";
	public static final String FOLDER_STRT2_SJ6_IVAN_MAR27 = "Ivan SJ 6th Floor 2 Pass Mar 27 Trial 1 - 287 Steps";

	/* LS Datasets */
	public static final String FOLDER_STRT1_LS1_S3CAM_IVAN_MAR6 = "Ivan LS 1st Floor 1 Pass Mar 6 - 162 Steps S3cam (2nd Run)";
	public static final String FOLDER_STRT2_LS1_S3CAM_IVAN_MAR6 = "Ivan LS 1st Floor 2 Pass Mar 6 - 332 Steps S3cam";

	public static final String FOLDER_STRT1_LS1_IVAN_MAR27_1 = "Ivan LS 1st Floor 1 Pass Mar 27 Trial 1 - 151 Steps";
	public static final String FOLDER_STRT1_LS1_IVAN_MAR27_2 = "Ivan LS 1st Floor 1 Pass Mar 27 Trial 2 - 158 Steps";
	public static final String FOLDER_STRT2_LS1_IVAN_MAR27_1 = "Ivan LS 1st Floor 2 Pass Mar 27 Trial 1 - 310 Steps";
	public static final String FOLDER_STRT2_LS1_IVAN_MAR27_2 = "Ivan LS 1st Floor 2 Pass Mar 27 Trial 2 - 317 Steps";

	/* Razon Datasets */
	public static final String FOLDER_OVAL1_RAZON8_IVAN_MAR24_1 = "Ivan Razon 8th Floor 1 Pass Mar 24 Trial 1 - 222 Steps";
	public static final String FOLDER_OVAL1_RAZON8_IVAN_MAR24_2 = "Ivan Razon 8th Floor 1 Pass Mar 24 Trial 2 - 225 Steps";
	public static final String FOLDER_OVAL2_RAZON8_IVAN_MAR24 = "Ivan Razon 8th Floor 2 Pass Mar 24 Trial 2 - 441 Steps";
	public static final String FOLDER_OVAL2_RAZON8_IVAN_MAR27 = "Ivan Razon 8th Floor 2 Pass Mar 27 Trial 1 - 440 Steps";

	public static final List<String> SJ_PARTIAL_DATASETS;
	static {
		SJ_PARTIAL_DATASETS = new ArrayList<String>();
		SJ_PARTIAL_DATASETS.add(FOLDER_STRT1_SJ6_PARTIAL_S4_MAR5_BLACK_CAM);
		SJ_PARTIAL_DATASETS.add(FOLDER_STRT2_SJ6_PARTIAL_S4_MAR5);
	}

	public static final List<String> SJ_DATASETS;
	static {
		SJ_DATASETS = new ArrayList<String>();
		SJ_DATASETS.add(FOLDER_STRT1_SJ6_IVAN_MAR27_1);
		SJ_DATASETS.add(FOLDER_STRT1_SJ6_IVAN_MAR27_2);
		SJ_DATASETS.add(FOLDER_STRT2_SJ6_IVAN_MAR24);
		SJ_DATASETS.add(FOLDER_STRT2_SJ6_IVAN_MAR27);
	}

	public static final List<String> LS_DATASETS;
	static {
		LS_DATASETS = new ArrayList<String>();
		LS_DATASETS.add(FOLDER_STRT1_LS1_IVAN_MAR27_1);
		LS_DATASETS.add(FOLDER_STRT1_LS1_IVAN_MAR27_2);
		LS_DATASETS.add(FOLDER_STRT2_LS1_IVAN_MAR27_1);
		LS_DATASETS.add(FOLDER_STRT2_LS1_IVAN_MAR27_2);
	}

	public static final List<String> MIGUEL_DATASETS;
	static {
		MIGUEL_DATASETS = new ArrayList<String>();

		MIGUEL_DATASETS.add(FOLDER_RECT1_MIGUEL_S4_IVAN_MAR5);
		MIGUEL_DATASETS.add(FOLDER_RECT1_MIGUEL_S4_IVAN_MAR27_1);
		MIGUEL_DATASETS.add(FOLDER_RECT1_MIGUEL_S4_IVAN_MAR27_2);
		MIGUEL_DATASETS.add(FOLDER_RECT2_MIGUEL_S4_IVAN_MAR5);
		MIGUEL_DATASETS.add(FOLDER_RECT2_MIGUEL_S4_IVAN_MAR24_1);
		MIGUEL_DATASETS.add(FOLDER_RECT2_MIGUEL_S4_IVAN_MAR24_2);
	}

	public static final List<String> ASYNC_DATASETS;

	static {
		ASYNC_DATASETS = new ArrayList<>();
		ASYNC_DATASETS.add(FOLDER_RECT1_MIGUEL2_S3);
		ASYNC_DATASETS.add(FOLDER_RECT1_MIGUEL3_S3);
		ASYNC_DATASETS.add(FOLDER_RECT1_MIGUEL3_S4);
		ASYNC_DATASETS.add(FOLDER_STRT1_SJ5_S4);
		ASYNC_DATASETS.add(FOLDER_STRT2_SJ6_S4);
	}

	// Others
	public static final String INS_DATA_HEADER = "Acc_x,Acc_y,Acc_z,Gyro_x,Gyro_y,Gyro_z,Orient_x,Orient_y,Orient_z,Time\n";

}
