package commondata;

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
	
	public static final String FOLDER_LS_NOFAIL = "ls mod";

	// Others
	public static final String INS_DATA_HEADER = "Acc_x,Acc_y,Acc_z,Gyro_x,Gyro_y,Gyro_z,Orient_x,Orient_y,Orient_z,Time\n";

}