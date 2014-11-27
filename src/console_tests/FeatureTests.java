package console_tests;

import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.Mat;

import commondata.Constants;
import desktop.img.ImgLogReader;
import idp.features.FeatureUpdate;
import idp.features.FeatureManager;
import junit.framework.TestCase;

public class FeatureTests extends TestCase {
	static {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
	}
	
	private final String TARGET_FOLDER = Constants.FOLDER_STRAIGHT_7M;
	private final int IMAGES_TO_TEST = 50;
	private FeatureManager featureManager;
	private List<Mat> imgDataset;
	private FeatureUpdate update;
	private double previousNew;
	private double previousCurrent;
	
	
	@Override
	protected void setUp() throws Exception {
		ImgLogReader imgLogReader = new ImgLogReader("data/" + TARGET_FOLDER + "/img");
		imgDataset = imgLogReader.readImages();
		featureManager = new FeatureManager();
	}
	
	
	public void testContinuousFeatureNumber() {
		int images = 0; 
		for (Mat image : imgDataset) {
			if (images >= IMAGES_TO_TEST) {
				break;
			}
			update = featureManager.getFeatureUpdate(image);
			if (images == 0) {
				previousCurrent = update.getCurrentPoints().size();
				previousNew = update.getNewPoints().size();
				images++;
				continue;
			}
			
			double currentBad = update.getBadPointsIndex().size();
			double currentCurrent = update.getCurrentPoints().size();
			double currentTotal = currentBad + currentCurrent;
			
			double previousTotal = previousCurrent + previousNew;
			
			String message = "Image " + images + ".";
			assertEquals(message, previousTotal, currentTotal);
			
			previousCurrent = currentCurrent;
			previousNew = update.getNewPoints().size();
		}
	}
}
