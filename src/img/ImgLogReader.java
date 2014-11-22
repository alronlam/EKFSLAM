package img;

import img.util.ImageIO;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Mat;

public class ImgLogReader {

	private File targetFolder;

	public ImgLogReader(String folderLocation) {
		this.targetFolder = new File(folderLocation);
	}

	public List<Mat> readImages() {

		ArrayList<Mat> images = new ArrayList<Mat>();

		ImageIO io = new ImageIO(targetFolder);
		while (io.hasNext()) {
			Mat image = io.loadNext();
			images.add(image);
		}

		return images;
	}
}
