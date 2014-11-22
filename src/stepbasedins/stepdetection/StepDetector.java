package stepbasedins.stepdetection;

import java.util.ArrayList;

import stepbasedins.data.DetectedEntry;
import stepbasedins.data.SensorEntry;

public interface StepDetector {
	public ArrayList<DetectedEntry> detectSteps(ArrayList<SensorEntry> batch);
}
