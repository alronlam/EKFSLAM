package vins.motionestimation;

import java.util.ArrayList;
import java.util.Calendar;

import stepbasedins.data.SensorEntry;
import Jama.Matrix;

import commondata.DevicePose;

public class IntegrateMotionEstimation implements MotionEstimation {
	// timer counters
	long start = -1;
	long curr = -1;

	// sensor data
	ArrayList<SensorEntry> entries = new ArrayList<SensorEntry>();

	// overflow data, add when next time
	ArrayList<SensorEntry> overflow = new ArrayList<SensorEntry>();

	public void inputData(SensorEntry s) {
		if (start == -1)
			startTimer();
		else
			curr = Calendar.getInstance().getTimeInMillis();

		if (start + 350 >= curr) {
			entries.add(s);
		} else {
			overflow.add(s);
		}

	}

	public DevicePose getHeadingAndDisplacement() {

		double heading;
		double orientXAvg, orientYAvg, orientZAvg;

		orientXAvg = 0;
		orientYAvg = 0;
		orientZAvg = 0;

		for (SensorEntry s : entries) {
			orientXAvg += s.getOrient_x();
			orientYAvg += s.getOrient_y();
			orientZAvg += s.getOrient_z();
		}

		if (entries.size() != 0) {
			orientXAvg /= entries.size();
			orientYAvg /= entries.size();
			orientZAvg /= entries.size();
		}

		heading = orientXAvg;
		orientXAvg = Math.toRadians(orientXAvg);
		orientYAvg = Math.toRadians(orientYAvg);
		orientZAvg = Math.toRadians(orientZAvg);

		double cosVal, sinVal;

		// TODO: might need to double check this
		// orientX is actually rotation along Z, hence rotZArr
		// orientY is the rotation along X, hence rotYArr
		// orientZ is rotation along Y, hence rotZArr

		cosVal = Math.cos(orientXAvg);
		sinVal = Math.sin(orientXAvg);
		double[][] rotZArr = { { cosVal, -sinVal, 0 }, { sinVal, cosVal, 0 }, { 0, 0, 1 } };
		Matrix rotZ = new Matrix(rotZArr);

		cosVal = Math.cos(orientYAvg);
		sinVal = Math.sin(orientYAvg);
		double[][] rotXArr = { { 1, 0, 0 }, { 0, cosVal, -sinVal }, { 0, sinVal, cosVal } };
		Matrix rotX = new Matrix(rotXArr);

		cosVal = Math.cos(orientZAvg);
		sinVal = Math.sin(orientZAvg);
		double[][] rotYArr = { { cosVal, 0, sinVal }, { 0, 1, 0 }, { -sinVal, 0, cosVal } };
		Matrix rotY = new Matrix(rotYArr);

		Matrix rotFinal = rotX.times(rotY).times(rotZ).transpose();

		// D O U B L E I N T E G R A T I O N
		double[] pos = new double[3];

		// x = a*t^2/2 (t is in seconds)

		for (SensorEntry s : entries) {
			pos[0] += s.getAcc_x();
			pos[1] += s.getAcc_y();
			pos[2] += s.getAcc_z();
		}

		if (entries.size() > 0) {
			pos[0] /= entries.size();
			pos[1] /= entries.size();
			pos[2] /= entries.size();
		}

		Matrix xyzMatrix = new Matrix(new double[][] { pos });

		xyzMatrix = xyzMatrix.times(rotFinal);

		pos[0] = xyzMatrix.get(0, 0);
		pos[1] = xyzMatrix.get(0, 1);
		pos[2] = xyzMatrix.get(0, 2);

		pos[0] *= Math.pow((curr - start), 2) / 2000000;
		pos[1] *= Math.pow((curr - start), 2) / 2000000;
		pos[2] *= Math.pow((curr - start), 2) / 2000000;

		// Log.i("ME", heading + " " + numInstances + "\n");

		// clear all the arraylist
		entries.clear();

		// if there are overflows
		for (int i = 0; i < overflow.size(); ++i) {
			entries.add(overflow.get(i));
		}

		overflow.clear();

		// and reset the timers
		start = -1;
		curr = -1;

		// Log.i("ME", "POS Vector: " + pos[0] + " " + pos[1] + " " + pos[2]);

		// swap y and z so that x,y represents top-view position
		DevicePose devicePose = new DevicePose(pos[0], pos[2], pos[1], heading);
		devicePose.setRotWorld(rotFinal);

		return devicePose;
	}

	public void startTimer() {
		start = Calendar.getInstance().getTimeInMillis();
		curr = Calendar.getInstance().getTimeInMillis();
	}
}