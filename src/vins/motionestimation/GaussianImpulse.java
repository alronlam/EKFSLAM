package vins.motionestimation;

import java.util.Random;

import stepbasedins.data.SensorEntry;

import commondata.DevicePose;

public class GaussianImpulse implements MotionEstimation {

	private Random random;

	public GaussianImpulse() {
		this.random = new Random();
	}

	@Override
	public void inputData(SensorEntry s) {
	}

	@Override
	public DevicePose getHeadingAndDisplacement() throws Exception {

		return null;
	}

	@Override
	public void startTimer() {
		// TODO Auto-generated method stub

	}

}
