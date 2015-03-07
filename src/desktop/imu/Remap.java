package desktop.imu;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Size;

import Jama.Matrix;

public class Remap {

	static {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
	}

	// public static void main(String args[]) {
	// double init[] = { 0, 0, 0 };
	//
	// Mat refMat = convertToRotMat(init);
	//
	// double samp[] = { 30, 0, 0 };
	//
	// double res[] = remapRotVec(samp, refMat);
	//
	// System.out.println(res[0]);
	// }

	public static void oldmain(String args[]) {
		float init[] = { -15, -10, -20 };

		float z = (float) Math.toRadians(init[0]);
		float x = (float) Math.toRadians(init[1]);
		float y = (float) Math.toRadians(init[2]);

		System.out.println(z + ", " + x + ", " + y);

		Mat rotMX = getRx(x, true);
		Mat rotMY = getRy(y, false);
		Mat rotMZ = getRz(z, true);

		Mat temp1 = new Mat(), rotM = new Mat(), nullMat = new Mat();

		Core.gemm(rotMZ, rotMX, 1, nullMat, 0, temp1);
		Core.gemm(temp1, rotMY, 1, nullMat, 0, rotM);

		System.out.println(rotM.dump());
		float rotMat[] = { (float) rotM.get(0, 0)[0], (float) rotM.get(0, 1)[0], (float) rotM.get(0, 2)[0], (float) rotM.get(1, 0)[0], (float) rotM.get(1, 1)[0],
				(float) rotM.get(1, 2)[0], (float) rotM.get(2, 0)[0], (float) rotM.get(2, 1)[0], (float) rotM.get(2, 2)[0] };

		float[] values = new float[3];
		values = getOrientation(rotMat, values);
		System.out.println(values[0] + ", " + values[1] + ", " + values[2]);

	}

	// public static void main(String args[]){
	// float init[] = {15, -10, 20};
	//
	//
	// System.out.println(init[0]+", "+init[1]+", "+init[2]);
	//
	// float z = (float) Math.toRadians(init[0]);
	// float x = (float) Math.toRadians(init[1]);
	// float y = (float) Math.toRadians(init[2]);
	//
	// System.out.println(z+", "+x+", "+y);
	//
	// Mat rotM1 = getRx(y,true);
	// Mat rotM2 = getRy(x,false);
	// Mat rotM3 = getRz(z,true);
	//
	// Mat temp1 = new Mat(), rotM = new Mat(), nullMat = new Mat();
	//
	// Core.gemm(rotM1.t(), rotM2, 1, nullMat, 0, temp1);
	// Core.gemm(temp1.t(), rotM3, 1, nullMat, 0, rotM);
	//
	// System.out.println(rotM.dump());
	// float rotMat[] = {(float)rotM.get(0, 0)[0],(float)rotM.get(0, 1)[0],(float)rotM.get(0, 2)[0],
	// (float)rotM.get(1, 0)[0],(float)rotM.get(1, 1)[0],(float)rotM.get(1, 2)[0],
	// (float)rotM.get(2, 0)[0],(float)rotM.get(2, 1)[0],(float)rotM.get(2, 2)[0]};
	//
	// // for(int i = 0; i < 9 ; ++i)
	// // System.out.println(rotMat[i]);
	//
	// // System.out.println(rotM.dump());
	//
	// float [] values = new float[3];
	// values = getOrientation(rotMat, values);
	// System.out.println(values[0]+", "+values[1]+", "+values[2]);
	//
	//
	// System.out.println(Math.toDegrees(values[0])+", "+Math.toDegrees(values[1])+", "+Math.toDegrees(values[2]));
	//
	// }

	public static final int AXIS_X = 1;
	/** see {@link #remapCoordinateSystem} */
	public static final int AXIS_Y = 2;
	/** see {@link #remapCoordinateSystem} */
	public static final int AXIS_Z = 3;

	public static void main(String args[]) {

	}

	public static float[] recoverOrient(float[] orient) {

		float[] rotMat = getRotationMatrix(orient);

		float remap[] = new float[9];
		remapCoordinateSystem(rotMat, AXIS_X, AXIS_Z, remap);
		float remOrient[] = new float[3];

		getOrientation(remap, remOrient);

		for (int i = 0; i < remOrient.length; ++i) {
			remOrient[i] = (float) Math.toDegrees(remOrient[i]);
			if (remOrient[i] < 0)
				remOrient[i] += 360;
		}

		return remOrient;
	}

	private static final float[] mTempMatrix = new float[16];

	public static boolean remapCoordinateSystem(float[] inR, int X, int Y, float[] outR) {
		if (inR == outR) {
			final float[] temp = mTempMatrix;
			synchronized (temp) {
				// we don't expect to have a lot of contention
				if (remapCoordinateSystemImpl(inR, X, Y, temp)) {
					final int size = outR.length;
					for (int i = 0; i < size; i++)
						outR[i] = temp[i];
					return true;
				}
			}
		}
		return remapCoordinateSystemImpl(inR, X, Y, outR);
	}

	private static boolean remapCoordinateSystemImpl(float[] inR, int X, int Y, float[] outR) {
		/*
		 * X and Y define a rotation matrix 'r':
		 * 
		 * (X==1)?((X&0x80)?-1:1):0 (X==2)?((X&0x80)?-1:1):0 (X==3)?((X&0x80)?-1:1):0 (Y==1)?((Y&0x80)?-1:1):0 (Y==2)?((Y&0x80)?-1:1):0 (Y==3)?((X&0x80)?-1:1):0 r[0] ^ r[1]
		 * 
		 * where the 3rd line is the vector product of the first 2 lines
		 */

		final int length = outR.length;
		if (inR.length != length)
			return false; // invalid parameter
		if ((X & 0x7C) != 0 || (Y & 0x7C) != 0)
			return false; // invalid parameter
		if (((X & 0x3) == 0) || ((Y & 0x3) == 0))
			return false; // no axis specified
		if ((X & 0x3) == (Y & 0x3))
			return false; // same axis specified

		// Z is "the other" axis, its sign is either +/- sign(X)*sign(Y)
		// this can be calculated by exclusive-or'ing X and Y; except for
		// the sign inversion (+/-) which is calculated below.
		int Z = X ^ Y;

		// extract the axis (remove the sign), offset in the range 0 to 2.
		final int x = (X & 0x3) - 1;
		final int y = (Y & 0x3) - 1;
		final int z = (Z & 0x3) - 1;

		// compute the sign of Z (whether it needs to be inverted)
		final int axis_y = (z + 1) % 3;
		final int axis_z = (z + 2) % 3;
		if (((x ^ axis_y) | (y ^ axis_z)) != 0)
			Z ^= 0x80;

		final boolean sx = (X >= 0x80);
		final boolean sy = (Y >= 0x80);
		final boolean sz = (Z >= 0x80);

		// Perform R * r, in avoiding actual muls and adds.
		final int rowLength = ((length == 16) ? 4 : 3);
		for (int j = 0; j < 3; j++) {
			final int offset = j * rowLength;
			for (int i = 0; i < 3; i++) {
				if (x == i)
					outR[offset + i] = sx ? -inR[offset + 0] : inR[offset + 0];
				if (y == i)
					outR[offset + i] = sy ? -inR[offset + 1] : inR[offset + 1];
				if (z == i)
					outR[offset + i] = sz ? -inR[offset + 2] : inR[offset + 2];
			}
		}
		if (length == 16) {
			outR[3] = outR[7] = outR[11] = outR[12] = outR[13] = outR[14] = 0;
			outR[15] = 1;
		}
		return true;
	}

	public static float[] getRotationMatrix(float[] orient) {
		float z = (float) Math.toRadians(orient[0]);
		float x = (float) Math.toRadians(orient[1]);
		float y = (float) Math.toRadians(orient[2]);

		// System.out.println(z + ", " + x + ", " + y);

		Mat rotMX = getRx(x, true);
		Mat rotMY = getRy(y, false);
		Mat rotMZ = getRz(z, true);

		Mat temp1 = new Mat(), rotM = new Mat(), nullMat = new Mat();

		Core.gemm(rotMZ, rotMX, 1, nullMat, 0, temp1);
		Core.gemm(temp1, rotMY, 1, nullMat, 0, rotM);

		// System.out.println(rotM.dump());
		float rotMat[] = { (float) rotM.get(0, 0)[0], (float) rotM.get(0, 1)[0], (float) rotM.get(0, 2)[0], (float) rotM.get(1, 0)[0], (float) rotM.get(1, 1)[0],
				(float) rotM.get(1, 2)[0], (float) rotM.get(2, 0)[0], (float) rotM.get(2, 1)[0], (float) rotM.get(2, 2)[0] };

		return rotMat;

	}

	public static float[] getOrientation(float[] R, float values[]) {
		/*
		 * 4x4 (length=16) case: / R[ 0] R[ 1] R[ 2] 0 \ | R[ 4] R[ 5] R[ 6] 0 | | R[ 8] R[ 9] R[10] 0 | \ 0 0 0 1 /
		 * 
		 * 3x3 (length=9) case: / R[ 0] R[ 1] R[ 2] \ | R[ 3] R[ 4] R[ 5] | \ R[ 6] R[ 7] R[ 8] /
		 */
		if (R.length == 9) {
			values[0] = (float) Math.atan2(R[1], R[4]);
			values[1] = (float) Math.asin(-R[7]);
			values[2] = (float) Math.atan2(-R[6], R[8]);
		} else {
			values[0] = (float) Math.atan2(R[1], R[5]);
			values[1] = (float) Math.asin(-R[9]);
			values[2] = (float) Math.atan2(-R[8], R[10]);
		}
		return values;
	}

	public static double mod(double a, double b) {
		while (a < 0)
			a += b;
		return a % b;
	}

	public static double mod360(double a) {
		return mod(a, 360);
	}

	public static double[] remapRotVec(double rotVec[], Mat refMat) {
		Mat rotV = Mat.zeros(1, 3, CvType.CV_64F);

		rotV.put(0, 0, rotVec);

		Mat remapMat = new Mat(), nullMat = new Mat();
		Core.gemm(rotV, refMat, 1, nullMat, 0, remapMat);

		double remapVec[] = { remapMat.get(0, 0)[0], remapMat.get(0, 1)[1], remapMat.get(0, 2)[2] };

		return remapVec;
	}

	static Size size = new Size(3, 3);
	static int type = CvType.CV_64F;

	public static Mat convertToRotMat(double rotVec[]) {
		double z = Math.toRadians(mod360(rotVec[0]));
		double x = Math.toRadians(mod360(rotVec[1]));
		double y = Math.toRadians(mod360(rotVec[2]));

		Mat rotMZ = getRz(z, true);

		Mat rotMX = getRx(x, true);

		Mat rotMY = getRy(y, true);

		Mat temp1 = new Mat(), rotM = new Mat(), nullMat = new Mat();

		Core.gemm(rotMZ, rotMX, 1, nullMat, 0, temp1);
		Core.gemm(temp1, rotMY, 1, nullMat, 0, rotM);

		return rotM;
	}

	public static Mat getRx(double x, boolean flipped) {
		double cosVal, sinVal;

		cosVal = Math.cos(x);
		sinVal = Math.sin(x);

		if (flipped)
			sinVal = -sinVal;
		double[][] rotXArr = { { 1, 0, 0 }, { 0, cosVal, -sinVal }, { 0, sinVal, cosVal } };

		Mat rotMX = Mat.eye(size, type);
		rotMX.put(0, 0, rotXArr[0]);
		rotMX.put(1, 0, rotXArr[1]);
		rotMX.put(2, 0, rotXArr[2]);

		return rotMX;
	}

	public static Mat getRy(double y, boolean flipped) {
		double cosVal, sinVal;

		cosVal = Math.cos(y);
		sinVal = Math.sin(y);

		if (flipped)
			sinVal = -sinVal;
		double[][] rotYArr = { { cosVal, 0, sinVal }, { 0, 1, 0 }, { -sinVal, 0, cosVal } };

		Mat rotMY = Mat.eye(size, type);
		rotMY.put(0, 0, rotYArr[0]);
		rotMY.put(1, 0, rotYArr[1]);
		rotMY.put(2, 0, rotYArr[2]);

		return rotMY;
	}

	public static Mat getRz(double z, boolean flipped) {
		double cosVal, sinVal;

		cosVal = Math.cos(z);
		sinVal = Math.sin(z);

		if (flipped)
			sinVal = -sinVal;
		double[][] rotZArr = { { cosVal, -sinVal, 0 }, { sinVal, cosVal, 0 }, { 0, 0, 1 } };

		Mat rotMZ = Mat.eye(size, type);
		rotMZ.put(0, 0, rotZArr[0]);
		rotMZ.put(1, 0, rotZArr[1]);
		rotMZ.put(2, 0, rotZArr[2]);

		return rotMZ;
	}

	public static Matrix convertToRotMat_Jama(double rotVec[]) {

		double z = Math.toRadians(mod360(rotVec[0]));
		double x = Math.toRadians(mod360(rotVec[1]));
		double y = Math.toRadians(mod360(rotVec[2]));

		double cosVal, sinVal;

		cosVal = Math.cos(z);
		sinVal = Math.sin(z);
		double[][] rotZArr = { { cosVal, -sinVal, 0 }, { sinVal, cosVal, 0 }, { 0, 0, 1 } };
		Matrix rotMZ = new Matrix(rotZArr);

		cosVal = Math.cos(x);
		sinVal = Math.sin(x);
		double[][] rotXArr = { { 1, 0, 0 }, { 0, cosVal, -sinVal }, { 0, sinVal, cosVal } };
		Matrix rotMX = new Matrix(rotXArr);

		cosVal = Math.cos(y);
		sinVal = Math.sin(y);
		double[][] rotYArr = { { cosVal, 0, sinVal }, { 0, 1, 0 }, { -sinVal, 0, cosVal } };
		Matrix rotMY = new Matrix(rotYArr);

		Matrix rotM = rotMZ.times(rotMX).times(rotMY);

		return rotM;
	}
}
