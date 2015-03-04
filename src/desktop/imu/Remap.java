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

	 public static void main(String args[]) {
	 double init[] = { 0, 0, 0 };
	
	 Mat refMat = convertToRotMat(init);
	
	 System.out.println(refMat.dump());
	 
	 double samp[] = { 30, 0, 0 };
	
	 double res[] = remapRotVec(samp, refMat);
	
	 System.out.println(res[0]);
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

	public static Mat convertToRotMat(double rotVec[]) {
		double z = Math.toRadians(mod360(rotVec[0]));
		double x = Math.toRadians(mod360(rotVec[1]));
		double y = Math.toRadians(mod360(rotVec[2]));

		double cosVal, sinVal;
		Size size = new Size(3, 3);
		int type = CvType.CV_64F;

		cosVal = Math.cos(z);
		sinVal = Math.sin(z);
		double[][] rotZArr = { { cosVal, -sinVal, 0 }, { sinVal, cosVal, 0 }, { 0, 0, 1 } };
		Mat rotMZ = Mat.eye(size, type);
		rotMZ.put(0, 0, rotZArr[0]);
		rotMZ.put(1, 0, rotZArr[1]);
		rotMZ.put(2, 0, rotZArr[2]);

		cosVal = Math.cos(x);
		sinVal = Math.sin(x);
		double[][] rotXArr = { { 1, 0, 0 }, { 0, cosVal, -sinVal }, { 0, sinVal, cosVal } };
		Mat rotMX = Mat.eye(size, type);
		rotMX.put(0, 0, rotXArr[0]);
		rotMX.put(1, 0, rotXArr[1]);
		rotMX.put(2, 0, rotXArr[2]);

		cosVal = Math.cos(y);
		sinVal = Math.sin(y);
		double[][] rotYArr = { { cosVal, 0, sinVal }, { 0, 1, 0 }, { -sinVal, 0, cosVal } };
		Mat rotMY = Mat.eye(size, type);
		rotMY.put(0, 0, rotYArr[0]);
		rotMY.put(1, 0, rotYArr[1]);
		rotMY.put(2, 0, rotYArr[2]);

		Mat temp1 = new Mat(), rotM = new Mat(), nullMat = new Mat();

		Core.gemm(rotMZ, rotMX, 1, nullMat, 0, temp1);
		Core.gemm(temp1, rotMY, 1, nullMat, 0, rotM);

		return rotM;
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
