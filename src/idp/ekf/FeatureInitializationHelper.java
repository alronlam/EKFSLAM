package idp.ekf;

import Jama.Matrix;

public class FeatureInitializationHelper {

	public static IDPFeature createFeature(PointTriple xyzPosition, Quaternion quaternion, int u, int v,
			double initialRho, Camera cam) {

		double x = xyzPosition.getX();
		double y = xyzPosition.getY();
		double z = xyzPosition.getZ();
		
		Matrix uv = IDPUtility.undistort_fm(u, v, cam);
		double ud = uv.get(0, 0);
		double vd = uv.get(1, 0);
		
		double h_LRx = (ud-cam.K[0][2])/cam.K[0][0];
		double h_LRy = (vd-cam.K[1][2])/cam.K[1][1];
		
		double[][] mat = {	{h_LRx},
							{h_LRy},
							{1}};
		
		Matrix h_LR = new Matrix(mat);
		
		Matrix rotationMatrix = Helper.quaternionToRotationMatrix(quaternion).times(h_LR);
		double nx = rotationMatrix.get(0, 0);
		double ny = rotationMatrix.get(1, 0);
		double nz = rotationMatrix.get(2, 0);

		double azimuth = Math.atan2(nx, nz);
		double elevation = Math.atan2(-ny, Math.sqrt(nx * nx + nz * nz));

		return new IDPFeature(x, y, z, azimuth, elevation, initialRho);
	}

}
