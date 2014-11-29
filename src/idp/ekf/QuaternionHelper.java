package idp.ekf;

import Jama.Matrix;

public class QuaternionHelper {

	public static Matrix dq3_by_dq1(Quaternion q) {
		double[][] arr = { { q.getR(), -q.getX(), -q.getY(), -q.getZ() }, 
				{ q.getX(), q.getR(), -q.getZ(), q.getY() },
				{ q.getY(), q.getZ(), q.getR(), -q.getX() }, 
				{ q.getZ(), -q.getY(), q.getX(), q.getR() } };

		return new Matrix(arr);
	}

	public static Matrix dq3_by_dq2(Quaternion q) {
		double[][] arr = { 	{ q.getR(), -q.getX(), -q.getY(), -q.getZ() }, 
							{ q.getX(), q.getR(), q.getZ(), -q.getY() },
							{ q.getY(), -q.getZ(), q.getR(), q.getX() }, 
							{ q.getZ(), q.getY(), -q.getX(), q.getR() } };

		return new Matrix(arr);
	}

	public static Quaternion calculateQWT(PointTriple w, double deltaTime) {
		// VW::Quaternion qwt(omegaold * delta_t);
		w = w.times(deltaTime);

		double theta = w.getNorm();
		if (theta < Helper.EPS)
			return new Quaternion(0, 0, 0, 1);
		else {
			PointTriple w_n = w.divide(theta);
			return new Quaternion(w_n, theta);
		}
	}

	public static Matrix dqomegadt_by_domega(PointTriple omega, double deltaTime) {
		// Modulus
		double omegamod = Math.sqrt(omega.getX() * omega.getX() + omega.getY() * omega.getY() + omega.getZ()
				* omega.getZ());

		Matrix dqomegadt_by_domega = new Matrix(4, 3);

		// Use generic ancillary functions to calculate components of Jacobian
		dqomegadt_by_domega.set(0, 0, dq0_by_domegaA(omega.getX(), omegamod, deltaTime));
		dqomegadt_by_domega.set(0, 1, dq0_by_domegaA(omega.getY(), omegamod, deltaTime));
		dqomegadt_by_domega.set(0, 2, dq0_by_domegaA(omega.getZ(), omegamod, deltaTime));

		dqomegadt_by_domega.set(1, 0, dqA_by_domegaA(omega.getX(), omegamod, deltaTime));
		dqomegadt_by_domega.set(1, 1, dqA_by_domegaB(omega.getX(), omega.getY(), omegamod, deltaTime));
		dqomegadt_by_domega.set(1, 2, dqA_by_domegaB(omega.getX(), omega.getZ(), omegamod, deltaTime));

		dqomegadt_by_domega.set(2, 0, dqA_by_domegaB(omega.getY(), omega.getX(), omegamod, deltaTime));
		dqomegadt_by_domega.set(2, 1, dqA_by_domegaA(omega.getY(), omegamod, deltaTime));
		dqomegadt_by_domega.set(2, 2, dqA_by_domegaB(omega.getY(), omega.getZ(), omegamod, deltaTime));

		dqomegadt_by_domega.set(3, 0, dqA_by_domegaB(omega.getZ(), omega.getX(), omegamod, deltaTime));
		dqomegadt_by_domega.set(3, 1, dqA_by_domegaB(omega.getZ(), omega.getY(), omegamod, deltaTime));
		dqomegadt_by_domega.set(3, 2, dqA_by_domegaA(omega.getZ(), omegamod, deltaTime));
		
		return dqomegadt_by_domega;
	}

	private static double dq0_by_domegaA(double omegaA, double omega, double deltaTime) {
		return (-deltaTime / 2.0) * (omegaA / omega) * Math.sin(omega * deltaTime / 2.0);
	}

	private static double dqA_by_domegaA(double omegaA, double omega, double deltaTime) {
		return (deltaTime / 2.0) * omegaA * omegaA / (omega * omega) * Math.cos(omega * deltaTime / 2.0)
				+ (1.0 / omega) * (1.0 - omegaA * omegaA / (omega * omega)) * Math.sin(omega * deltaTime / 2.0);
	}

	private static double dqA_by_domegaB(double omegaA, double omegaB, double omega, double deltaTime) {
		return (omegaA * omegaB / (omega * omega))
				* ((deltaTime / 2.0) * Math.cos(omega * deltaTime / 2.0) - (1.0 / omega)
						* Math.sin(omega * deltaTime / 2.0));
	}
	
	public static Matrix qconj(Matrix q) {
		double[][] out = new double[4][1];
		
		out[0][0] = q.get(0, 0);
		out[1][0] = -q.get(1, 0);
		out[2][0] = -q.get(2, 0);
		out[3][0] = -q.get(3, 0);
		
		return new Matrix(out);
	}
	
	public static Matrix normJac(Quaternion q) {
		double r = q.getR();
		double x = q.getX();
		double y = q.getY();
		double z = q.getZ();
		
		double c = Math.pow(r*r + x*x + y*y + z*z, -1.5);
		
		double[][] mat = {	{x*x+y*y+z*z, -r*x, -r*y, -r*z},
							{-x*r, r*r+y*y+z*z, -x*y, -x*z},
							{-y*r, -y*x, r*r+x*x+z*z, -y*z},
							{-z*r, -z*x, -z*y, r*r+x*x+y*y}};
		
		return new Matrix(mat).times(c);
	}
	
	public static Quaternion normalize(Quaternion q) {
		double r = q.getR();
		double x = q.getX();
		double y = q.getY();
		double z = q.getZ();
		
		double d = Math.pow(r*r+x*x+y*y+z*z, 0.5);
		
		return new Quaternion(x/d,y/d,z/d,r/d);
	}
}
