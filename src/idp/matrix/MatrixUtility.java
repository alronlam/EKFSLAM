package idp.matrix;

import Jama.Matrix;

public class MatrixUtility {
	public static String matrixBounds(Matrix m) {
		return m.getRowDimension() + " by " + m.getColumnDimension();
	}
	
	public static void printMatrix(Matrix m) {
		m.print(0, 0);
	}
}
