package geometry;

/**
 * A generic class for matrix mathematics.
 *
 * @author DrPixelCat - 7842 alum
 * @author Sohum Arora - 22985 Paraducks
 */
public class Matrix {
    private final double[][] data;
    private final int rows;
    private final int cols;

    /**
     * Constructs a new Matrix from a 2D double array.
     *
     * @param data The 2D array representing the matrix [rows][columns]
     */
    public Matrix(double[][] data) {
        this.rows = data.length;
        this.cols = data[0].length;
        this.data = new double[rows][cols];

        // Deep copy to ensure the matrix is immutable
        for (int i = 0; i < rows; i++) {
            System.arraycopy(data[i], 0, this.data[i], 0, cols);
        }
    }

    /**
     * Multiplies this matrix by a 1D column vector.
     *
     * @param vector The 1D array representing the column vector
     * @return A new 1D array containing the result
     */
    public double[] multiply(double[] vector) {
        if (vector.length != this.cols) {
            throw new IllegalArgumentException(
                    "Matrix columns (" + this.cols + ") must match vector length (" + vector.length + ")."
            );
        }

        double[] result = new double[this.rows];
        for (int i = 0; i < this.rows; i++) {
            double sum = 0.0;
            for (int j = 0; j < this.cols; j++) {
                sum += this.data[i][j] * vector[j];
            }
            result[i] = sum;
        }
        return result;
    }

    /** Get a value from the matrix. */
    public double get(int row, int col) {
        if (row < 0 || row > rows || col < 0 || col > cols) {
            throw new IllegalArgumentException("Index out of bounds!");
        }
        return data[row][col];
    }

    public int getRows() { return rows; }

    public int getCols() { return cols; }
}
