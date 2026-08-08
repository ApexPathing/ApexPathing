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
                    "Matrix cols (" + cols + ") must match vector length (" + vector.length + ")."
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

    /**
     * Multiplies this matrix by another Matrix.
     */
    public Matrix multiply(Matrix other) {
        if (this.cols != other.rows) {
            throw new IllegalArgumentException("Dimension mismatch for multiplication.");
        }
        double[][] result = new double[this.rows][other.cols];
        for (int i = 0; i < this.rows; i++) {
            for (int j = 0; j < other.cols; j++) {
                for (int k = 0; k < this.cols; k++) {
                    result[i][j] += this.data[i][k] * other.get(k, j);
                }
            }
        }
        return new Matrix(result);
    }

    /**
     * Multiplies this matrix by a scalar value.
     */
    public Matrix multiply(double scalar) {
        double[][] result = new double[rows][cols];
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                result[i][j] = this.data[i][j] * scalar;
            }
        }
        return new Matrix(result);
    }

    /**
     * Adds another Matrix to this matrix.
     */
    public Matrix add(Matrix other) {
        if (this.rows != other.rows || this.cols != other.cols) {
            throw new IllegalArgumentException("Dimension mismatch for addition.");
        }
        double[][] result = new double[rows][cols];
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                result[i][j] = this.data[i][j] + other.get(i, j);
            }
        }
        return new Matrix(result);
    }

    /**
     * Subtracts another Matrix from this matrix.
     */
    public Matrix subtract(Matrix other) {
        if (this.rows != other.rows || this.cols != other.cols) {
            throw new IllegalArgumentException("Dimension mismatch for subtraction.");
        }
        double[][] result = new double[rows][cols];
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                result[i][j] = this.data[i][j] - other.get(i, j);
            }
        }
        return new Matrix(result);
    }

    /**
     * Transposes this matrix (swaps rows and columns).
     */
    public Matrix transpose() {
        double[][] result = new double[cols][rows];
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                result[j][i] = this.data[i][j];
            }
        }
        return new Matrix(result);
    }

    /**
     * Calculates the inverse of this matrix.
     */
    public Matrix inverse() {
        if (rows != cols) {
            throw new UnsupportedOperationException("Inverse requires a square matrix.");
        }
        if (rows == 1) {
            return new Matrix(new double[][]{{1.0 / data[0][0]}});
        } else if (rows == 2) {
            double det = data[0][0] * data[1][1] - data[0][1] * data[1][0];
            return new Matrix(new double[][]{
                    {data[1][1] / det, -data[0][1] / det},
                    {-data[1][0] / det, data[0][0] / det}
            });
        }
        throw new UnsupportedOperationException("Inverse only supported for 1x1 and 2x2 in this lightweight class.");
    }

    /**
     * Generates an identity matrix of the specified size.
     */
    public static Matrix identity(int size) {
        double[][] result = new double[size][size];
        for (int i = 0; i < size; i++) {
            result[i][i] = 1.0;
        }
        return new Matrix(result);
    }

    /** Get a value from the matrix. */
    public double get(int row, int col) {
        return data[row][col];
    }
}