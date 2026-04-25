package org.firstinspires.ftc.teamcode.util;
public class KalmanFilter2d {

    // State: [position, velocity]
    private double[] x = new double[2];

    // Covariance matrix P (2x2)
    private double[][] P = new double[2][2];

    // System matrices
    private double[][] A = new double[2][2];
    private double[][] B = new double[2][1];
    private double[][] C = new double[1][2];

    // Noise matrices
    private double[][] Q = new double[2][2]; // process noise
    private double[][] R = new double[1][1]; // measurement noise

    private double dt;

    public KalmanFilter2d(double dt) {
        this.dt = dt;

        // A matrix
        A[0][0] = 1;  A[0][1] = dt;
        A[1][0] = 0;  A[1][1] = 1;

        // B matrix (input affects velocity)
        B[0][0] = 0;
        B[1][0] = dt;

        // C matrix (we measure position only)
        C[0][0] = 1;
        C[0][1] = 0;

        reset();
        setDefaultNoise();
    }

    public void reset() {
        x[0] = 0;
        x[1] = 0;

        P[0][0] = 1; P[0][1] = 0;
        P[1][0] = 0; P[1][1] = 1;
    }

    public void setDefaultNoise() {
        // Process noise
        Q[0][0] = 1e-4; Q[0][1] = 0;
        Q[1][0] = 0;    Q[1][1] = 1e-2;

        // Measurement noise
        R[0][0] = 1e-3;
    }

    public void setProcessNoise(double posNoise, double velNoise) {
        Q[0][0] = posNoise;
        Q[1][1] = velNoise;
    }

    public void setMeasurementNoise(double measNoise) {
        R[0][0] = measNoise;
    }

    /**
     * Update step
     * @param u input (motor command, approx acceleration)
     * @param z measured position (encoder)
     */
    public void update(double u, double z) {

        // === PREDICT ===

        // x = A*x + B*u
        double x0 = A[0][0]*x[0] + A[0][1]*x[1] + B[0][0]*u;
        double x1 = A[1][0]*x[0] + A[1][1]*x[1] + B[1][0]*u;
        x[0] = x0;
        x[1] = x1;

        // P = A*P*A^T + Q
        double[][] P_pred = new double[2][2];

        P_pred[0][0] = A[0][0]*P[0][0] + A[0][1]*P[1][0];
        P_pred[0][1] = A[0][0]*P[0][1] + A[0][1]*P[1][1];
        P_pred[1][0] = A[1][0]*P[0][0] + A[1][1]*P[1][0];
        P_pred[1][1] = A[1][0]*P[0][1] + A[1][1]*P[1][1];

        double[][] P_new = new double[2][2];

        P_new[0][0] = P_pred[0][0]*A[0][0] + P_pred[0][1]*A[0][1] + Q[0][0];
        P_new[0][1] = P_pred[0][0]*A[1][0] + P_pred[0][1]*A[1][1] + Q[0][1];
        P_new[1][0] = P_pred[1][0]*A[0][0] + P_pred[1][1]*A[0][1] + Q[1][0];
        P_new[1][1] = P_pred[1][0]*A[1][0] + P_pred[1][1]*A[1][1] + Q[1][1];

        P = P_new;

        // === UPDATE ===

        // y = z - C*x
        double y = z - (C[0][0]*x[0] + C[0][1]*x[1]);

        // S = C*P*C^T + R (scalar)
        double S = P[0][0] + R[0][0];

        // K = P*C^T / S
        double k0 = P[0][0] / S;
        double k1 = P[1][0] / S;

        // x = x + K*y
        x[0] += k0 * y;
        x[1] += k1 * y;

        // P = (I - K*C)*P
        double p00 = (1 - k0)*P[0][0];
        double p01 = (1 - k0)*P[0][1];
        double p10 = -k1*P[0][0] + P[1][0];
        double p11 = -k1*P[0][1] + P[1][1];

        P[0][0] = p00;
        P[0][1] = p01;
        P[1][0] = p10;
        P[1][1] = p11;
    }

    public double getPosition() {
        return x[0];
    }

    public double getVelocity() {
        return x[1];
    }
}