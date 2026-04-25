package org.firstinspires.ftc.teamcode.pedroPathing.control;

/**
 * This is the KalmanFilterParameters class. This class handles holding parameters Kalman filters.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @version 1.0, 7/17/2024
 */
public class KalmanFilterParameters {
    public double Q;
    public double R;

    /**
     * This creates a new KalmanFilterParameters with a specified model and data covariance.
     *
     * @param modelCovariance the covariance of the model.
     * @param dataCovariance the covariance of the data.
     */
    public KalmanFilterParameters(double modelCovariance, double dataCovariance) {
        this.Q = modelCovariance;
        this.R = dataCovariance;
    }
}
