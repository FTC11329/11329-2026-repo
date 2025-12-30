package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.teamcode.pedroPathing.math.MathFunctions.clamp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.control.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

public class SuperDuperPID {
        private PIDFCoefficients coefficients;
        private double previousPosition;
        private double error;
        private double position;
        private double targetPosition;
        private double errorIntegral;
        private double errorDerivative;
        private double feedForwardInput;

        private long previousUpdateTimeNano;
        private long deltaTimeNano;
        private boolean integralFreeze;
        private boolean stuck;
        private Timer stuckTime = new Timer();

    public SuperDuperPID(PIDFCoefficients set) {
            setCoefficients(set);
            reset();
        }

        public double run() {
            double pidOutput = error * P() + errorDerivative * D() + errorIntegral * I() + feedForwardInput * F() * Math.signum(error);

            // adds a boost to break from the wheel
            double measuredVelocity = (position - previousPosition) / (deltaTimeNano / 1e9);
            //todo: tune these constants
            double stuckThreshold = 32; //ticks/second
            double errorThreshold = Constants.Indexer.indexerTolerance;
            double breakawayBoost = 0.2;

            stuck = (Math.abs(error) > 0.01) && (Math.abs(measuredVelocity) < stuckThreshold);

            if (stuck && error > errorThreshold && false) {
                if (stuckTime.getElapsedTimeSeconds() < .1) {
                    pidOutput += breakawayBoost * Math.signum(error);
                    integralFreeze = true;
                } else {
                    if (stuckTime.getElapsedTimeSeconds() < .15) {
                        pidOutput = -pidOutput;
                    } else {
                        stuckTime.resetTimer();
                    }
                }
            } else {
                integralFreeze = false;
                stuckTime.resetTimer();
            }

            return pidOutput;
        }
        public void updateCurrentPosition(double position) {
            previousPosition = this.position;
            this.position = position;
            error = targetPosition - this.position;

            long systemTime = System.nanoTime();
            deltaTimeNano = systemTime - previousUpdateTimeNano;
            previousUpdateTimeNano = systemTime;

            if (!integralFreeze) {errorIntegral += error * (deltaTimeNano * 1e-9);}
            errorDerivative = - (position - previousPosition) / (deltaTimeNano * 1e-9);
        }

        public void updateFeedForwardInput(double input) {
            feedForwardInput = input;
        }

        /**
         * This resets all the PIDF's error and position values, as well as the time stamps.
         */
        public void reset() {
            error = 0;
            position = 0;
            targetPosition = 0;
            errorIntegral = 0;
            errorDerivative = 0;
            previousUpdateTimeNano = System.nanoTime();
        }

        /**
         * This is used to set the target position if the PIDF is being run with current position and
         * target position inputs rather than error inputs.
         *
         * This also resets the I and D terms so that we avoid integral windup and derivative kick.
         * @param set this sets the target position.
         */
        public void setTargetPosition(double set) {
            targetPosition = set;
            errorIntegral = 0;
        }
        public double getTargetPosition() {
            return targetPosition;
        }

        public void setCoefficients(PIDFCoefficients set) {
            coefficients = set;
        }
        public PIDFCoefficients getCoefficients() {
            return coefficients;
        }

        public void setP(double set) {
            coefficients.P = set;
        }
        public double P() {
            return coefficients.P;
        }

        public void setI(double set) {
            coefficients.I = set;
        }

        public double I() {
            return coefficients.I;
        }

        public void setD(double set) {
            coefficients.D = set;
        }

        public double D() {
            return coefficients.D;
        }

        public void setF(double set) {
            coefficients.F = set;
        }

        public double F() {
            return coefficients.F;
        }

        public double getError() {
            return error;
        }
        public double getErrorDerivative() {
            return errorDerivative;
        }
        public double getTargetTicks() {
            return targetPosition;
        }
        public double getITerm() {return I() * errorIntegral * error;}
        public double getPTerm() {return P() * error;}
        public double getDTerm() {return  D() * error;}
        public boolean getStuck() {return stuck;}
        public double getDeltaTime() {return deltaTimeNano * 1e-6;}
}
