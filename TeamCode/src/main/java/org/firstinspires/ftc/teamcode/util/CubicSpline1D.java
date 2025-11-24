package org.firstinspires.ftc.teamcode.util;

public class CubicSpline1D {

    // Class to hold coefficients of one cubic segment
    public static class CubicSegment {
        public double a0, a1, a2, a3;
        public double t0, t1;

        public CubicSegment(double a0, double a1, double a2, double a3, double t0, double t1) {
            this.a0 = a0;
            this.a1 = a1;
            this.a2 = a2;
            this.a3 = a3;
            this.t0 = t0;
            this.t1 = t1;
        }

        // Evaluate position at time t
        public double evaluate(double t) {
            double dt = t - t0;
            return a0 + a1 * dt + a2 * dt * dt + a3 * dt * dt * dt;
        }

        // Evaluate velocity at time t
        public double velocity(double t) {
            double dt = t - t0;
            return a1 + 2 * a2 * dt + 3 * a3 * dt * dt;
        }

        public double acceleration(double t) {
            double dt = t - t0;
            return 2 * a2 + 6 * a3 * dt;
        }
    }

    private CubicSegment[] segments;

    /**
     * Constructor to generate cubic spline.
     * @param t times of points (length n+1)
     * @param x positions (length n+1)
     * @param v velocities (length n+1)
     */
    public CubicSpline1D(double[] t, double[] x, double[] v) {
        int n = t.length - 1;  // number of segments
        segments = new CubicSegment[n];

        for (int i = 0; i < n; i++) {
            double t0 = t[i];
            double t1 = t[i + 1];
            double x0 = x[i];
            double x1 = x[i + 1];
            double v0 = v[i];
            double v1 = v[i + 1];
            double dt = t1 - t0;

            // Solve for coefficients using boundary conditions
            double a0 = x0;
            double a1 = v0;
            double a2 = (3*(x1 - x0)/dt - 2*v0 - v1) / dt;
            double a3 = (2*(x0 - x1)/dt + v0 + v1) / (dt*dt);

            segments[i] = new CubicSegment(a0, a1, a2, a3, t0, t1);
        }
    }

    // Evaluate the spline at time t
    public double evaluate(double t) {
        CubicSegment s = segment(t);
        if (s != null){ return s.evaluate(t); }
        // If t is outside the range, return nearest endpoint
        if (t < segments[0].t0) return segments[0].evaluate(segments[0].t0);
        return segments[segments.length - 1].evaluate(segments[segments.length - 1].t1);
    }

    // Evaluate velocity at time t
    public double velocity(double t) {
        CubicSegment s = segment(t);
        if (s != null){ return s.velocity(t); }

        if (t < segments[0].t0) return segments[0].velocity(segments[0].t0);
        return segments[segments.length - 1].velocity(segments[segments.length - 1].t1);
    }

    // Evaluate acceleration at time t
    public double acceleration(double t) {
        CubicSegment s = segment(t);
        if (s != null){ return s.velocity(t); }

        if (t < segments[0].t0) return segments[0].acceleration(segments[0].t0);
        return segments[segments.length - 1].acceleration(segments[segments.length - 1].t1);
    }

    public CubicSegment segment(double t){
        for (CubicSegment s : segments) {
            if (t >= s.t0 && t <= s.t1) {
                return s;
            }
        }
        return null;
    }

    // For debugging: print all segment coefficients
    public void printCoefficients() {
        for (int i = 0; i < segments.length; i++) {
            CubicSegment s = segments[i];
            System.out.printf("Segment %d: a0=%.4f, a1=%.4f, a2=%.4f, a3=%.4f\n",
                    i, s.a0, s.a1, s.a2, s.a3);
        }
    }
}