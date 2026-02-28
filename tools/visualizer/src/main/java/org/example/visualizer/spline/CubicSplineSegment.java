package org.example.visualizer.spline;

@SuppressWarnings("unused")
public class CubicSplineSegment {
    private final CubicPolynomial xPolynomial, yPolynomial, rotPolynomial;
    private final double startTime, endTime;

    /**
     * Construct a spline segment between startPoint and endPoint using Catmull-Rom interpolation.
     */
    public CubicSplineSegment(Waypoint lastPoint, Waypoint startPoint, Waypoint endPoint, Waypoint nextPoint, double startTime, double robotTopSpeed) {
        this.startTime = Math.abs(startTime);
        robotTopSpeed = Math.abs(robotTopSpeed);

        double speedRatio = endPoint.getSpeed();
        double segmentSpeed = speedRatio * robotTopSpeed;
        segmentSpeed = segmentSpeed > 0.0 ? segmentSpeed : 1.0;

        // Build Catmull-Rom cubic coefficients for x, y, and rotation
        double[] xCoeffs = catmullRomCoeffs(lastPoint.getX(), startPoint.getX(), endPoint.getX(), nextPoint.getX());
        double[] yCoeffs = catmullRomCoeffs(lastPoint.getY(), startPoint.getY(), endPoint.getY(), nextPoint.getY());
        double[] rCoeffs = catmullRomCoeffs(lastPoint.getAngle(), startPoint.getAngle(), endPoint.getAngle(), nextPoint.getAngle());

        CubicPolynomial xp = new CubicPolynomial(xCoeffs[0], xCoeffs[1], xCoeffs[2], xCoeffs[3]);
        CubicPolynomial yp = new CubicPolynomial(yCoeffs[0], yCoeffs[1], yCoeffs[2], yCoeffs[3]);
        CubicPolynomial rp = new CubicPolynomial(rCoeffs[0], rCoeffs[1], rCoeffs[2], rCoeffs[3]);

        // Estimate segment length by sampling the curve in parameter space [0,1]
        double length = estimateArcLength(xp, yp, 100);
        length = length > 0.0 ? length : 1.0;
        double segmentTime = length / segmentSpeed;

        xp.setTimeScalar(segmentTime);
        yp.setTimeScalar(segmentTime);
        rp.setTimeScalar(segmentTime);

        this.xPolynomial = xp;
        this.yPolynomial = yp;
        this.rotPolynomial = rp;

        this.endTime = this.startTime + segmentTime;
    }

    private double[] catmullRomCoeffs(double P0, double P1, double P2, double P3) {
        // Coeffs such that value(t) = a*t^3 + b*t^2 + c*t + d for t in [0,1]
        double d = P1;
        double c = 0.5 * (P2 - P0);
        double b = 0.5 * (2*P0 - 5*P1 + 4*P2 - P3);
        double a = 0.5 * (-P0 + 3*P1 - 3*P2 + P3);
        return new double[]{a, b, c, d};
    }

    private double estimateArcLength(CubicPolynomial xp, CubicPolynomial yp, int samples) {
        samples = Math.max(2, samples);
        double prevX = xp.compute(0.0);
        double prevY = yp.compute(0.0);
        double len = 0.0;
        for (int i = 1; i < samples; i++) {
            double t = (double) i / (samples - 1);
            double cx = xp.compute(t);
            double cy = yp.compute(t);
            double dx = cx - prevX;
            double dy = cy - prevY;
            len += Math.hypot(dx, dy);
            prevX = cx;
            prevY = cy;
        }
        return len;
    }

    public double getX(double time){
        return xPolynomial.compute(time - startTime);
    }

    public double getY(double time){
        return yPolynomial.compute(time - startTime);
    }

    public double getRotation(double time){
        return rotPolynomial.compute(time - startTime);
    }

    /**
     * Return instantaneous scalar speed (units per second) at absolute time `time` along this segment.
     */
    public double getSpeed(double time) {
        double dxdt = xPolynomial.derivative(time - startTime);
        double dydt = yPolynomial.derivative(time - startTime);
        return Math.hypot(dxdt, dydt);
    }

    public double endTime(){
        return this.endTime;
    }
}

class CubicPolynomial {
    private final double a, b, c, d; // coefficients for t^3, t^2, t, 1
    private double timeScalar = 1.0; // maps absolute time to 0..1 parameter by input/timeScalar

    public CubicPolynomial(double a, double b, double c, double d) {
        this.a = a;
        this.b = b;
        this.c = c;
        this.d = d;
    }

    public void setTimeScalar(double segmentDuration) {
        this.timeScalar = segmentDuration > 0 ? segmentDuration : 1.0;
    }

    /**
     * compute expects an absolute time (seconds) relative to the segment start; it divides by timeScalar to get t in [0,1]
     */
    public double compute(double input) {
        double t = input / timeScalar; // normalized
        // Clamp t to [0,1] to avoid extrapolation for safety
        if (t < 0) t = 0;
        if (t > 1) t = 1;
        return a * t * t * t + b * t * t + c * t + d;
    }

    /**
     * derivative with respect to absolute time: d/dt value(t)
     */
    public double derivative(double input) {
        double t = input / timeScalar;
        // clamp within [0,1]
        if (t < 0) t = 0;
        if (t > 1) t = 1;
        // derivative w.r.t normalized parameter tau is: 3a*t^2 + 2b*t + c
        double d_dtau = 3.0 * a * t * t + 2.0 * b * t + c;
        // d(tau)/dt = 1/timeScalar
        return d_dtau * (1.0 / timeScalar);
    }
}