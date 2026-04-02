/// Copyright (c) 2026 Tobin Rumsey, all rights reserved
/// Comment and debug assist by Claude, Anthropic
/// Prototype for class and python implementation made in Google Colab, Formatting Help by Google Gemini. EJML Matrix library used because I needed Matrix's :)
/// Theory behind concept from Catmull-Rom splines and general cubic spline interpolation, as well as time-based path following algorithms commonly used in robotics.
/// Permitted use of class for FIRST Tech Challenge team 21430, BroomBots until the end of the 2029-2030 season,
/// with the expectation that the next generation of students will have to learn how to do this themselves by then.
/// All other FTC teams and non-FTC users are permitted to use this code for educational and or competition purposes, but are encouraged to learn how to implement cubic spline interpolation and path following algorithms themselves for a deeper understanding of the underlying mathematics and robotics concepts.
/// This code is provided as-is for educational purposes, and is not guaranteed to be bug-free or suitable for all use cases. It is the responsibility of the user to test and validate the code for their specific application,
///  and to understand the underlying mathematics and algorithms involved in cubic spline interpolation and path following.
/// Thank you for respecting the intellectual property and educational intent of this code! <3 - Tobin Rumsey, BroomBots 21430
package org.example.visualizer.spline;

import org.ejml.simple.SimpleMatrix;

/**
 * CubicSplineSegment represents a single segment of a cubic spline path, defined by cubic polynomials for x, y, and rotation (angle) as functions of time along the segment.
 * Each segment is constructed using Catmull-Rom interpolation based on 4 control points (the previous waypoint, the start waypoint, the end waypoint, and the next waypoint)
 * to ensure smooth transitions between segments.
 * The segment also calculates its duration based on the distance between the start and end points and the desired speed defined in the end waypoint,
 * allowing for time-based evaluation of the spline.
 * The compute() method allows for evaluating the position and rotation at any given absolute time along the segment, making it compatible with path following
 * algorithms that operate in real time and need to know the target position and orientation at any given time.
 * This class is designed to be used as part of a larger path following system, where multiple CubicSplineSegments are chained together to form a complete path.
 * The class also includes an alternate constructor for creating "hold" segments, which keep the robot at a fixed position and orientation for a specified duration,
 * allowing for easy implementation of wait times at waypoints without needing special handling in the path following logic.
 */
public class CubicSplineSegment {
    private final CubicPolynomial xPolynomial, yPolynomial, rotPolynomial;
    private final boolean constantAngle;

    private final double startTime, endTime;

    /**
     * One pathing segment, contains three cubic polynomials. Input four waypoints that define the path in terms of time.
     * If one of these points does not exist due to a stop or the start or end of the path, provide a duplicate of either the start or end point.
     * start and end points are REQUIRED to be unique.
     *
     * @param lastPoint   the start point of the previous spline
     * @param startPoint  where this spline starts
     * @param endPoint    where this spline ends
     * @param nextPoint   where the next spline ends
     * @param startTime   absolute time at which this segment starts (seconds)
     * @param robotTopSpeed maximum speed of the robot (units/second)
     * @param constantAngle if true, interpolate the rotation polynomial; if false, derive heading from direction of travel
     */
    public CubicSplineSegment(Waypoint lastPoint, Waypoint startPoint, Waypoint endPoint, Waypoint nextPoint,
                               double startTime, double robotTopSpeed, boolean constantAngle) {
        this.constantAngle = constantAngle;
        this.startTime = Math.abs(startTime);
        robotTopSpeed = Math.abs(robotTopSpeed);

        double speedRatio = endPoint.getSpeed();
        double segmentSpeed = speedRatio * robotTopSpeed;
        segmentSpeed = segmentSpeed > 0.0 ? segmentSpeed : 1.0;

        // Build uniform Catmull-Rom coefficients per dimension independently.
        // This matches the original working implementation: each axis uses the classic
        // alpha=0 formula operating on scalar values, with no cross-axis knot distances.
        SimpleMatrix xCoeffs = catmullRomCoeffs(
                lastPoint.getX(), startPoint.getX(), endPoint.getX(), nextPoint.getX());
        SimpleMatrix yCoeffs = catmullRomCoeffs(
                lastPoint.getY(), startPoint.getY(), endPoint.getY(), nextPoint.getY());
        SimpleMatrix rCoeffs = catmullRomCoeffs(
                lastPoint.getAngle(), startPoint.getAngle(), endPoint.getAngle(), nextPoint.getAngle());

        CubicPolynomial xp = new CubicPolynomial(xCoeffs);
        CubicPolynomial yp = new CubicPolynomial(yCoeffs);
        CubicPolynomial rp = new CubicPolynomial(rCoeffs);

        // Estimate arc length by sampling the raw [0,1] curve before applying the time scalar
        double length = getArcLength(xp, yp, 100);
        length = length > 0.0 ? length : 1.0;
        double segmentTime = length / segmentSpeed;

        // Map the [0,1] polynomial parameter to real segment duration
        xp.setTimeScalar(segmentTime);
        yp.setTimeScalar(segmentTime);
        rp.setTimeScalar(segmentTime);

        this.xPolynomial   = xp;
        this.yPolynomial   = yp;
        this.rotPolynomial = rp;
        this.endTime = this.startTime + segmentTime;
    }

    /**
     * Alternate constructor for a "hold" segment that keeps the robot at a fixed position and orientation for a specified duration.
     *
     * @param holdPoint the point to hold at
     * @param startTime the absolute time at which this hold segment starts (seconds)
     * @param duration  the duration to hold at the point (seconds)
     */
    public CubicSplineSegment(Waypoint holdPoint, double startTime, double duration) {
        this.constantAngle = true;
        this.startTime = startTime;
        this.endTime   = startTime + duration;

        // Constant polynomials: only the d term is non-zero, so compute() always returns the hold value
        this.xPolynomial   = new CubicPolynomial(new SimpleMatrix(new double[]{0, 0, 0, holdPoint.getX()}));
        this.yPolynomial   = new CubicPolynomial(new SimpleMatrix(new double[]{0, 0, 0, holdPoint.getY()}));
        this.rotPolynomial = new CubicPolynomial(new SimpleMatrix(new double[]{0, 0, 0, holdPoint.getAngle()}));

        this.xPolynomial.setTimeScalar(duration);
        this.yPolynomial.setTimeScalar(duration);
        this.rotPolynomial.setTimeScalar(duration);
    }

    /**
     * Compute uniform Catmull-Rom coefficients [a, b, c, d] for a single scalar dimension,
     * returned as a 1x4 SimpleMatrix so CubicPolynomial can unpack them directly.
     * Uses the standard alpha=0 (uniform) formula operating on scalar values independently —
     * no cross-axis distances involved, which keeps each dimension clean.
     *
     * @param P0 value at the point before the segment start (for tangent)
     * @param P1 value at the segment start
     * @param P2 value at the segment end
     * @param P3 value at the point after the segment end (for tangent)
     * @return 1x4 SimpleMatrix {a, b, c, d} such that f(t) = a*t^3 + b*t^2 + c*t + d for t in [0,1]
     */
    private SimpleMatrix catmullRomCoeffs(double P0, double P1, double P2, double P3) {
        double d = P1;
        double c = 0.5 * (P2 - P0);
        double b = 0.5 * (2*P0 - 5*P1 + 4*P2 - P3);
        double a = 0.5 * (-P0 + 3*P1 - 3*P2 + P3);
        return new SimpleMatrix(new double[][]{{a, b, c, d}});
    }

    /**
     * Estimate arc length of the parametric curve (xPoly, yPoly) over t in [0,1] by summing
     * chord distances between evenly-spaced sample points. Called before setTimeScalar so the
     * polynomials are still in their raw [0,1] domain.
     */
    private double getArcLength(CubicPolynomial xPoly, CubicPolynomial yPoly, int samples) {
        samples = Math.max(2, samples);
        double length = 0.0;
        double prevX = xPoly.compute(0.0);
        double prevY = yPoly.compute(0.0);
        for (int i = 1; i < samples; i++) {
            double t = (double) i / (samples - 1);
            double curX = xPoly.compute(t);
            double curY = yPoly.compute(t);
            length += Math.hypot(curX - prevX, curY - prevY);
            prevX = curX;
            prevY = curY;
        }
        return length;
    }

    private double putTimeInRange(double time) {
        return Math.max(Math.min(time, endTime), startTime);
    }

    /**
     * Returns the x-coordinate at the given absolute time.
     */
    public double getX(double time) {
        return xPolynomial.compute(putTimeInRange(time) - startTime);
    }

    /**
     * Returns the y-coordinate at the given absolute time.
     */
    public double getY(double time) {
        return yPolynomial.compute(putTimeInRange(time) - startTime);
    }

    /**
     * Returns the rotation at the given absolute time.
     * If constantAngle is true, evaluates the interpolated rotation polynomial.
     * Otherwise, derives heading from the direction of travel via a look-ahead.
     */
    public double getRotation(double time) {
        if (this.constantAngle) {
            return rotPolynomial.compute(putTimeInRange(time) - startTime);
        } else {
            time = putTimeInRange(time);
            double lookAheadTime = (time + 0.1 <= endTime) ? time + 0.1 : endTime;
            double xDifference = getX(lookAheadTime) - getX(time);
            double yDifference = getY(lookAheadTime) - getY(time);
            xDifference = Math.abs(xDifference) > 0 ? xDifference : 1e-7;
            yDifference = Math.abs(yDifference) > 0 ? yDifference : 1e-7;
            return Math.toDegrees(Math.atan2(yDifference, xDifference));
        }
    }

    /**
     * Returns the instantaneous speed (magnitude of velocity vector) at the given absolute time.
     * Computed from the derivatives of the x and y polynomials.
     */
    public double getSpeed(double time) {
        double dxdt = xPolynomial.derivative(putTimeInRange(time) - startTime);
        double dydt = yPolynomial.derivative(putTimeInRange(time) - startTime);
        return Math.hypot(dxdt, dydt);
    }

    /** Returns the absolute start time of this segment. */
    public double getStartTime() {
        return startTime;
    }

    /** Returns the absolute end time of this segment. */
    public double getEndTime() {
        return this.endTime;
    }
}

/**
 * Support class for CubicSplineSegment. Holds coefficients for a cubic polynomial
 * f(t) = a*t^3 + b*t^2 + c*t + d, evaluated over a normalized [0,1] domain via a
 * time scalar that maps real segment duration to that range.
 */
class CubicPolynomial {
    private final double a, b, c, d;
    private double timeScalar = 1.0;

    /**
     * Construct from a 1x4 (or 4-element) SimpleMatrix {a, b, c, d}.
     */
    public CubicPolynomial(SimpleMatrix coeffs) {
        this.a = coeffs.get(0);
        this.b = coeffs.get(1);
        this.c = coeffs.get(2);
        this.d = coeffs.get(3);
    }

    /**
     * Set the time scalar to map absolute time to the [0,1] range.
     * @param segmentDuration duration of this segment in seconds
     */
    public void setTimeScalar(double segmentDuration) {
        this.timeScalar = segmentDuration > 0 ? segmentDuration : 1.0;
    }

    /**
     * Evaluate the polynomial at the given time (relative to segment start).
     * Internally normalizes to t = time/timeScalar, clamped to [0,1].
     */
    public double compute(double time) {
        double t = time / timeScalar;
        if (t < 0) t = 0;
        if (t > 1) t = 1;
        return a * Math.pow(t, 3) + b * Math.pow(t, 2) + c * t + d;
    }

    /**
     * Compute the derivative of the polynomial with respect to absolute time at the given
     * relative time. Uses the chain rule: d/dt = (d/dτ) * (1/timeScalar).
     */
    public double derivative(double time) {
        double t = time / timeScalar;
        if (t < 0) t = 0;
        if (t > 1) t = 1;
        double dDtau = 3.0 * a * Math.pow(t, 2) + 2.0 * b * t + c;
        return dDtau / timeScalar;
    }
}
