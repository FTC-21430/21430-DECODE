/// Copyright (c) 2026 Tobin Rumsey, all rights reserved
/// Comment assist by ChatGPT, OpenAI - Github Copilot
/// Prototype for class and python implementation made in Google Colab, Formatting Help by Google Gemini.
/// Theory behind concept from Catmull-Rom splines and general cubic spline interpolation, as well as time-based path following algorithms commonly used in robotics.
/// Permitted use of class for FIRST Tech Challenge team 21430, BroomBots until the end of the 2029 season,
/// with the expectation that the next generation of students will have to learn how to do this themselves by then.
/// All other FTC teams and non-FTC users are permitted to use this code for educational and or competition purposes, but are encouraged to learn how to implement cubic spline interpolation and path following algorithms themselves for a deeper understanding of the underlying mathematics and robotics concepts.
/// This code is provided as-is for educational purposes, and is not guaranteed to be bug-free or suitable for all use cases. It is the responsibility of the user to test and validate the code for their specific application,
///  and to understand the underlying mathematics and algorithms involved in cubic spline interpolation and path following.
/// Thank you for respecting the intellectual property and educational intent of this code! <3 - Tobin Rumsey, BroomBots 21430
package org.example.visualizer.spline;

/**
 * CubicSplineSegment represents a single segment of a cubic spline path, defined by cubic polynomials for x, y, and rotation (angle) as functions of time along the segment.
 * Each segment is constructed using Catmull-Rom interpolation based on 4 control points (
 * the previous waypoint, the start waypoint, the end waypoint, and the next waypoint) to ensure smooth transitions between segments.
 * The segment also calculates its duration based on the distance between the start and end points and the desired speed defined in the end waypoint, allowing for time-based evaluation of the spline.
 * The compute() method allows for evaluating the position and rotation at any given absolute time along the segment, making it compatible with path following algorithms that operate in real time and need to know the target position and orientation at any given time.
 * This class is designed to be used as part of a larger path following system, where multiple CubicSplineSegments are chained together to form a complete path for the robot to follow.
 * The class also includes an alternate constructor for creating "hold" segments, which keep the robot at a fixed position and orientation for a specified duration, allowing for easy implementation of wait times at waypoints without needing special handling in the path following logic.
 * Overall, CubicSplineSegment encapsulates the mathematical representation of a segment of a cubic spline path, including the geometry defined by the control points and the timing based on the desired speed, providing a convenient interface for evaluating the target position and orientation at any given time along the segment.
 * Note: This class is designed to be used in a real-time path following context, where the compute() method will be called repeatedly with the current time to get the target position and orientation for the robot to follow. The internal use of cubic polynomials allows for smooth interpolation between waypoints, while the time-based evaluation ensures that the robot can follow the path at the desired speed.
 */

public class CubicSplineSegment {
    private final CubicPolynomial xPolynomial, yPolynomial, rotPolynomial; // Cubic polynomials for x, y, and rotation (angle) as functions of time along the segment, using custom class to optimize code complexity
    // CubicPolynomial is exclusive to this class, learn about it at the bottom of this file

    private final double startTime, endTime; // Absolute time at which this segment starts and ends; used to determine which segment to evaluate at a given time and to convert absolute time to the 0-1 range for polynomial evaluation

    /**
     * Construct a spline segment between startPoint and endPoint using Catmull-Rom interpolation.
     * @param lastPoint the waypoint before the start point (used for tangent calculation) - if this is the first segment, you can repeat the startPoint
     * @param startPoint the starting waypoint of this segment
     * @param endPoint the ending waypoint of this segment
     * @param nextPoint the waypoint after the end point (used for tangent calculation) - if this is the last segment, you can repeat the endPoint
     * @param startTime the absolute time at which this segment should start (seconds)
     * @param robotTopSpeed the maximum speed of the robot (units per second) used to scale the segment duration based on the speed ratio defined in the endPoint's speed field (0
     */
    public CubicSplineSegment(Waypoint lastPoint, Waypoint startPoint, Waypoint endPoint, Waypoint nextPoint, double startTime, double robotTopSpeed) {
        // Ensure valid parameters
        this.startTime = Math.abs(startTime);
        robotTopSpeed = Math.abs(robotTopSpeed);

        // calculate segment speed based on the speed ratio defined in the endPoint
        double speedRatio = endPoint.getSpeed();

        // Use the robot's top speed scaled by the speed ratio to determine how fast we want to traverse this segment;
        // if speedRatio is 0 or negative, default to a minimum speed to avoid division by zero or infinite time
        double segmentSpeed = speedRatio * robotTopSpeed;
        segmentSpeed = segmentSpeed > 0.0 ? segmentSpeed : 1.0;

        // Build Catmull-Rom cubic coefficients for x, y, and rotation
        // Using respective components from the 4 control points (last, start, end, next) to calculate the cubic polynomial coefficients for each dimension
        double[] xCoeffs = catmullRomCoeffs(lastPoint.getX(), startPoint.getX(), endPoint.getX(), nextPoint.getX());
        double[] yCoeffs = catmullRomCoeffs(lastPoint.getY(), startPoint.getY(), endPoint.getY(), nextPoint.getY());
        double[] rCoeffs = catmullRomCoeffs(lastPoint.getAngle(), startPoint.getAngle(), endPoint.getAngle(), nextPoint.getAngle());

        // Create CubicPolynomial instances for x, y, and rotation using the calculated coefficients
        CubicPolynomial xp = new CubicPolynomial(xCoeffs[0], xCoeffs[1], xCoeffs[2], xCoeffs[3]);
        CubicPolynomial yp = new CubicPolynomial(yCoeffs[0], yCoeffs[1], yCoeffs[2], yCoeffs[3]);
        CubicPolynomial rp = new CubicPolynomial(rCoeffs[0], rCoeffs[1], rCoeffs[2], rCoeffs[3]);

        // Estimate segment length by sampling the curve in parameter space [0,1]
        double length = estimateArcLength(xp, yp, 100);
        length = length > 0.0 ? length : 1.0;
        double segmentTime = length / segmentSpeed;

        // Currently the polynomials range from 0.0 - 1.0, time scalar allows the full duration of the segment to be mapped to this range, so that compute() can take absolute time and internally convert to 0-1 for the polynomial evaluation
        xp.setTimeScalar(segmentTime);
        yp.setTimeScalar(segmentTime);
        rp.setTimeScalar(segmentTime);

        // Save the polynomials and end time for this segment
        this.xPolynomial = xp;
        this.yPolynomial = yp;
        this.rotPolynomial = rp;
        this.endTime = this.startTime + segmentTime;
    }

    /**
     * Alternate constructor for a "hold" segment that keeps the robot at a fixed position and orientation for a specified duration.
     * compatible with the same path following algorithms since it just uses constant polynomials, but allows for easy implementation of wait times at waypoints.
     * @param holdPoint the point to hold at for the duration of this segment (Waypoint Object with x, y, and angle)
     * @param startTime the absolute time at which this hold segment should start (seconds)
     * @param duration the duration to hold at the point (seconds)
     */
    public CubicSplineSegment(Waypoint holdPoint, double startTime, double duration){
        // Save start time and end time
        this.startTime = startTime;
        this.endTime = startTime + duration;

        // set polynomials to be in form of constants so that the target x position return is the hold point,
        // works well because the same interpreter and algorithms for following paths can also be used in the same way for waits!
        this.xPolynomial = new CubicPolynomial(0,0,0,holdPoint.getX());
        this.yPolynomial = new CubicPolynomial(0,0,0,holdPoint.getY());
        this.rotPolynomial = new CubicPolynomial(0,0,0,holdPoint.getAngle());

        // Currently the polynomials range from 0.0 - 1.0, time scalar allows the full duration of the segment to be mapped to this range, so that compute() can take absolute time and internally convert to 0-1 for the polynomial evaluation
        this.xPolynomial.setTimeScalar(duration);
        this.yPolynomial.setTimeScalar(duration);
        this.rotPolynomial.setTimeScalar(duration);
    }

    /**
     * Calculate the coefficients of a cubic polynomial for Catmull-Rom interpolation given 4 control points (P0, P1, P2, P3).
     * @param P0 the point before the start point (used for tangent calculation)
     * @param P1 the start point of the segment
     * @param P2 the end point of the segment
     * @param P3 the point after the end point (used for tangent calculation)
     * @return returns an array of coefficients [a, b, c, d] such that the cubic polynomial can be evaluated as a*t^3 + b*t^2 + c*t + d for t in [0,1]
     */
    private double[] catmullRomCoeffs(double P0, double P1, double P2, double P3) {
        // Coeffs such that value(t) = a*t^3 + b*t^2 + c*t + d for t in [0,1]
        double d = P1;
        double c = 0.5 * (P2 - P0);
        double b = 0.5 * (2*P0 - 5*P1 + 4*P2 - P3);
        double a = 0.5 * (-P0 + 3*P1 - 3*P2 + P3);

        // return array with all our coeffs as the elements, in order from highest degree (3) to lowest (0)
        return new double[]{a, b, c, d};
    }

    /**
     * Estimate the arc length of the curve defined by the cubic polynomials xp and yp by sampling points along the curve and summing the distances between them.
     * @param xp the cubic polynomial representing the x coordinate as a function of time parameter t in [0,1]
     * @param yp the cubic polynomial representing the y coordinate as a function of time parameter t in [0,1]
     * @param samples the resolution of the sampling; more samples will yield a more accurate length estimate but will take more computation time; should be at least 2 to get a valid length estimate
     * @return returns the estimated arc length of the curve defined by xp and yp between t=0 and t=1, in the same units as the x and y coordinates (e.g. inches)
     */
    private double estimateArcLength(CubicPolynomial xp, CubicPolynomial yp, int samples) {
        // ensure valid parameters
        samples = Math.max(2, samples);

        // Sample the curve at regular intervals of t in [0,1], compute the distance between consecutive points, and sum these distances to get an estimate of the total arc length
        double prevX = xp.compute(0.0);
        double prevY = yp.compute(0.0);

        // local variable to accumulate the total length and return at the end
        double len = 0.0;

        // for loop to sample the curve at regular intervals of t in [0,1], compute the distance between consecutive points, and sum these distances to get an estimate of the total arc length
        for (int i = 1; i < samples; i++) {
            // compute the current t parameter for this sample, which ranges from 0 to 1 as i goes from 0 to samples-1
            double t = (double) i / (samples - 1);

            // compute the x and y coordinates of the curve at this t parameter using the provided cubic polynomials xp and yp
            double cx = xp.compute(t);
            double cy = yp.compute(t);

            // Get the difference of the arc length between prev point and current point
            double dx = cx - prevX;
            double dy = cy - prevY;

            // Add hypotenuse to total length
            len += Math.hypot(dx, dy);

            // And update prevX and prevY for the next iteration
            prevX = cx;
            prevY = cy;
        }

        // Return the estimated arc length of the curve defined by xp and yp between t=0 and t=1, in the same units as the x and y coordinates (e.g. inches)
        return len;
    }

    /**
     * Get the x position for time
     * @param time absolute time (seconds) from start of total path
     * @return return x coordinate in FTC field coordinate system (inches)
     */
    public double getX(double time){
        return xPolynomial.compute(time - startTime);
    }

    /**
     * Get the y position for time
     * @param time absolute time (seconds) from start of total path
     * @return return y coordinate in FTC field coordinate system (inches)
     */
    public double getY(double time){
        return yPolynomial.compute(time - startTime);
    }
    /**
     * Get the rotation (angle) for time
     * @param time absolute time (seconds) from start of total path
     * @return return angle in FTC field coordinate system (degrees)
     */

    public double getRotation(double time){
        return rotPolynomial.compute(time - startTime);
    }

    /**
     * @param time absolute time (seconds) from start of total path
     * @return return speed (units per second) at the given time
     */

    public double getSpeed(double time) {
        // Compute the derivatives of x and y with respect to time
        double dxdt = xPolynomial.derivative(time - startTime);
        double dydt = yPolynomial.derivative(time - startTime);

        // return speed using the Pythagorean theorem to get the speed (magnitude of velocity vector)
        return Math.hypot(dxdt, dydt);
    }

    /**
     * Get the absolute time at which this segment ends, used by the path following algorithm to determine when to switch to the next segment
     * @return time (seconds)
     */
    public double endTime(){
        return this.endTime;
    }

    // END OF CUBIC SPLINE SEGMENT CLASS
}

/**
 * CubicPolynomial is a helper class to represent a cubic polynomial of the form a*t^3 + b*t^2 + c*t + d, where t is a normalized parameter
 * that typically ranges from 0 to 1 over the duration of a spline segment.
 * All values are finally set in the constructor, but the timeScalar can be adjusted to map absolute time to the 0-1 range for polynomial evaluation.
 * the compute() method --main class purpose-- takes an absolute time input, converts it to the normalized parameter t using the timeScalar, and evaluates the cubic polynomial at that t.
 */
class CubicPolynomial {
    private final double a, b, c, d; // coefficients for t^3, t^2, t, 1
    private double timeScalar = 1.0; // maps absolute time to 0..1 parameter by input/timeScalar

    /**
     * Constructor for CubicPolynomial, takes coefficients a, b, c, d such that the polynomial can be evaluated as a*t^3 + b*t^2 + c*t + d for t in [0,1]
     * @param a coefficient for t^3 term
     * @param b coefficient for t^2 term
     * @param c coefficient for t term
     * @param d constant term, typically the start point value for the dimension this polynomial represents
     */
    public CubicPolynomial(double a, double b, double c, double d) {
        this.a = a;
        this.b = b;
        this.c = c;
        this.d = d;
    }

    /**
     * Set the time scalar to map absolute time to the 0-1 range for polynomial evaluation.
     * The segment duration is used as the time scalar so that when compute() is given an absolute time,
     * it can internally convert it to the normalized parameter t in [0,1] for evaluating the cubic polynomial.
     * @param segmentDuration The duration of the segment (seconds) end-start
     */
    public void setTimeScalar(double segmentDuration) {
        this.timeScalar = segmentDuration > 0 ? segmentDuration : 1.0;
    }

    /**
     * compute expects an absolute time (seconds) relative to the segment start; it divides by timeScalar to get t in [0,1]
     * @param time absolute time (seconds) from start of total path
     * @return resulting polynomial, (normally inches for x and y, degrees for rotation)
     */
    public double compute(double time) {
        double t = time / timeScalar; // scale down absolute time to the 0-1 range for polynomial evaluation
        // Clamp t to [0,1] to avoid extrapolation for safety
        if (t < 0) t = 0;
        if (t > 1) t = 1;

        // Compute the time value, in form a*t^3 + b*t^2 + c*t + d using the coefficients and the normalized parameter t
        return a * Math.pow(t,3) + b * Math.pow(t,2) + c * Math.pow(t,1) + d;
    }

    /**
     * Compute the derivative of the cubic polynomial with respect to absolute time at a given input time. This is used to calculate the speed along the curve at a given time.
     * @param time absolute time (seconds) from start of total path
     * @return returns the derivative of the cubic polynomial with respect to absolute time at the given position
     */
    public double derivative(double time) {
        // scale time in terms of duration from segment start time to get t in [0,1] for polynomial evaluation
        double t = time / timeScalar;

        // clamp within [0,1]
        if (t < 0) t = 0;
        if (t > 1) t = 1;

        // derivative w.r.t normalized parameter tau is: 3a*t^2 + 2b*t + c
        double d_dtau = 3.0 * a * Math.pow(t,2) + 2.0 * b * t + c;

        // return resulting derivative with respect to absolute time by applying the chain rule: dx/dt = dx/dtau * dtau/dt, where dtau/dt = 1/timeScalar since tau = time/timeScalar
        return d_dtau * (1.0 / timeScalar);
    }
}