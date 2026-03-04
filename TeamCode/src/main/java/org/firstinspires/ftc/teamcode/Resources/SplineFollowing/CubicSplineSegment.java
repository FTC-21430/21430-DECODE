/// Copyright (c) 2026 Tobin Rumsey, all rights reserved
/// Comment assist by ChatGPT, OpenAI - Github Copilot
/// Prototype for class and python implementation made in Google Colab, Formatting Help by Google Gemini. EJML Matrix library used because I needed Matrix's :)
/// Theory behind concept from Catmull-Rom splines and general cubic spline interpolation, as well as time-based path following algorithms commonly used in robotics.
/// Permitted use of class for FIRST Tech Challenge team 21430, BroomBots until the end of the 2029-2030 season,
/// with the expectation that the next generation of students will have to learn how to do this themselves by then.
/// All other FTC teams and non-FTC users are permitted to use this code for educational and or competition purposes, but are encouraged to learn how to implement cubic spline interpolation and path following algorithms themselves for a deeper understanding of the underlying mathematics and robotics concepts.
/// This code is provided as-is for educational purposes, and is not guaranteed to be bug-free or suitable for all use cases. It is the responsibility of the user to test and validate the code for their specific application,
///  and to understand the underlying mathematics and algorithms involved in cubic spline interpolation and path following.
/// Thank you for respecting the intellectual property and educational intent of this code! <3 - Tobin Rumsey, BroomBots 21430
package org.firstinspires.ftc.teamcode.Resources.SplineFollowing;

import org.ejml.simple.SimpleMatrix;

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
     * One pathing segment, contains three cubic polynomials. Input four waypoints that define the path in terms of time.
     * Calculates the spline
     * If one of these points does not exist due to a stop or the start or end of the path, provide a duplicate of either the start or end point.
     * start and end points are REQUIRED to be unique.
     *
     * @param lastPoint  the start point of the previous spline
     * @param startPoint where this spline starts
     * @param endPoint   where this spline ends
     * @param nextPoint  where the next spline ends
     */
    public CubicSplineSegment(Waypoint lastPoint, Waypoint startPoint, Waypoint endPoint, Waypoint nextPoint, double startTime, double robotTopSpeed) {
        // ensure that negative numbers -- which we should not get but nice to not crash -- is supported (yes I used an em-dash. I am not an AI, but I like them. - Tobin)
        this.startTime = Math.abs(startTime);
        robotTopSpeed = Math.abs(robotTopSpeed);

        double speedRatio = endPoint.getSpeed();
        double segmentSpeed = speedRatio * robotTopSpeed;


        SimpleMatrix segmentControlPoints = new SimpleMatrix(new double[][]{
            {lastPoint.getX(),lastPoint.getY(),lastPoint.getAngle()},
            {startPoint.getX(),startPoint.getY(),startPoint.getAngle()},
            {endPoint.getX(),endPoint.getY(),endPoint.getAngle()},
            {nextPoint.getX(),nextPoint.getY(),nextPoint.getAngle()}
        });

        // alpha = 0.5 is standard for Catmull-Rom implementation
        CubicPolynomial[] polynomials = createCubicSplineSegment(segmentControlPoints, 0.5);

        double segmentLength = getArcLength(polynomials[0], polynomials[1]);
        segmentLength = segmentLength > 0.0 ? segmentLength : 1.0;
        segmentSpeed = segmentSpeed > 0.0 ? segmentSpeed : 1.0;
        double segmentTime = segmentLength / segmentSpeed;
        for (CubicPolynomial poly : polynomials){
            poly.setTimeScalar(segmentTime);
        }
        this.xPolynomial = polynomials[0];
        this.yPolynomial = polynomials[1];
        this.rotPolynomial = polynomials[2];

        this.endTime = startTime + segmentTime;
    }
    private CubicPolynomial[] createCubicSplineSegment(SimpleMatrix segmentControlPoints, double splineAlpha){
        SimpleMatrix P0 = segmentControlPoints.getRow(0);
        SimpleMatrix P1 = segmentControlPoints.getRow(1);
        SimpleMatrix P2 = segmentControlPoints.getRow(2);
        SimpleMatrix P3 = segmentControlPoints.getRow(3);
        // P = (x, y, yaw)

        // Calculate times between all of the spline control points

        double t0 = 0.0;
        double t1 = t0 + Math.pow(P1.minus(P0).normF(), splineAlpha);
        double t2 = t1 + Math.pow(P2.minus(P1).normF(), splineAlpha);
        double t3 = t2 + Math.pow(P3.minus(P2).normF(), splineAlpha);


        double scaleFactor = (t2 - t1); // internal scale factor the current range of 0.0-1.0 that is the spline

        // Tangent segments:

        // prevent divide by zero errors
        if (t2 - t0 == 0) t2 += Math.pow(1,-7);
        if (t3 - t1 == 0) t3 += Math.pow(1,-7);

        // first for the X polynomial
        double Mx1 = ((P2.get(0) - P0.get(0)) / (t2 - t0)) * scaleFactor;
        double Mx2 = ((P3.get(0) - P1.get(0)) / (t3 - t1)) * scaleFactor;

        // now for the Y polynomial

        double My1 = ((P2.get(1) - P0.get(1)) / (t2 - t0)) * scaleFactor;
        double My2 = ((P3.get(1) - P1.get(1)) / (t3 - t1)) * scaleFactor;

        // Now for the angle (yaw) polynomial

        double Ma1 = ((P2.get(2) - P0.get(2)) / (t2 - t0)) * scaleFactor;
        double Ma2 = ((P3.get(2) - P1.get(2)) / (t3 - t1)) * scaleFactor;

        // a = angle = yaw

        SimpleMatrix xCoeffs = getSplineCoeffs(P1.get(0),P2.get(0),Mx1,Mx2);
        SimpleMatrix yCoeffs = getSplineCoeffs(P1.get(1),P2.get(1),My1,My2);
        SimpleMatrix aCoeffs = getSplineCoeffs(P1.get(2),P2.get(2),Ma1,Ma2);

        CubicPolynomial XPolynomial = new CubicPolynomial(xCoeffs);
        CubicPolynomial YPolynomial = new CubicPolynomial(yCoeffs);
        CubicPolynomial YawPolynomial = new CubicPolynomial(aCoeffs);
        return new CubicPolynomial[]{
                XPolynomial, YPolynomial, YawPolynomial
        };

    }
    private SimpleMatrix getSplineCoeffs(double v1, double v2, double Mu1, double Mu2){
        // Coeffs formulas are from the expansion of the standard Catmull-Rom Spline formula - Learn more here https://en.wikipedia.org/wiki/Catmull%E2%80%93Rom_spline
        double d = v1;
        double c = Mu1;
        double b = (-1 * v1) - (2 * Mu1) + (3 * v2) - Mu2;
        double a = (2 * v1) + Mu1 - (2 * v2) + Mu2;
        // Return in Matrix form - 1x4
        return new SimpleMatrix(new double[][]{
            {a,b,c,d}
        });
    }
    /**
     * Compute the geometric arc length of a parametric curve defined by x(t) and y(t)
     * cubic polynomials over t in [0,1]. Returned length is in the same units as
     * the x/y polynomials (e.g. meters).
     */
    private double getArcLength(CubicPolynomial xPoly, CubicPolynomial yPoly){
        int segment_resolution = Math.max(2, 100);
        SimpleMatrix ts = new SimpleMatrix(1, segment_resolution);
        for (int i = 0; i < segment_resolution; i++) {
            ts.set(0, i, (double) i / (segment_resolution - 1));
        }

        double length = 0.0;
        double prevX = xPoly.compute(ts.get(0, 0));
        double prevY = yPoly.compute(ts.get(0, 0));
        for (int i = 1; i < segment_resolution; i++) {
            double t = ts.get(0, i);
            double curX = xPoly.compute(t);
            double curY = yPoly.compute(t);
            double dx = curX - prevX;
            double dy = curY - prevY;
            length += Math.hypot(dx, dy);
            prevX = curX;
            prevY = curY;
        }
        return length;
    }
    private double putTimeInRange(double time){
        // ensure that the inputted time is within the range of the polynomial
        return Math.max(Math.min(time, endTime), startTime);

    }

   /**
     * Returns the x-coordinate value of this spline segment at the given time.
     *
     * @param time the parameter (time) at which to evaluate the cubic polynomial
     * @return the x-coordinate value at the specified time
     */
    public double getX(double time){
        return xPolynomial.compute(putTimeInRange(time) - startTime);
    }

    /**
     * Returns the y-coordinate value of this spline segment at the given time.
     *
     * @param time the parameter (time) at which to evaluate the cubic polynomial
     * @return the y-coordinate value at the specified time
     */
    public double getY(double time){
        return yPolynomial.compute(putTimeInRange(time) - startTime);
    }

    /**
     * Returns the rotation value of this spline segment at the given time.
     *
     * @param time the parameter (time) at which to evaluate the cubic polynomial
     * @return the rotation value at the specified time
     */
    public double getRotation(double time){
        return rotPolynomial.compute(putTimeInRange(time) - startTime);
    }

    /**
     * Returns the absolute start time of this spline segment.
     * @return
     */
    public double getStartTime() {
        return startTime;
    }

    /**
     * Returns the absolute end time of this spline segment.
     * @return
     */
    public double getEndTime(){
        return this.endTime;
    }
}
/**
 * support class for CubicSplineSegment, holds information for each component of the spline,
 * takes 4 constants and can return the result in terms of what should be time in this case
 */
class CubicPolynomial {
    private final double a, b, c, d;
    private double timeScalar = 1;
    public CubicPolynomial(SimpleMatrix coeffs){
        this.a = coeffs.get(0);
        this.b = coeffs.get(1);
        this.c = coeffs.get(2);
        this.d = coeffs.get(3);
    }
    public void setTimeScalar(double segmentDuration){
        this.timeScalar = segmentDuration;
    }
    public double compute(double input){
        input /= timeScalar;
        double cubicTerm = a * Math.pow(input,3);
        double quadraticTerm = b * Math.pow(input,2);
        double linearTerm = c * input;
        double constantTerm = d;
        return cubicTerm + quadraticTerm + linearTerm + constantTerm;
    }
}