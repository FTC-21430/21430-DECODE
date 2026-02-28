package org.firstinspires.ftc.teamcode.Resources.SplineFollowing;

import android.os.Bundle;

@SuppressWarnings("unused")
public class CubicSplineSegment {
    private final CubicPolynomial xPolynomial, yPolynomial, rotPolynomial;
    private final double startTime, endTime;

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
    public CubicSplineSegment(Waypoint lastPoint, Waypoint startPoint, Waypoint endPoint, Waypoint nextPoint) {
        // TODO: Something like waypoint.get start time and stuff to set these,
        startTime = 0;
        endTime = 0;

        // TODO: write motified Catmull-Rom algorithm to convert these points to the components of each of these polynomials.
        xPolynomial = new CubicPolynomial(0, 0, 0, 0);
        yPolynomial = new CubicPolynomial(0, 0, 0, 0);
        rotPolynomial = new CubicPolynomial(0, 0, 0, 0);

    }

   /**
     * Returns the x-coordinate value of this spline segment at the given time.
     *
     * @param time the parameter (time) at which to evaluate the cubic polynomial
     * @return the x-coordinate value at the specified time
     */
    public double getX(double time){
        return xPolynomial.compute(time);
    }

    /**
     * Returns the y-coordinate value of this spline segment at the given time.
     *
     * @param time the parameter (time) at which to evaluate the cubic polynomial
     * @return the y-coordinate value at the specified time
     */
    public double getY(double time){
        return yPolynomial.compute(time);
    }

    /**
     * Returns the rotation value of this spline segment at the given time.
     *
     * @param time the parameter (time) at which to evaluate the cubic polynomial
     * @return the rotation value at the specified time
     */
    public double getRotation(double time){
        return rotPolynomial.compute(time);
    }
}
/**
 * support class for CubicSplineSegment, holds information for each component of the spline,
 * takes 4 constants and can return the result in terms of what should be time in this case
 */
class CubicPolynomial {
    private final double a, b, c, d;
    public CubicPolynomial(double a, double b, double c, double d){
        this.a = a;
        this.b = b;
        this.c = c;
        this.d = d;
    }
    public double compute(double input){
        double cubicTerm = a * Math.pow(input,3);
        double quadraticTerm = b * Math.pow(input,2);
        double linearTerm = c * input;
        double constantTerm = d;
        return cubicTerm + quadraticTerm + linearTerm + constantTerm;
    }
}
