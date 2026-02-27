package org.firstinspires.ftc.teamcode.Resources.SplineFollowing;

import org.firstinspires.ftc.teamcode.Resources.SplineFollowing.Waypoint;

/** This class is designed as the path planner, goal is to take inputs from an opmode using methods such as "splineTo()" to add waypoints to an array,
 * The waypoint sub-class is an object designed to make this part cleaner
 * Once the route is defined, it gets compiled into splines.
 * Splines are retuned back to the SplineFollower in form of af CubicSplineSegment array.
**/
public class PathPlanning {
    private Waypoint[] waypoints;
    public PathPlanning(){

    }

    /**
     * Main waypoint generation function, makes robot spine to a point while matching the front of the robot toward the spline.
     * @param x end x - inches
     * @param y end y - inches
     * @param velocity - the top velocity ratio of the robot, 0 = no movement, 1 = full speed
     */
    public void splineTo(double x, double y, double velocity){
        // TODO: make this
    }
    public void splineToConstantAngle(double x, double y, double angle, double velocity){
        // TODO: make this
    }

    /**
     * Add a wait to the route so that the robot paused in it's path
     * @param time
     */
    public void wait(double time){
        // TODO: make this
    }
    public CubicSplineSegment[] generatePath(){
        //TODO - make this return a fully generated set of splines
        return new CubicSplineSegment[0];
    }
}