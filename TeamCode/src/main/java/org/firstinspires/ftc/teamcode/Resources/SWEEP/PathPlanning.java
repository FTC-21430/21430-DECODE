package org.firstinspires.ftc.teamcode.Resources.SWEEP;

import org.firstinspires.ftc.teamcode.Firmware.DecodeBot;

import java.util.ArrayList;

/** This class is designed as the path planner, goal is to take inputs from an opmode using methods such as "splineTo()" to add waypoints to an array,
 * The waypoint sub-class is an object designed to make this part cleaner
 * Once the route is defined, it gets compiled into splines.
 * Splines are retuned back to the SplineFollower in form of af CubicSplineSegment array.
**/
public class PathPlanning {
    private RobotActions robotActions;
    private ArrayList<Waypoint> waypoints;

    // spline count goes up with every new spline that is going to exist. One waypoint is not enough. splines = waypoints - 1. 0 indexed
    private int splineCount = -1;
    public PathPlanning(DecodeBot bot){
        this.robotActions = new RobotActions(bot);
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
    public void resetGeneration(){
        waypoints = new ArrayList<Waypoint>();
        splineCount = 0;

    }
    /**
     * Add a wait to the route so that the robot paused in it's path
     * @param time
     */
    public void chill(double x, double y, double angle, double time){
        //TODO:
    }
    public void addAction(RobotActions.Actions actionType, double triggerTime){
        robotActions.addAction(actionType, triggerTime);
    }
    public void addAction(RobotActions.Actions actionType){
        robotActions.addAction(actionType, splineCount);
    }
    // TODO, move Action outside of 'RobotActions.java' into it's own file
    public Action[] compileActions(){
        return robotActions.compileActions();
    }
    public CubicSplineSegment[] generatePath(){
        //TODO - make this return a fully generated set of splines
        return new CubicSplineSegment[0];
    }
}