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
    private double robotSpeed = 20; // my guess of 20 inches / s
    public PathPlanning(DecodeBot bot){
        this.robotActions = new RobotActions(bot);
        this.waypoints = new ArrayList<Waypoint>();
    }

    /**
     * Main waypoint generation function, makes robot spine to a point while matching the front of the robot toward the spline.
     * @param x end x - inches
     * @param y end y - inches
     * @param speedRatio - the top velocity ratio of the robot, 0 = no movement, 1 = full speed
     */
    //This function 
    public void splineTo(double x, double y, double speedRatio){
        Waypoint waypoint = new Waypoint(x,y,0,speedRatio, false);
        waypoints.add(waypoint);
        splineCount ++;
    }
    //The spline start function is the start point for spline curve
    public void splineStart(double x, double y, double angle){
        Waypoint waypoint = new Waypoint(x,y,angle,0,false);
        waypoints.add(waypoint);
        splineCount++;
    }
    //This function gets the spline to the constant angle
    public void splineToConstantAngle(double x, double y, double angle, double speedRatio){
        Waypoint waypoint = new Waypoint(x,y,angle,speedRatio,true);
        waypoints.add(waypoint);
        splineCount ++;
    }
    //This function resets it
    public void resetGeneration(){
        waypoints = new ArrayList<Waypoint>();
        splineCount = -1;

    }
    /**
     * Add a wait to the route so that the robot paused in it's path
     * @param time
     */
    //This chill function is basically the wait time
    public void chill(double x, double y, double angle, double time){
        Waypoint waypoint = new Waypoint(x,y,angle,time);
        waypoints.add(waypoint);
        splineCount ++;
    }
    //This adds the robot action and the trigger time
    public void addAction(RobotActions.Actions actionType, double triggerTime){
        robotActions.addAction(actionType, triggerTime);
    }
    //This adds the action spefically the action type
    public void addAction(RobotActions.Actions actionType){
        robotActions.addAction(actionType, splineCount);
    }
    public Action[] compileActions(){
        return robotActions.compileActions();
    }
    //This is the function where it these everything
    public CubicSplineSegment[] generatePath() {
        //A if statement that just returns 0 if it can't identify more than 1 spline point
        if (waypoints.size()<2){
            return new CubicSplineSegment[0];
        }
        double time = 0;
        ArrayList<CubicSplineSegment> path = new ArrayList<CubicSplineSegment>();
        // iterate every waypoint; create a wait-segment for chill points, otherwise build a spline
        for (int i = 0; i < waypoints.size(); i++) {
            Waypoint current = waypoints.get(i);
            // if this is an explicit wait/chill waypoint, create a time-based segment
            if (current.isWaitPoint()) {
                CubicSplineSegment spline = new CubicSplineSegment(current, time, current.getDuration());
                path.add(spline);
                time = spline.getEndTime();
                continue;
            }

            // regular spline: need a following waypoint to form a segment; if none, skip
            if (i >= waypoints.size() - 1) {
                continue;
            }

            int prevIdx = Math.max(0, i - 1);
            int startIdx = i;
            int endIdx = i + 1;
            int nextIdx = Math.min(i + 2, waypoints.size() - 1);

            Waypoint prev = waypoints.get(prevIdx);
            Waypoint start = waypoints.get(startIdx);
            Waypoint end = waypoints.get(endIdx);
            Waypoint next = waypoints.get(nextIdx);

            CubicSplineSegment spline = new CubicSplineSegment(prev, start, end, next, time, robotSpeed, end.shouldHoldAngle());
            time = spline.getEndTime();
            path.add(spline);
        }
        return path.toArray(new CubicSplineSegment[0]);

    }

}