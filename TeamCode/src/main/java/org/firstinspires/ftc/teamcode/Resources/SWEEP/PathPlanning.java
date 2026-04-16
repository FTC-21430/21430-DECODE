package org.firstinspires.ftc.teamcode.Resources.SWEEP;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Firmware.DecodeBot;

import java.util.ArrayList;

/** This class is designed as the path planner, goal is to take inputs from an opmode using methods such as "splineTo()" to add waypoints to an array,
 * The waypoint sub-class is an object designed to make this part cleaner
 * Once the route is defined, it gets compiled into splines.
 * Splines are retuned back to the SplineFollower in form of af CubicSplineSegment array.
**/
@Config
public class PathPlanning {
    private RobotActions robotActions;
    private ArrayList<Waypoint> waypoints;

    // spline count goes up with every new spline that is going to exist. One waypoint is not enough. splines = waypoints - 1. 0 indexed
    private int splineCount = 0;
    public static double robotSpeed = 38; // my guess of 20 inches / s
    public static double minHoldTime = 0.2;

    private double previousX, previousY, previousAngle;

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
        previousX = x;
        previousY = y;
    }
    //The spline start function is the start point for spline curve
    public void splineStart(double x, double y, double angle){
        Waypoint waypoint = new Waypoint(x,y,angle,0,true);
        waypoints.add(waypoint);
        previousX = x;
        previousY = y;
        previousAngle = angle;

    }
    public void splineEnd(double x,double y,double angle){
        chill(x,y,angle,2);
        previousX = x;
        previousY = y;
        previousAngle = angle;
    }
    //This function gets the spline to the constant angle
    public void splineToConstantAngle(double x, double y, double angle, double speedRatio){
        Waypoint waypoint = new Waypoint(x,y,angle,speedRatio,true);
        waypoints.add(waypoint);
        splineCount ++;
        previousX = x;
        previousY = y;
        previousAngle = angle;
    }
    //This function resets it
    public void resetGeneration(){
        waypoints = new ArrayList<Waypoint>();
        splineCount = -1;

    }
    /**
     * Add a wait to the route so that the robot paused in it's path
     * @param x
     * @param y
     * @param angle
     * @param time
     */
    //This chill function is basically the wait time
    public void chill(double x, double y, double angle, double time){
        Waypoint waypoint = new Waypoint(x,y,angle,1,true);
        Waypoint waitpoint = new Waypoint(x,y,angle,time);
        waypoints.add(waypoint);
        waypoints.add(waitpoint);
        splineCount += 2;
    }
    public void chill(double time){
        Waypoint waypoint = new Waypoint(previousX,previousY,previousAngle,time);
        waypoints.add(waypoint);
        splineCount++;
    }
    //This adds the robot action and the trigger time
    public void addAction(RobotActions.Actions actionType, double triggerTime){
        robotActions.addAction(actionType, triggerTime);
    }
    //This adds the action specifically the action type
    public void addAction(RobotActions.Actions actionType){
        robotActions.addAction(actionType, splineCount);
    }
    public Action[] compileActions(){
        return robotActions.compileActions();
    }
    //This is the function where it these everything
    public CubicSplineSegment[] generatePath() {
        if (waypoints.size() < 2) {
            return new CubicSplineSegment[0];
        }

        double time = 0;
        ArrayList<CubicSplineSegment> path = new ArrayList<CubicSplineSegment>();

        for (int i = 0; i < waypoints.size(); i++) {

            Waypoint current = waypoints.get(i);
            if (current.isWaitPoint()){
                CubicSplineSegment hold = new CubicSplineSegment(current,time, current.getDuration());
                path.add(hold);
                time = hold.getEndTime();
                continue;
            }

            if (i == 0){
                continue;
            }
            if (i+1>=waypoints.size()){
                continue;
            }

            Waypoint previous = (i-2 >= 0)? waypoints.get(i-2):waypoints.get(0);
            Waypoint start = waypoints.get(i-1);
            Waypoint end = waypoints.get(i);
            Waypoint next = (i+1 < waypoints.size())? waypoints.get(i+1):end;

            // get the travel distance
            double distance = Math.hypot(end.getX()-start.getX(),end.getY()-start.getY());
            final double distanceThreshold = 1e-3;
            if (distance <= distanceThreshold) {
                CubicSplineSegment hold = new CubicSplineSegment(start, time, minHoldTime);
                path.add(hold);
                time = hold.getEndTime();
                continue;
            }

            CubicSplineSegment spline = new CubicSplineSegment(previous,start,end,next, time, robotSpeed, end.shouldHoldAngle());
            path.add(spline);
            time = spline.getEndTime();
        }


        return path.toArray(new CubicSplineSegment[0]);
    }

}