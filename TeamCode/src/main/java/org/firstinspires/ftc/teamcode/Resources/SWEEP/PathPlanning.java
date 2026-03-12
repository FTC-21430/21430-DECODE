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
    private int splineCount = -1;
    public static double robotSpeed = 4; // my guess of 20 inches / s
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
        if (waypoints.size() < 2) {
            return new CubicSplineSegment[0];
        }

        double time = 0;
        ArrayList<CubicSplineSegment> path = new ArrayList<CubicSplineSegment>();

        // Separate wait points from spline control points so each type is processed correctly.
        // Wait points are NOT control points — they must not be used as prev/start/end/next
        // for spline coefficient calculation, only as time separators between spline segments.
        for (int i = 0; i < waypoints.size(); i++) {
            Waypoint current = waypoints.get(i);

            // Hold segment: just insert a timed pause at this position, then continue
            if (current.isWaitPoint()) {
                CubicSplineSegment spline = new CubicSplineSegment(current, time, current.getDuration());
                path.add(spline);
                time = spline.getEndTime();
                continue;
            }

            // Find the next non-wait waypoint to be the end of this spline segment
            int endIdx = -1;
            for (int j = i + 1; j < waypoints.size(); j++) {
                if (!waypoints.get(j).isWaitPoint()) {
                    endIdx = j;
                    break;
                }
            }

            // No non-wait waypoint follows this one — nothing left to spline to
            if (endIdx == -1) continue;

            // Find prev: nearest non-wait waypoint before i (or duplicate start if none)
            int prevIdx = i;
            for (int j = i - 1; j >= 0; j--) {
                if (!waypoints.get(j).isWaitPoint()) {
                    prevIdx = j;
                    break;
                }
            }

            // Find next: nearest non-wait waypoint after endIdx (or duplicate end if none)
            int nextIdx = endIdx;
            for (int j = endIdx + 1; j < waypoints.size(); j++) {
                if (!waypoints.get(j).isWaitPoint()) {
                    nextIdx = j;
                    break;
                }
            }

            Waypoint prev  = waypoints.get(prevIdx);
            Waypoint start = waypoints.get(i);
            Waypoint end   = waypoints.get(endIdx);
            Waypoint next  = waypoints.get(nextIdx);

            CubicSplineSegment spline = new CubicSplineSegment(prev, start, end, next, time, robotSpeed, end.shouldHoldAngle());
            time = spline.getEndTime();
            path.add(spline);

            // Skip i forward to endIdx so the next iteration starts from the end waypoint,
            // correctly chaining segments without double-processing any waypoint.
            // The loop's i++ will then advance past it to the waypoint after endIdx.
            i = endIdx - 1;
        }

        return path.toArray(new CubicSplineSegment[0]);
    }

}