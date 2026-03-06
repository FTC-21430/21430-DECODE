package org.firstinspires.ftc.teamcode.Resources.SWEEP;

import org.firstinspires.ftc.teamcode.Firmware.DecodeBot;
import org.firstinspires.ftc.teamcode.Firmware.Robot;

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
    private double robotSpeed = 0;
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
    public void splineTo(double x, double y, double speedRatio){
        Waypoint waypoint = new Waypoint(x,y,0,speedRatio, false);
        waypoints.add(waypoint);
        splineCount ++;
    }
    public void splineStart(double x, double y, double angle){
        Waypoint waypoint = new Waypoint(x,y,angle,0,false);
        waypoints.add(waypoint);
        splineCount++;
    }
    public void splineToConstantAngle(double x, double y, double angle, double speedRatio){
        Waypoint waypoint = new Waypoint(x,y,angle,speedRatio,true);
        waypoints.add(waypoint);
        splineCount ++;
    }
    public void resetGeneration(){
        waypoints = new ArrayList<Waypoint>();
        splineCount = -1;

    }
    /**
     * Add a wait to the route so that the robot paused in it's path
     * @param time
     */
    public void chill(double x, double y, double angle, double time){
        Waypoint waypoint = new Waypoint(x,y,angle,time);
        waypoints.add(waypoint);
        splineCount ++;
    }
    public void addAction(RobotActions.Actions actionType, double triggerTime){
        robotActions.addAction(actionType, triggerTime);
    }
    public void addAction(RobotActions.Actions actionType){
        robotActions.addAction(actionType, splineCount);
    }
    public Action[] compileActions(){
        return robotActions.compileActions();
    }
    public CubicSplineSegment[] generatePath() {
        //A if statement that just returns 0 if it can't identify more than
        if (waypoints.size()<2){
            return new CubicSplineSegment[0];
        }
         double time = 0;
        CubicSplineSegment[] path = new CubicSplineSegment[waypoints.size()-1];
        //The first part of this for loop is for having the previous, start, end and next point
        for (int i = 0; i <  waypoints.size(); i++) {
            if(waypoints.get(i).isWaitPoint()){
                CubicSplineSegment spline = new CubicSplineSegment(waypoints.get(i),time,waypoints.get(i).getDuration());
                path[i] = spline;
                time = spline.getEndTime();
                continue;
            }
            if (i  >= waypoints.size()-1){
                continue;
            }
            //For the previous and the new idx the ? is basically like a else statment
            int prevIdx = (i - 1 >= 0) ? i - 1 : 0;
            int startIdx = i;
            int endIdx = i + 1;
            int nextIdx = (i + 2 < waypoints.size()) ? i + 2 : endIdx;

            //This is when it actually applies the logic in order to get the points it needs
            Waypoint prev = waypoints.get(prevIdx);
            Waypoint start = waypoints.get(startIdx);
            Waypoint end = waypoints.get(endIdx);
            Waypoint next = waypoints.get(nextIdx);
            CubicSplineSegment spline = new CubicSplineSegment(prev, start, end, next, time, robotSpeed);
            time = spline.getEndTime();
            path[i] = spline;
            }
        return path;

    }

}