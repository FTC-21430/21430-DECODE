package org.firstinspires.ftc.teamcode.Resources.SWEEP;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.ejml.simple.SimpleMatrix;
import java.util.Arrays;

/**
 * Handle spline segment switching and interpolation to get the robot's target position at any given time.
 * Executes robot actions at specified times and /  or spline indexes along the path, allowing for complex autonomous routines.
 * Also handles time delays and program speed adjustments for testing purposes.
 */
public class SplinePathInterpreter {
    private final ElapsedTime runtime;
    private double timeDelay = 0;
    public static double programSpeed = 1.0; // FTC dashboard variable for changing follow speed on the fly - must be public and static for this.
    private int currentSplineIndex = 0;
    private CubicSplineSegment[] path;
    private Action[] actions;
    public SplinePathInterpreter(double programSpeedDEBUG){
        this.programSpeed = programSpeedDEBUG > 0.0 ? programSpeedDEBUG : 0.1;
        this.runtime = new ElapsedTime();
    }
    public SplinePathInterpreter(){
        this.programSpeed = 1.0;
        this.runtime = new ElapsedTime();
    }
    public void startPath(CubicSplineSegment[] path, Action[] actions){
        this.path = path;
        this.actions = actions;
        timeDelay = 0.0;
        runtime.reset();
        currentSplineIndex = 0;
    }
    public void startPath(CubicSplineSegment[] path, Action[] actions, double startTime){
        // varify valid path before beginning
        if (path == null || path.length == 0) return;

        this.path = path;
        this.actions = actions;
        this.timeDelay = startTime >= 0 ? startTime : 0.0;
        runtime.reset();
        currentSplineIndex = 0;
    }

    public SimpleMatrix getRobotPosition(double lookaheadTime){
        if (path == null || path.length == 0) return new SimpleMatrix(new double[]{0,0,0});
        lookaheadTime = lookaheadTime >= 0 ? lookaheadTime : 0.0;
        double scaledTime = (runtime.seconds() + timeDelay + lookaheadTime) * SplinePathInterpreter.programSpeed;
        CubicSplineSegment segment = getCurrentSpline(scaledTime);
        return new SimpleMatrix(new double[]{
                segment.getX(scaledTime),
                segment.getY(scaledTime),
                segment.getRotation(scaledTime)
        });
    }
    public void executeActions(){
        if (actions == null || actions.length == 0) return;
        Action queuedAction = actions[0];
        double scaledTime = (runtime.seconds() + timeDelay) * SplinePathInterpreter.programSpeed;
        if (queuedAction.checkAction(currentSplineIndex, scaledTime)){
            actions = completeAction(actions);
        }
    }
    public boolean isPathFinished(){
        if (path == null || path.length == 0) return true;
        double scaledTime = (runtime.seconds() + timeDelay) * SplinePathInterpreter.programSpeed;
        CubicSplineSegment last = path[path.length - 1];
        return scaledTime >= last.getEndTime();
    }
    public void setProgramSpeed(double speedRatio){
        programSpeed = speedRatio;
    }
    private CubicSplineSegment getCurrentSpline(double time){
        if (path == null || path.length == 0) {
            // if there is no valid path
            Waypoint emptyWaypoint = new Waypoint(0,0,0,0,false);
            return new CubicSplineSegment(emptyWaypoint,0.0,1);
        }

        // start with the last index we used, as it is likely that, else do a search to find the current segment
        CubicSplineSegment currentSpline = path[currentSplineIndex];
        if (checkIfSplineIsCurrentlyActive(currentSpline, time)) return currentSpline;

        // the next most likely case is that we have moved onto the next spline
        if (currentSplineIndex+1 < path.length) {
            CubicSplineSegment nextSpline = path[currentSplineIndex + 1];
            if (checkIfSplineIsCurrentlyActive(nextSpline,time)) {
                currentSplineIndex += 1;
                return nextSpline;
            }
        }

        // now we need to just do a linear search on the total path to find the current path.
        for (int i = 0; i < path.length; i++){
            CubicSplineSegment spline = path[i];
            if (checkIfSplineIsCurrentlyActive(spline, time)) {
                currentSplineIndex = i;
                return spline;
            }
        };
        currentSplineIndex = path.length -1 ;
        return path[currentSplineIndex]; // return the last spline if all else fails :(
    }
    private boolean checkIfSplineIsCurrentlyActive(CubicSplineSegment spline, double time){
        return time >= spline.getStartTime() && time < spline.getEndTime();
    }
    private Action[] completeAction(Action[] actions){
        actions[0].triggerAction();
        Action[] result = new Action[actions.length-1];
        for (int i = 1; i < actions.length; i++){
            result[i-1] = actions[i];
        }
        return result;
    }
}