package org.firstinspires.ftc.teamcode.Resources.SplineFollowing;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.ejml.simple.SimpleMatrix;

/**
 * Handle spine segment switching and interpolation to get the robot's target position at any given time.
 * Executes robot actions at specified times or spline indexes along the path, allowing for complex autonomous routines.
 * Also handles time delays and program speed adjustments for testing purposes.
 *
 */
public class SplinePathInterpreter {
    private final ElapsedTime runtime;
    private double timeDelay = 0;
    private double programSpeed = 1.0;
    private int currentSplineIndex = 0;
    private CubicSplineSegment[] path;
    private Action[] actions;
    public SplinePathInterpreter(double programSpeedDEBUG){
        this.programSpeed = programSpeedDEBUG > 0.0 ? programSpeedDEBUG : 0.1;
        this.runtime = new ElapsedTime();
    }
    public SplinePathInterpreter(){

        this.runtime = new ElapsedTime();
    }
    public void startPath(CubicSplineSegment[] path, Action[] actions){
        this.path = path;
        this.actions = actions;
        timeDelay = 0.0;
        runtime.reset();
    }
    public void startPath(CubicSplineSegment[] path, Action[] actions, double startTime){
        this.path = path;
        this.actions = actions;
        this.timeDelay = startTime >= 0 ? startTime : 0.0;
        runtime.reset();
    }

    public SimpleMatrix getRobotPosition(double lookaheadTime){
        lookaheadTime = lookaheadTime >= 0 ? lookaheadTime : 0.0;
        double time = runtime.seconds() + timeDelay + lookaheadTime;
        time *= programSpeed;
        CubicSplineSegment splineSegment = getCurrentSpline(time);
        return new SimpleMatrix(new double[]{
                splineSegment.getX(time), // x - inches
                splineSegment.getY(time), // y - inches
                splineSegment.getRotation(time) // yaw - degrees
        });
    }
    public void executeActions(){
        Action queuedAction = actions[0];
        if (queuedAction.checkAction(currentSplineIndex,runtime.seconds() + timeDelay)){
            actions = completeAction(actions);
        }
    }
    private CubicSplineSegment getCurrentSpline(double time){
        // start with the last index we used, as it is likely that, else do a search to find the current segment
        CubicSplineSegment currentSpline = path[currentSplineIndex];
        if (checkIfSplineIsCurrentlyActive(currentSpline, time)) return currentSpline;

        // the next most likely case is that we have moved onto the next spline
        if (currentSplineIndex+1 < path.length) {
            CubicSplineSegment nextSpline = path[currentSplineIndex + 1];
            currentSplineIndex += 1;
            if (checkIfSplineIsCurrentlyActive(nextSpline,time)) return nextSpline;
        }

        // now we need to just do a linear search on the total path to find the current path.
        for (int i = 0; i < path.length; i++) {
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
        for (int i = 0; i < actions.length; i++){
            if (i != 0){
                result[i] = actions[i];
            }
        }
        return result;
    }
}
