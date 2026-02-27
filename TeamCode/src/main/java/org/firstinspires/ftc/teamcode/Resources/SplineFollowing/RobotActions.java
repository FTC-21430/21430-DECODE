package org.firstinspires.ftc.teamcode.Resources.SplineFollowing;

import org.firstinspires.ftc.teamcode.Firmware.DecodeBot;
import org.firstinspires.ftc.teamcode.Firmware.OperatorStateMachine;

/**
 * Don't have a super solid plan for this yet, but it needs to be involved with the PathPlanning class to add to an action cue alongside a spline movement
 *
 * The actions contained within this class are robot specific, but lets try to make it easy to change and reuse for future seasons! if someone has a better idea, LMK! - Tobin
 */
public class RobotActions {

    public enum Actions {
        INTAKE,
        LAUNCH_ONE,
        LAUNCH_TWO,
        LAUNCH_THREE,
        LAUNCH_SORTED_THREE,
        SCAN_MOTIF,
        READY_RAMP
        //TODO: add extra possible actions the robot's mechanisms could do during autonomous to this list
    }
    public RobotActions(DecodeBot decodeBot){

    }
}

/**
 * assist class for one action that robot is commanded to do
 */
class Action {
    private final RobotActions.Actions actionType;
    private final triggerConditions triggerMode;
    private double triggerTime = 0.0;
    private int triggerSplineOrder = 0;
    private final DecodeBot robot;
    private enum triggerConditions {
        SPLINE_ORDER,
        TIMING
    }
    public Action(RobotActions.Actions actionType, DecodeBot robot, int splineOrder){
        this.actionType = actionType;
        triggerMode = triggerConditions.SPLINE_ORDER;
        triggerSplineOrder = splineOrder;
        this.robot = robot;
    }
    public Action(RobotActions.Actions actionType, DecodeBot robot, double triggerTime){
        this.actionType = actionType;
        triggerMode = triggerConditions.TIMING;
        this.triggerTime = triggerTime;
        this.robot = robot;
    }

    public boolean checkAction(int splineID, double Time){
        switch (triggerMode){
            case SPLINE_ORDER:
                if (splineID >= triggerSplineOrder){
                    actionIntake();
                    return true;
                }
                break;
            case TIMING:
                // about the same as above
                break;
        }
        // return true if the action was triggered, false if it was not
        return false;
    }
    public void triggerAction(){
        switch (actionType){
            // all the types, have a function for each of these
            case INTAKE:
                actionIntake();
        }
    }


    // action run methods

    // action run methods should look like this!
    //TODO: made an action run method for EVERY action the robot needs to be able to do.
    private void actionIntake(){
        robot.operatorStateMachine.moveToState(OperatorStateMachine.State.INTAKE);
    }


}


