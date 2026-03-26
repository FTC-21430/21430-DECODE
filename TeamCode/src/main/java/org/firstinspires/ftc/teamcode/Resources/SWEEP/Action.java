package org.firstinspires.ftc.teamcode.Resources.SWEEP;

import org.firstinspires.ftc.teamcode.Firmware.DecodeBot;
import org.firstinspires.ftc.teamcode.Firmware.OperatorStateMachine;

/**
 * Assist class for one action that robot is commanded to do.
 */
public class Action {
    private final RobotActions.Actions actionType;
    private final TriggerConditions triggerMode;
    private double triggerTime = 0.0;
    private int triggerSplineOrder = 0;
    private final DecodeBot robot;

    private enum TriggerConditions {
        SPLINE_ORDER,
        TIMING
    }

    public Action(RobotActions.Actions actionType, DecodeBot robot, int splineOrder) {
        this.actionType = actionType;
        this.triggerMode = TriggerConditions.SPLINE_ORDER;
        this.triggerSplineOrder = splineOrder;
        this.robot = robot;
    }

    public Action(RobotActions.Actions actionType, DecodeBot robot, double triggerTime) {
        this.actionType = actionType;
        this.triggerMode = TriggerConditions.TIMING;
        this.triggerTime = triggerTime;
        this.robot = robot;
    }

    public boolean checkAction(int splineID, double time) {
        switch (triggerMode) {
            case SPLINE_ORDER:
                if (splineID >= triggerSplineOrder) {
                    
                    return true;
                }
                break;
            case TIMING:
                // timing-based trigger: fire when runtime time >= triggerTime
                if (time >= triggerTime) {
                    return true;
                }
                break;
        }
        return false; // Return true if the action was triggered, false otherwise
    }

    public void triggerAction() {
        switch (actionType) {
            case INTAKE:
                actionIntake();
                break;
            case LAUNCH_THREE:
                actionLaunch();
                break;
            case LAUNCH_SORTED_THREE:
                actionSortedLaunch();
                break;
            case SCAN_MOTIF:
                actionScanMotiff();
                break;
            case IDLE:
                actionIdle();
                break;
        }
    }

    // Action run methods

    private void actionIntake() {
        robot.intake.turnOn();
    }
    private void actionLaunch(){robot.operatorStateMachine.moveToState(OperatorStateMachine.State.LAUNCH);}
    private void actionSortedLaunch(){robot.spindexer.storeColorAtIndex();}
    private void actionScanMotiff(){robot.aprilTags.getMotifID();}

    private void actionIdle(){
        robot.intake.turnOff();
    }
}