package org.firstinspires.ftc.teamcode.Resources.SplineFollowing;

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
                    actionIntake();
                    return true;
                }
                break;
            case TIMING:
                // timing-based trigger: fire when runtime time >= triggerTime
                if (time >= triggerTime) {
                    actionIntake();
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
            // Add cases for other actions here
        }
    }

    // Action run methods

    private void actionIntake() {
        if (robot != null && robot.operatorStateMachine != null) {
            robot.operatorStateMachine.moveToState(OperatorStateMachine.State.INTAKE);
        }
    }
}