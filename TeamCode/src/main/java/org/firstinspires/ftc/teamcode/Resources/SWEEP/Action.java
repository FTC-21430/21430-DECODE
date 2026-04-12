package org.firstinspires.ftc.teamcode.Resources.SWEEP;

import org.firstinspires.ftc.teamcode.Firmware.DecodeBot;
import org.firstinspires.ftc.teamcode.Firmware.OperatorStateMachine;
import org.firstinspires.ftc.teamcode.Firmware.Systems.SpindexerColorSensor;

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
            case LAUNCH:
                actionLaunch();
                break;
            case SCAN_MOTIF:
                actionScanMotif();
                break;
            case READY_RAMP:
                actionReadyRamp();
                break;
            case IDLE:
                actionIdle();
                break;
            case PREPPING:
                actionPrep();
                break;
            case REV_FAR:
                actionRevFar();
                break;
            case REV_CLOSE:
                actionRevClose();
                break;
            case SET_LAUNCHER:
                actionSetLauncher();
                break;
            case TOGGLE_GOAL_AIMING:
                actionToggleGoalAiming();
        }
    }

    // Action run methods

    private void actionIntake() {
        robot.operatorStateMachine.moveToState(OperatorStateMachine.State.INTAKE);
    }
    private void actionLaunch(){
        robot.operatorStateMachine.moveToState(OperatorStateMachine.State.LAUNCH);
    }
    private void actionPrep(){
        if (robot.motifId == 21){
            robot.operatorStateMachine.addToQueue(SpindexerColorSensor.COLORS.GREEN);
            robot.operatorStateMachine.addToQueue(SpindexerColorSensor.COLORS.PURPLE);
            robot.operatorStateMachine.addToQueue(SpindexerColorSensor.COLORS.PURPLE);
        }
        if (robot.motifId == 22){
            robot.operatorStateMachine.addToQueue(SpindexerColorSensor.COLORS.PURPLE);
            robot.operatorStateMachine.addToQueue(SpindexerColorSensor.COLORS.GREEN);
            robot.operatorStateMachine.addToQueue(SpindexerColorSensor.COLORS.PURPLE);
        }
        if (robot.motifId == 23){
            robot.operatorStateMachine.addToQueue(SpindexerColorSensor.COLORS.PURPLE);
            robot.operatorStateMachine.addToQueue(SpindexerColorSensor.COLORS.PURPLE);
            robot.operatorStateMachine.addToQueue(SpindexerColorSensor.COLORS.GREEN);
        }
        robot.operatorStateMachine.moveToState(OperatorStateMachine.State.PREPPING);
    }
    private void actionScanMotif(){
       robot.scanMotif();
    }

    private void actionIdle(){
        robot.intake.turnOff();
    }
    private void actionReadyRamp(){
        robot.trajectoryKinematics.calculateTrajectory(robot.trajectoryKinematics.getDistance(robot.alliance,robot.odometry.getRobotX(),robot.odometry.getRobotY()), robot.launcher.getFlywheelError());
        robot.launcher.setLaunchAngle(robot.trajectoryKinematics.getInitialAngle());
    }
    private void actionRevFar(){
        robot.trajectoryKinematics.calculateTrajectory(95,0);
        robot.launcher.setSpeed(robot.trajectoryKinematics.getLaunchMagnitude());
    }
    private void actionRevClose(){
        robot.trajectoryKinematics.calculateTrajectory(30,0);
        robot.launcher.setSpeed(robot.trajectoryKinematics.getLaunchMagnitude());
    }
    private void actionSetLauncher(){
        robot.trajectoryKinematics.updateVelocities(robot.odometry.getVelocityX(),robot.odometry.getVelocityY());
        robot.trajectoryKinematics.calculateTrajectory(robot.trajectoryKinematics.getDistance(robot.alliance,robot.odometry.getRobotX(),robot.odometry.getRobotY()), robot.launcher.getFlywheelError());
        robot.launcher.setSpeed(robot.trajectoryKinematics.getLaunchMagnitude());
        robot.launcher.setLaunchAngle(robot.trajectoryKinematics.getInitialAngle());
    }
    private void actionToggleGoalAiming(){
        // toggles the aiming flag. should not impact the existent SWEEP rotation code unless this action is called.
        robot.setSweepAimingAtGoal(!robot.shouldSWEEPAimAtGoal());
    }
}