package org.firstinspires.ftc.teamcode.Opmodes.Autonomous;

import org.firstinspires.ftc.teamcode.Firmware.OperatorStateMachine;
import org.firstinspires.ftc.teamcode.Firmware.Systems.Spindexer;
import org.firstinspires.ftc.teamcode.Firmware.Systems.SpindexerColorSensor;
import org.firstinspires.ftc.teamcode.Opmodes.BaseAuto;

public class BlueFar6And9 extends BaseAuto {
    private void sortedLaunch(boolean finalLaunch, boolean firstLaunch) {
        if (!firstLaunch) {
            robot.autoMoveTo(52,-19,10,2);
        }
        if (!finalLaunch) {
            if (firstLaunch) {
                robot.autoMoveTo(52, -19, 10, 8);
            } else {
                robot.autoMoveTo(52, -19, 10, 8);
            }

        } else {
            robot.autoMoveTo(52, -19, 10, 8);
        }
        if (!firstLaunch) {
            robot.chill(true, 0.8);
        } else {
            robot.chill(true, 0.3);
        }
        robot.aimAtGoal();
        robot.setLauncherBasedOnTags();

        robot.chill(true, 0.3);
        robot.aimAtGoal();
        autonomousLaunching(motifId);
//
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true, true,false);
        robot.setAlliance("blue");
        robot.odometry.recalibrateIMU();
        robot.spindexer.setColorIndexing(SpindexerColorSensor.COLORS.GREEN, SpindexerColorSensor.COLORS.PURPLE, SpindexerColorSensor.COLORS.PURPLE);

        waitForStart();
        //This is the starting location of the robot
        robot.odometry.overridePosition(64,-13,0);
        robot.spindexer.setIndexOffset(Spindexer.INDEX_TYPE.NONE);
        robot.chill(false,0.2);
        //This is the position that the robot moves to to shoot the first three balls
        motifId = 0;
        robot.aimAtGoal();
        robot.chill(true,0.2);
        sortedLaunch(false, true);
        //Move to corner set
        //robot.autoMoveTo(,,,);
        //Intake
        robot.operatorStateMachine.moveToState(OperatorStateMachine.State.INTAKE);
        //Move to launch zone
        //robot.autoMoveTo(,,,);
        //Launch
        //robot.autoMoveTo(,,,);
        robot.operatorStateMachine.moveToState(OperatorStateMachine.State.LAUNCH);
        //Intake 2nd set
        //robot.autoMoveTo(,,,);
        robot.operatorStateMachine.moveToState(OperatorStateMachine.State.INTAKE);
        //Open gate
        //robot.autoMoveTo(,,,);
        //Launch 2nd from close zone
        //robot.autoMoveTo(,,,);
        robot.operatorStateMachine.moveToState(OperatorStateMachine.State.LAUNCH);
        //Intake closest set
        //robot.autoMoveTo(,,,);
        robot.operatorStateMachine.moveToState(OperatorStateMachine.State.INTAKE);
        //Launch
        //robot.autoMoveTo(,,,);
        robot.operatorStateMachine.moveToState(OperatorStateMachine.State.LAUNCH);
        //Intake 3rd set
        //robot.autoMoveTo(,,,);
        robot.operatorStateMachine.moveToState(OperatorStateMachine.State.INTAKE);
        //Launch from far
        //robot.autoMoveTo(,,,);
        robot.operatorStateMachine.moveToState(OperatorStateMachine.State.LAUNCH);
        //move off line
        //robot.autoMoveTo(,,,);
    }
}