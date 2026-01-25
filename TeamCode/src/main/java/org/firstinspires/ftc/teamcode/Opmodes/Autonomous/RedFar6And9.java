package org.firstinspires.ftc.teamcode.Opmodes.Autonomous;

import org.firstinspires.ftc.teamcode.Firmware.OperatorStateMachine;
import org.firstinspires.ftc.teamcode.Firmware.Systems.Spindexer;
import org.firstinspires.ftc.teamcode.Firmware.Systems.SpindexerColorSensor;
import org.firstinspires.ftc.teamcode.Opmodes.BaseAuto;

public class RedFar6And9 extends BaseAuto {
    private void sortedLaunch(boolean finalLaunch, boolean firstLaunch) {
        if (!firstLaunch) {
            robot.autoMoveTo(52,1,60,2);
        }
        if (!finalLaunch) {
            if (firstLaunch) {
                robot.autoMoveTo(52, 19, 7, 8);
            } else {
                robot.autoMoveTo(52, 19, 9, 8);
            }

        } else {
            robot.autoMoveTo(-35, 18, 116, 4);
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

        initialize(true, true);
        robot.setAlliance("red");
        robot.odometry.recalibrateIMU();
        robot.spindexer.setColorIndexing(SpindexerColorSensor.COLORS.GREEN, SpindexerColorSensor.COLORS.PURPLE, SpindexerColorSensor.COLORS.PURPLE);

        waitForStart();
        //This is the starting location of the robot
        robot.odometry.overridePosition(-49.82,54.5782,125.08);
        robot.spindexer.setIndexOffset(Spindexer.INDEX_TYPE.NONE);
        robot.chill(false,0.2);
        //This is the position that the robot moves to to shoot the first three balls
        motifId = 0;
        robot.aimAtGoal();
        robot.chill(true,0.2);
        sortedLaunch(false, true);
        detectMotifWhileMoveTo(-39,40,210,10);
        robot.operatorStateMachine.moveToState(OperatorStateMachine.State.INTAKE);
    }
}
