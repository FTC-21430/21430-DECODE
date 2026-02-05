package org.firstinspires.ftc.teamcode.Opmodes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Firmware.OperatorStateMachine;
import org.firstinspires.ftc.teamcode.Firmware.Systems.Spindexer;
import org.firstinspires.ftc.teamcode.Firmware.Systems.SpindexerColorSensor;
import org.firstinspires.ftc.teamcode.Opmodes.BaseAuto;

// This Autonomous is for when the robot starts in the close zone
@Autonomous
public class RedElephant extends BaseAuto {
    private void sortedLaunch(boolean finalLaunch, boolean firstLaunch){
        if (!firstLaunch) {
            robot.autoMoveTo(-15.5, 30, 195, 25);
        }
//        robot.autoMoveTo(0, 40, 135, 2);
        //The robot moves to the launch zone and it launches the three balls
        if (!finalLaunch) {
            if (firstLaunch){
                robot.autoMoveTo(-15.5, 19, 132, 8);
            }else{
                robot.autoMoveTo(-15.5, 19, 134, 8);
            }

        }else{
            robot.autoMoveTo(-35, 18, 116, 4);
        }
        if (!firstLaunch) {
            robot.chill(true, 0.8);
        }else{
            robot.chill(true,0.3);
        }
        robot.aimAtGoal();
        robot.setLauncherBasedOnTags();

        robot.chill(true,0.3);
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
        //This is the position that the robot moves to to shoot the first three balls
        motifId = 0;
        robot.aimAtGoal();
        robot.chill(true,0.2);
        sortedLaunch(false, true);

        //move off the line
        robot.autoMoveTo(-50, 18, 90, 4);
        robot.chill(true,0.2);
    }
}