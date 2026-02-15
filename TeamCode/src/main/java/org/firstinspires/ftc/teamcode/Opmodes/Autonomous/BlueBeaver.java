package org.firstinspires.ftc.teamcode.Opmodes.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Firmware.OperatorStateMachine;
import org.firstinspires.ftc.teamcode.Firmware.Systems.Spindexer;
import org.firstinspires.ftc.teamcode.Firmware.Systems.SpindexerColorSensor;
import org.firstinspires.ftc.teamcode.Opmodes.BaseAuto;
@Autonomous
public class BlueBeaver extends BaseAuto {

    private void sortedLaunchClose(boolean finalLaunch, boolean firstLaunch) {
        robot.setLauncherBasedOnTags();

        //The robot moves to the launch zone and it launches the three balls
        if (!finalLaunch) {
            if (firstLaunch){
                robot.autoMoveTo(-17.5, -25, -127, 20);
            }else{
                robot.autoMoveTo(-17.5, -26, -127, 20);
            }

        }else{
            robot.autoMoveTo(-35, -18, -116, 20);
        }

        robot.aimAtGoal();
        robot.setLauncherBasedOnTags();
        robot.chill(true,0.35);
        autonomousLaunching(motifId);
    }
    private void sortedLaunchFar(boolean finalLaunch, boolean firstLaunch) {
        robot.autoMoveTo(52.3, -16.9, -160.9, 20);


        robot.aimAtGoal();
        robot.chill(true,0.08);
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
        robot.odometry.overridePosition(-49.82,-54.5782,-125.08);
        robot.spindexer.setIndexOffset(Spindexer.INDEX_TYPE.NONE);
        robot.launcher.revFlywheel();
        robot.chill(false,0.2);
        //This is the position that the robot moves to to shoot the first three balls
        motifId = 0;
        robot.aimAtGoal();
        robot.chill(true,0.2);
        sortedLaunchClose(false, true);
        detectMotifWhileMoveTo(-39,-21,-210,4);
        robot.pathFollowing.setFollowSpeed(1);

        robot.operatorStateMachine.moveToState(OperatorStateMachine.State.INTAKE);
        robot.autoMoveTo(-12,-23,-272,3);
        robot.chill(true,0.3);
        robot.pathFollowing.setFollowSpeed(1);
        robot.autoMoveTo(-12.5,-50.8,-270,4);

        robot.pathFollowing.setFollowSpeed(1);
        //The robot bumps the gate
        robot.autoMoveTo(-15.1,-42.2,-270,3);

        robot.autoMoveTo(-6,-44,-180,2.5);
        robot.setLauncherBasedOnTags();
        robot.autoMoveTo(-4.5,-53.3,-180,3);
        chillAndDetect(true,0.7);
        sortedLaunchClose(false, false);

        //The robot moves to the place to intake the balls
        robot.operatorStateMachine.moveToState(OperatorStateMachine.State.INTAKE);
        robot.autoMoveTo(13,-22,-270,8);

        robot.setLauncherBasedOnTags();
        robot.autoMoveTo(13,-60.8,-270,4);
        robot.launcher.setSpeed(1400);
        robot.launcher.setLaunchAngle(40);
        robot.autoMoveTo(13,-31,-210,14);
        robot.setLauncherBasedOnTags();
        robot.chill(true,0.4);
        sortedLaunchClose(false, false);
//
//        // sorted cycle 2
//
//        //The robot moves to the place to intake the balls
        robot.operatorStateMachine.moveToState(OperatorStateMachine.State.INTAKE);
        robot.autoMoveTo(38,-28,-270,6);
        robot.setLauncherBasedOnTags();
        robot.autoMoveTo(38,-60.1,-270,3);
        robot.launcher.setSpeed(1400);
        robot.chill(true,0.4);


//
        sortedLaunchClose(true,false);
//
        robot.autoMoveTo(-50, -18, -90, 4);
        robot.chill(true,0.2);


    }
}
