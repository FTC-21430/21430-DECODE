package org.firstinspires.ftc.teamcode.Opmodes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Firmware.OperatorStateMachine;
import org.firstinspires.ftc.teamcode.Firmware.Systems.Spindexer;
import org.firstinspires.ftc.teamcode.Firmware.Systems.SpindexerColorSensor;
import org.firstinspires.ftc.teamcode.Opmodes.BaseAuto;
@Autonomous
public class BlueFar6And9 extends BaseAuto {
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
        robot.chill(true,0.1);
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
        runtime.reset();
        //This is the starting location of the robot
        robot.odometry.overridePosition(62.7,-31.2,-90);
        robot.spindexer.setIndexOffset(Spindexer.INDEX_TYPE.NONE);
        //This is the position that the robot moves to to shoot the first three balls
        motifId = 0;
        robot.launcher.revFlywheel();
        robot.aimAtGoal();
        robot.chill(true,0.2);
        sortedLaunchFar(false, true);
        //Move to corner set
        robot.autoMoveTo(56,-23,-200,6);
        robot.operatorStateMachine.moveToState(OperatorStateMachine.State.INTAKE);
        robot.autoMoveTo(59,-51,-270,6);
        robot.autoMoveTo(63,-57.5,-270,3);
        robot.chill(true,0.3);

        //Moves to the mid zone and launches the second set
        detectMotifWhileMoveTo(51.3,-27.2,-265,14);
        robot.launcher.revFlywheel();
        detectMotifWhileMoveTo(20,-18.2,-210,14);
        robot.launcher.setSpeed(1300);
        robot.launcher.setLaunchAngle(52);

        // Launch the second set

        robot.aimAtGoal();
        sortedLaunchClose(false,true);
        robot.operatorStateMachine.moveToState(OperatorStateMachine.State.INTAKE);
        robot.autoMoveTo(-12,-23,-272,3);
        robot.chill(true,0.1);
        robot.pathFollowing.setFollowSpeed(1);
        robot.autoMoveTo(-12.5,-50.8,-270,4);

        robot.pathFollowing.setFollowSpeed(1);
        //The robot bumps the gate
        robot.autoMoveTo(-15.1,-42.2,-270,3);

        robot.autoMoveTo(-6,-44,-180,2.5);
        robot.setLauncherBasedOnTags();
        robot.autoMoveTo(-4.5,-53.3,-180,3);
        chillAndDetect(true,0.64);
        sortedLaunchClose(false, false);

        //The robot moves to the place to intake the balls
        robot.operatorStateMachine.moveToState(OperatorStateMachine.State.INTAKE);
        robot.autoMoveTo(13,-22,-270,8);

        robot.setLauncherBasedOnTags();
        robot.autoMoveTo(13,-60.8,-270,4);
        robot.launcher.setSpeed(1650);
        robot.launcher.setLaunchAngle(32);
        robot.autoMoveTo(13,-31,-210,14);
        robot.setLauncherBasedOnTags();
        robot.chill(true,0.25);
        sortedLaunchFar(false, false);
//
//        // sorted cycle 2
//
//        //The robot moves to the place to intake the balls
        robot.operatorStateMachine.moveToState(OperatorStateMachine.State.INTAKE);
        robot.autoMoveTo(38,-28,-270,6);
        robot.setLauncherBasedOnTags();
        robot.autoMoveTo(38,-60.1,-270,3);
        robot.launcher.setSpeed(1700);
        robot.chill(true,0.25);


//
        sortedLaunchFar(true,false);
//
//        // sorted cycle 3
//
////      park off of launch line and close to the gate to clear the classifier at teleop start
        robot.autoMoveTo(30,-22,-170,5);
        robot.chill(true,0.05);
    }
}