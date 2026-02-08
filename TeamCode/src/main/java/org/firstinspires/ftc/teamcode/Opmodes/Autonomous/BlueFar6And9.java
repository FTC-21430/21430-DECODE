package org.firstinspires.ftc.teamcode.Opmodes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Firmware.OperatorStateMachine;
import org.firstinspires.ftc.teamcode.Firmware.Systems.Spindexer;
import org.firstinspires.ftc.teamcode.Firmware.Systems.SpindexerColorSensor;
import org.firstinspires.ftc.teamcode.Opmodes.BaseAuto;

@Autonomous
public class BlueFar6And9 extends BaseAuto {
    private void sortedLaunchClose(boolean finalLaunch, boolean firstLaunch) {
        if (!firstLaunch) {
            robot.autoMoveTo(-15.5, -30, -195, 25);
        }
//        robot.autoMoveTo(0, 40, 135, 2);
        //The robot moves to the launch zone and it launches the three balls
        if (!finalLaunch) {
            if (firstLaunch){
                robot.autoMoveTo(-15.5, -19, -132, 8);
            }else{
                robot.autoMoveTo(-15.5, -19, -134, 8);
            }

        }else{
            robot.autoMoveTo(-35, -18, -116, 4);
        }

        robot.aimAtGoal();
        robot.setLauncherBasedOnTags();

        robot.chill(true,0.3);
        robot.aimAtGoal();
        autonomousLaunching(motifId);
    }
    private void sortedLaunchFar(boolean finalLaunch, boolean firstLaunch) {
        robot.autoMoveTo(52.3, -18.9, -162.9, 8);
        robot.chill(true, 0.3);

        robot.aimAtGoal();
        robot.setLauncherBasedOnTags();

        robot.chill(true, 0.3);
        robot.aimAtGoal();
        autonomousLaunching(motifId);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true, true,false);
        robot.setAlliance("red");
        robot.odometry.recalibrateIMU();
        robot.spindexer.setColorIndexing(SpindexerColorSensor.COLORS.GREEN, SpindexerColorSensor.COLORS.PURPLE, SpindexerColorSensor.COLORS.PURPLE);

        waitForStart();
        //This is the starting location of the robot
        robot.odometry.overridePosition(62.7,-31.2,-90);
        robot.spindexer.setIndexOffset(Spindexer.INDEX_TYPE.NONE);
        robot.chill(false,0.2);
        //This is the position that the robot moves to to shoot the first three balls
        motifId = 0;
        robot.launcher.revFlywheel();
        robot.aimAtGoal();
        robot.chill(true,0.0);
        sortedLaunchFar(false, true);
        //Move to corner set
        robot.autoMoveTo(56,-23,-200,6);
        robot.operatorStateMachine.moveToState(OperatorStateMachine.State.INTAKE);
        robot.autoMoveTo(59,-51,-270,6);
        robot.autoMoveTo(63,-58.1,-270,3);
        robot.chill(true,0.45);

        //Moves to the mid zone and launches the second set
        robot.autoMoveTo(51.3,-27.2,-265,5);
        robot.launcher.revFlywheel();
        robot.autoMoveTo(20,-18.2,-210,8);
        robot.launcher.revFlywheel();
        robot.autoMoveTo(-8.6,-18.2,-170,8);
        // Launch the second set

        robot.launcher.revFlywheel();
        robot.chill(true,0.2);

        robot.aimAtGoal();
        sortedLaunchClose(false,true);
        robot.operatorStateMachine.moveToState(OperatorStateMachine.State.INTAKE);
        robot.chill(true,0.2);
        robot.autoMoveTo(-12,-26,-272,3);
        robot.pathFollowing.setFollowSpeed(0.6);
        robot.autoMoveTo(-12.5,-50.8,-270,4);

        //The robot bumps the gate
        robot.autoMoveTo(-15.1,-42.2,-270,3);
        robot.pathFollowing.setFollowSpeed(1);
        robot.autoMoveTo(-5,-44,-180,2.5);
        robot.launcher.revFlywheel();
        robot.autoMoveTo(-3,-53.3,-180,3);
        chillAndDetect(true,0.55);
        sortedLaunchClose(false, false);

        //The robot moves to the place to intake the balls
        robot.operatorStateMachine.moveToState(OperatorStateMachine.State.INTAKE);
        robot.autoMoveTo(-8,-46,-176,6);

        robot.launcher.revFlywheel();
        robot.autoMoveTo(15,-46,-177,3);

        robot.chill(true, 0.2);
//
        sortedLaunchClose(false, false);

//        // sorted cycle 2

          //The robot moves to the place to intake the balls
        robot.operatorStateMachine.moveToState(OperatorStateMachine.State.INTAKE);
        robot.autoMoveTo(18.5,-44,-175,4);
        robot.chill(true,0.2);
        robot.launcher.revFlywheel();
        robot.autoMoveTo(32.3,-44,-175,4);
        robot.chill(true, 0.2);

//
        sortedLaunchFar(true,false);
//
//        // sorted cycle 3
//
////      park off of launch line and close to the gate to clear the classifier at teleop start
        robot.autoMoveTo(30,-22,-170,2);
        robot.chill(true,0.2);
    }
}