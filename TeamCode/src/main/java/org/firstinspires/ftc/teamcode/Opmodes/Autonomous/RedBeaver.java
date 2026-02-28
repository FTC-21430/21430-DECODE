package org.firstinspires.ftc.teamcode.Opmodes.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Firmware.OperatorStateMachine;
import org.firstinspires.ftc.teamcode.Firmware.Systems.Spindexer;
import org.firstinspires.ftc.teamcode.Firmware.Systems.SpindexerColorSensor;
import org.firstinspires.ftc.teamcode.Opmodes.BaseAuto;
@Autonomous
public class RedBeaver extends BaseAuto {

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

        robot.aimAtGoal();
        robot.setLauncherBasedOnTags();

        robot.chill(true,0.7);
        robot.aimAtGoal();
        autonomousLaunching(motifId);
//
    }

    @Override
    public void runOpMode() throws InterruptedException {

        initialize(true, true,false);
        robot.setAlliance("red");
        robot.odometry.recalibrateIMU();
        robot.spindexer.setColorIndexing(SpindexerColorSensor.COLORS.GREEN, SpindexerColorSensor.COLORS.PURPLE, SpindexerColorSensor.COLORS.PURPLE);

        waitForStart();
        runtime.reset();
        //This is the starting location of the robot
        robot.odometry.overridePosition(-49.82,54.5782,125.08);
        robot.spindexer.setIndexOffset(Spindexer.INDEX_TYPE.NONE);
        robot.launcher.revFlywheel();
        robot.chill(false,0.2);
        //This is the position that the robot moves to to shoot the first three balls
        motifId = 0;
        robot.aimAtGoal();
        robot.launcher.revFlywheel();
        robot.chill(true,0.2);
        sortedLaunch(false, true);

        robot.operatorStateMachine.moveToState(OperatorStateMachine.State.INTAKE);

//        //The robot moves to the place to intake the balls
        robot.autoMoveTo(-34,43.5,180,14);
        robot.chill(true,0.3);
        robot.pathFollowing.setFollowSpeed(0.5);
        robot.chill(true,0.3);

        robot.autoMoveTo(-27,44,180,3);


        robot.autoMoveTo(-8,44,180,4);

        robot.pathFollowing.setFollowSpeed(0.4);
//
//        //The robot bumps the gate
        robot.autoMoveTo(-7,44,180,2.5);
        robot.pathFollowing.setFollowSpeed(1);

        robot.autoMoveTo(-7,53.3,180,3);
        robot.chill(true,0.6);
        robot.autoMoveTo(-10,23,180,10);
        robot.launcher.revFlywheel();
        chillAndDetect(true,0.5);

//
        sortedLaunch(false, false);
//
//        // sorted cycle 1
//
//        //The robot moves to the place to intake the balls
        robot.operatorStateMachine.moveToState(OperatorStateMachine.State.INTAKE);
        robot.autoMoveTo(-9,43,176,6);
        robot.pathFollowing.setFollowSpeed(0.6);

        robot.launcher.revFlywheel();
        robot.autoMoveTo(15,43.5,177,3);
        robot.pathFollowing.setFollowSpeed(1);



        robot.chill(true, 0.28);
//
        sortedLaunch(false, false);
//
//        // sorted cycle 2
//
//        //The robot moves to the place to intake the balls
        robot.operatorStateMachine.moveToState(OperatorStateMachine.State.INTAKE);
        robot.autoMoveTo(18.5,40,175,4);
        robot.pathFollowing.setFollowSpeed(0.7);
        robot.chill(true,0.2);
        robot.launcher.revFlywheel();
        robot.autoMoveTo(34.3,38.5,175,4);
        robot.chill(true, 0.2);

        robot.pathFollowing.setFollowSpeed(1);
//
        sortedLaunch(true,false);
//
//        // sorted cycle 3
//
////      park off of launch line and close to the gate to clear the classifier at teleop start
        robot.autoMoveTo(-50, 18, 90, 4);
        robot.chill(true,0.2);


    }
}
