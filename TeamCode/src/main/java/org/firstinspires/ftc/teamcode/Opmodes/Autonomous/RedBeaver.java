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
        if (!firstLaunch) {
            robot.chill(true, 0.8);
        }else{
            robot.chill(true,0.3);
        }
        robot.aimBasedOnTags();
        robot.setLauncherBasedOnTags();

        robot.chill(true,0.3);
        robot.aimBasedOnTags();
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
        robot.aimBasedOnTags();
        robot.chill(true,0.2);
        sortedLaunch(false, true);
        detectMotifWhileMoveTo(-39,40,210,10);
        robot.operatorStateMachine.moveToState(OperatorStateMachine.State.INTAKE);

//        //The robot moves to the place to intake the balls
        detectMotifWhileMoveTo(-34,43,180,14);
        robot.chill(true,0.3);

        detectMotifWhileMoveTo(-27,44,180,3);


        detectMotifWhileMoveTo(-8.4,42,180,4);

//
//        //The robot bumps the gate
        detectMotifWhileMoveTo(-8.4,44,180,2.5);

        detectMotifWhileMoveTo(-8.4,53.3,180,3);
        chillAndDetect(true,0.4);

//
        sortedLaunch(false, false);
//
//        // sorted cycle 1
//
//        //The robot moves to the place to intake the balls
        robot.operatorStateMachine.moveToState(OperatorStateMachine.State.INTAKE);
        detectMotifWhileMoveTo(-8,46,176,3);

        detectMotifWhileMoveTo(15,46,177,4);


        robot.chill(true, 0.2);
//
        sortedLaunch(false, false);
//
//        // sorted cycle 2
//
//        //The robot moves to the place to intake the balls
        robot.operatorStateMachine.moveToState(OperatorStateMachine.State.INTAKE);
        robot.autoMoveTo(18.5,44,175,4);
        robot.chill(true,0.2);
        robot.autoMoveTo(29.3,44,175,4);
        robot.chill(true, 0.2);


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
