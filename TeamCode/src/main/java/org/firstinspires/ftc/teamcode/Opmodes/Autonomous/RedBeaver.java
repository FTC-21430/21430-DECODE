package org.firstinspires.ftc.teamcode.Opmodes.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Firmware.OperatorStateMachine;
import org.firstinspires.ftc.teamcode.Firmware.Systems.Spindexer;
import org.firstinspires.ftc.teamcode.Firmware.Systems.SpindexerColorSensor;
import org.firstinspires.ftc.teamcode.Opmodes.BaseAuto;
@Autonomous
public class RedBeaver extends BaseAuto {

    private void sortedLaunch(boolean finalLaunch){
//        robot.autoMoveTo(0, 40, 135, 2);
        //The robot moves to the launch zone and it launches the three balls
        if (!finalLaunch) {
            robot.autoMoveTo(-15.5, 24, 124, 13);
        }else{
            robot.autoMoveTo(-37, 24, 129, 13);
        }

        robot.aimBasedOnTags();

        robot.chill(true,0.1);
        autonomousLaunching(motifId);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot.setAlliance("red");
        initialize(true, true);
        robot.odometry.recalibrateIMU();
        robot.spindexer.setColorIndexing(SpindexerColorSensor.COLORS.GREEN, SpindexerColorSensor.COLORS.PURPLE, SpindexerColorSensor.COLORS.PURPLE);

        waitForStart();
        //This is the starting location of the robot
        robot.odometry.overridePosition(-49.82,54.5782,125.08);
        robot.spindexer.setIndexOffset(Spindexer.INDEX_TYPE.NONE);
        robot.chill(false,0.5);
        //This is the position that the robot moves to to shoot the first three balls
        motifId = 0;
        sortedLaunch(false);
        detectMotifWhileMoveTo(-39,40,210,10);
        robot.operatorStateMachine.moveToState(OperatorStateMachine.State.INTAKE);

//        //The robot moves to the place to intake the balls
        detectMotifWhileMoveTo(-34,43,180,14);

        detectMotifWhileMoveTo(-27,44,180,3);


        detectMotifWhileMoveTo(-6,42,180,4);

//
//        //The robot bumps the gate
        detectMotifWhileMoveTo(-6,44,180,2.5);

        detectMotifWhileMoveTo(-6,53.3,180,3);
        chillAndDetect(true,0.4);

//
        sortedLaunch(false);
//
//        // sorted cycle 1
//
//        //The robot moves to the place to intake the balls
        robot.operatorStateMachine.moveToState(OperatorStateMachine.State.INTAKE);
        detectMotifWhileMoveTo(-5,44.7,172,4);

        detectMotifWhileMoveTo(16,43.5,173,4);


        robot.chill(true, 0.4);
//
        sortedLaunch(false);
//
//        // sorted cycle 2
//
//        //The robot moves to the place to intake the balls
        robot.operatorStateMachine.moveToState(OperatorStateMachine.State.INTAKE);
        robot.autoMoveTo(18.5,41,175,4);
        robot.chill(true,0.4);
        robot.autoMoveTo(31,42,175,4);
        robot.chill(true, 0.6);


//
        sortedLaunch(true);
//
//        // sorted cycle 3
//
////      park off of launch line and close to the gate to clear the classifier at teleop start
        robot.autoMoveTo(-2,43,180,2);
        robot.chill(true,0.5);


   }
    }
