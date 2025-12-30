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
            robot.autoMoveTo(-23.5, 24, 135, 4);
        }else{
            robot.autoMoveTo(-40, 18, 135, 4);
        }
        robot.chill(true,0.1);
        autonomousLaunching(motifId);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true, true);
        robot.odometry.recalibrateIMU();
        robot.spindexer.setColorIndexing(SpindexerColorSensor.COLORS.GREEN, SpindexerColorSensor.COLORS.PURPLE, SpindexerColorSensor.COLORS.PURPLE);

        waitForStart();
        //This is the starting location of the robot
        robot.odometry.overridePosition(-49.82,54.5782,125.08);
        robot.spindexer.setIndexOffset(Spindexer.INDEX_TYPE.NONE);
        

        //This is the position that the robot moves to to shoot the first three balls
        motifId = 0;
        sortedLaunch(false);
        detectMotifWhileMoveTo(-39,25,210,2);
        robot.operatorStateMachine.moveToState(OperatorStateMachine.State.INTAKE);
        chillAndDetect(true,0.2);
//        //The robot moves to the place to intake the balls
        detectMotifWhileMoveTo(-32,46,180,2);
        chillAndDetect(true,0.1);

        detectMotifWhileMoveTo(-27,46,180,3);


        detectMotifWhileMoveTo(-6,46,180,4);

//
//        //The robot bumps the gate
        detectMotifWhileMoveTo(-6,50,180,2);
        chillAndDetect(true,1);

        detectMotifWhileMoveTo(-6,53.3,180,3);
        chillAndDetect(true,1);

//
        sortedLaunch(false);
//
//        // sorted cycle 1
//
//        //The robot moves to the place to intake the balls
        robot.operatorStateMachine.moveToState(OperatorStateMachine.State.INTAKE);
        detectMotifWhileMoveTo(0,44.0,185,3);

        detectMotifWhileMoveTo(15.5,44.5,180,4);


        robot.chill(true, 0.8);
//
        sortedLaunch(false);
//
//        // sorted cycle 2
//
//        //The robot moves to the place to intake the balls
        robot.operatorStateMachine.moveToState(OperatorStateMachine.State.INTAKE);
        detectMotifWhileMoveTo(18.5,41,180,5);


        detectMotifWhileMoveTo(31,41,180,4);
        robot.chill(true, 0.8);


//
        sortedLaunch(true);
//
//        // sorted cycle 3
//
////      park off of launch line and close to the gate to clear the classifier at teleop start
        robot.autoMoveTo(-2,46,180,2);
        robot.chill(true,0.2);


   }
    }
