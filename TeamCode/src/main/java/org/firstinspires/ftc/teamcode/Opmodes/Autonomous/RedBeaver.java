package org.firstinspires.ftc.teamcode.Opmodes.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Firmware.OperatorStateMachine;
import org.firstinspires.ftc.teamcode.Firmware.Systems.SpindexerColorSensor;
import org.firstinspires.ftc.teamcode.Opmodes.BaseAuto;
@Autonomous
public class RedBeaver extends BaseAuto {

    private void sortedLaunch(){
//        robot.autoMoveTo(0, 40, 135, 2);
        //The robot moves to the launch zone and it launches the three balls
        robot.autoMoveTo(-23.5, 24, 135, 4);
        robot.chill(true,1);
        autonomousLaunching(motifId);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true, true);
        robot.odometry.recalibrateIMU();
        robot.spindexer.setColorIndexing(SpindexerColorSensor.COLORS.GREEN, SpindexerColorSensor.COLORS.PURPLE, SpindexerColorSensor.COLORS.PURPLE);

        waitForStart();
        //TODO; tune starting positon for actual robot
        //This is the starting location of the robot
        robot.odometry.overridePosition(-49.82,54.5782,125.08);

        

        //This is the position that the robot moves to to shoot the first three balls
        motifId = 0;
        sortedLaunch();
        motifId = 21;
        detectMotifWhileMoveTo(-39,25,210,2);
        robot.chill(true,0.2);
        detectMotifWhileMoveTo(-27,46,195,3);
//        //The robot moves to the place to intake the balls
        robot.operatorStateMachine.moveToState(OperatorStateMachine.State.INTAKE);


        detectMotifWhileMoveTo(-5,46,180,4);

//
//        //The robot bumps the gate
        detectMotifWhileMoveTo(-4.5,50,180,2);
        robot.chill(true,1);

        detectMotifWhileMoveTo(-4.5,53.3,180,3);
        robot.chill(true,1);

//
        sortedLaunch();
//
//        // sorted cycle 1
//
//        //The robot moves to the place to intake the balls
        robot.operatorStateMachine.moveToState(OperatorStateMachine.State.INTAKE);
        robot.autoMoveTo(0,43.5,185,3);

        robot.autoMoveTo(15.5,43.5,180,4);


        robot.chill(true, 1.2);
//
        sortedLaunch();
//
//        // sorted cycle 2
//
//        //The robot moves to the place to intake the balls
        robot.operatorStateMachine.moveToState(OperatorStateMachine.State.INTAKE);
        robot.autoMoveTo(18.5,41,180,5);


        robot.autoMoveTo(31,41,180,4);
        robot.chill(true, 0.8);


//
        sortedLaunch();
//
//        // sorted cycle 3
//
////      park off of launch line and close to the gate to clear the classifier at teleop start
        robot.autoMoveTo(-2,46,180,2);
        robot.chill(true,0.2);


   }
    }
