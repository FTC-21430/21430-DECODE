package org.firstinspires.ftc.teamcode.Opmodes.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Opmodes.BaseAuto;

@Autonomous
public class BlueBeaver extends BaseAuto {

    private void sortedLaunch(){
        //The robot moves to the launch zone and it launches the three balls
        robot.chill(true,3);
        robot.autoMoveTo(-8, -10, 225, 2);
        robot.chill(true,5);
//        autonomousLaunching(motifId);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true, true);
        robot.odometry.recalibrateIMU();

        waitForStart();
        //TODO; tune starting positon for actual robot
        //This is the starting location of the robot
        robot.odometry.overridePosition(-50,-50.8,230.7);

        

        //This is the position that the robot moves to to shoot the first three balls
        robot.autoMoveTo(-15,-20,225,2);

        //TODO: Figure out the shooting code with actual robot
//        robot.launchFrom("mid");


//        //The robot moves to the place to intake the balls
//        robot.operatorStateMachine.moveToState(OperatorStateMachine.State.INTAKE);
        detectMotifWhileMoveTo(-23,-44,160,2);

        detectMotifWhileMoveTo(-5,-46,180,2);

//
//        //The robot bumps the gate
        detectMotifWhileMoveTo(-2,-51,180,2);

        detectMotifWhileMoveTo(-2,-55,180,2);
//
        sortedLaunch();
//
//        // sorted cycle 1
//
//        //The robot moves to the place to intake the balls
//        robot.operatorStateMachine.moveToState(OperatorStateMachine.State.INTAKE);
        robot.autoMoveTo(0,-44,180,2);

        robot.autoMoveTo(18.5,-46,180,2);


        robot.chill(true, 0.5);
//
        sortedLaunch();
//
//        // sorted cycle 2
//
//        //The robot moves to the place to intake the balls
//        robot.operatorStateMachine.moveToState(OperatorStateMachine.State.INTAKE);
        robot.autoMoveTo(23.5,-44,180,2);


        robot.autoMoveTo(42,-46,180,2);
        robot.chill(true, 0.5);


//
        sortedLaunch();
//
//        // sorted cycle 3
//
////      park off of launch line and close to the gate to clear the classifier at teleop start
        robot.autoMoveTo(-2,-51,180,2);
        robot.chill(true,2);


   }
    }
