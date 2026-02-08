package org.firstinspires.ftc.teamcode.Opmodes.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Firmware.OperatorStateMachine;
import org.firstinspires.ftc.teamcode.Firmware.Systems.Spindexer;
import org.firstinspires.ftc.teamcode.Firmware.Systems.SpindexerColorSensor;
import org.firstinspires.ftc.teamcode.Opmodes.BaseAuto;
@Autonomous
public class BlueBeaver extends BaseAuto {

    private void sortedLaunch(boolean finalLaunch, boolean firstLaunch){
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
            robot.autoMoveTo(-38, -18, -124, 4);
        }
        robot.chill(true,0.3);
        robot.aimAtGoal();
        autonomousLaunching(motifId);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        initialize(true, true);
        robot.setAlliance("blue");
        robot.odometry.recalibrateIMU();
        robot.spindexer.setColorIndexing(SpindexerColorSensor.COLORS.GREEN, SpindexerColorSensor.COLORS.PURPLE, SpindexerColorSensor.COLORS.PURPLE);

        waitForStart();
        robot.setAlliance("blue");
        //This is the starting location of the robot
        robot.odometry.overridePosition(-49.82,-54.5782,-125.08);
        robot.spindexer.setIndexOffset(Spindexer.INDEX_TYPE.NONE);
        robot.launcher.setSpeed(1400);
        robot.chill(false,0.2);
        robot.setAlliance("blue");
        //This is the position that the robot moves to to shoot the first three balls
        motifId = 0;
        robot.aimAtGoal();
        robot.chill(true,0.2);
        sortedLaunch(false, true);
        detectMotifWhileMoveTo(-39,-40,-210,10);
        robot.operatorStateMachine.moveToState(OperatorStateMachine.State.INTAKE);

//        //The robot moves to the place to intake the balls
        detectMotifWhileMoveTo(-36,-45,-180,14);
        robot.chill(true, 0.3);

        detectMotifWhileMoveTo(-27,-44.5,-180,5);


        detectMotifWhileMoveTo(-9,-44.5,-180,4);

//
//        //The robot bumps the gate
        detectMotifWhileMoveTo(-8,-44,-180,5);

        detectMotifWhileMoveTo(-8,-53.3,-180,3);
        robot.launcher.revFlywheel();
        chillAndDetect(true,0.4);

//
        sortedLaunch(false,false);
//
//        // sorted cycle 1
//
//        //The robot moves to the place to intake the balls
        robot.operatorStateMachine.moveToState(OperatorStateMachine.State.INTAKE);
        detectMotifWhileMoveTo(-5,-46.7,-176,4);

        detectMotifWhileMoveTo(16,-46.7,-177,4);
        robot.launcher.revFlywheel();


        robot.chill(true, 0.2);
//
        sortedLaunch(false,false);
//
//        // sorted cycle 2
//
//        //The robot moves to the place to intake the balls
        robot.operatorStateMachine.moveToState(OperatorStateMachine.State.INTAKE);
        robot.autoMoveTo(18.5,-43,-177,4);
        robot.chill(true,0.2);
        robot.autoMoveTo(31,-43.7,-177,4);
        robot.launcher.revFlywheel();
        robot.chill(true, 0.2);


//
        sortedLaunch(true,false);
//        robot.chill(true,0.1);
//
//        // sorted cycle 3
//
////      park off of launch line
        robot.autoMoveTo(-50, -18, -90, 4);
        robot.chill(true,0.2);


    }
}
