package org.firstinspires.ftc.teamcode.Opmodes.Autonomous;

import org.firstinspires.ftc.teamcode.Firmware.OperatorStateMachine;
import org.firstinspires.ftc.teamcode.Opmodes.BaseAuto;

// This Autonomous is for when the robot starts in the far zone
public class RedChihuahua extends BaseAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true,true);
        robot.odometry.recalibrateIMU();
        while (opModeInInit()){
            int tempID = robot.aprilTags.getMotifID();
            if (tempID != 0) motifId = tempID;
            telemetry.addData("CurrentMotif", motifId);
            telemetry.update();
        }

        // Runtime start

        //set positions
        robot.odometry.overridePosition(64,13,90);

        // launch preloads
        robot.autoMoveTo(49,10,154,1);
        autonomousLaunching(motifId);

        // grab the balls set in the loading zone
        robot.operatorStateMachine.moveToState(OperatorStateMachine.State.INTAKE);
        robot.autoMoveTo(63,54,90,2);
        robot.autoMoveTo(63,61,90,2);
        robot.chill(true,0.4);

        // launch second cycle

        robot.autoMoveTo(49,10,154,1);
        autonomousLaunching(motifId);

        // grab third ( farthest from goal ) set of on field balls
        robot.operatorStateMachine.moveToState(OperatorStateMachine.State.INTAKE);
        robot.autoMoveTo(51,48,0,2);
        robot.autoMoveTo(36,48,0,2);
        robot.chill(true,0.6);

        // launch third cycle

        robot.autoMoveTo(49,10,154,1);
        autonomousLaunching(motifId);

        // move off of launch line ( not super close ot the gate just in case an alliance partner goes there, but should be in a good place to dive for it)

        robot.autoMoveTo(28,9,90,2);
        robot.chill(true,3);
    }
}
