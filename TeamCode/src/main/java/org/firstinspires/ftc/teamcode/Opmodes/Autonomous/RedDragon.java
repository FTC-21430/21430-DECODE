package org.firstinspires.ftc.teamcode.Opmodes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Firmware.OperatorStateMachine;
import org.firstinspires.ftc.teamcode.Opmodes.BaseAuto;
@Autonomous
public class RedDragon extends BaseAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true, true);
        robot.odometry.recalibrateIMU();
        while (opModeInInit()) {
            int tempID = robot.aprilTags.getMotifID();
            if (tempID != 0) motifId = tempID;
            telemetry.addData("CurrentMotif", motifId);
            telemetry.update();
        }
        robot.odometry.overridePosition(62,15,-45);

        robot.autoMoveTo(48,-1,50,2);
        autonomousLaunching(motifId);

        robot.operatorStateMachine.moveToState(OperatorStateMachine.State.INTAKE);
        robot.autoMoveTo(26,47,270,2);
        robot.autoMoveTo(32,48,270,2);
        robot.chill(true,0.4);

        robot.autoMoveTo(48,-1,50,2);
        autonomousLaunching(motifId);

        robot.operatorStateMachine.moveToState(OperatorStateMachine.State.INTAKE);
        robot.autoMoveTo(2,47,270,2);
        robot.autoMoveTo(7,47,270,2);
        robot.chill(true,0.4);

        robot.autoMoveTo(48,-1,50,2);
        autonomousLaunching(motifId);

        robot.autoMoveTo(35,12,450,2);
    }
    }
