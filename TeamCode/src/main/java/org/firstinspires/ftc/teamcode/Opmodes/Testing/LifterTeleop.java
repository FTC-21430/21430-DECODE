package org.firstinspires.ftc.teamcode.Opmodes.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Firmware.Systems.Lifter;

public class LifterTeleop extends LinearOpMode {
private Lifter lift = null;
    @Override
    public void runOpMode() throws InterruptedException {
        lift = new Lifter(hardwareMap, telemetry);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.squareWasPressed()) {
                //lift
                lift.lift();
            }
            if (gamepad1.crossWasPressed()) {
                //lower
                lift.retract();
            }
            if (gamepad1.rightBumperWasPressed()) {
                //defence
                lift.defence();
            }
            if (gamepad1.triangleWasPressed()) {
                //open latches
                lift.unlockLatches();
            }
            if (gamepad1.touchpadWasPressed()){
                lift.home();
            }
            if (gamepad1.circleWasPressed()) {
                //closes latches
                lift.lockLatches();
            }
            if (gamepad1.dpadDownWasPressed()){
                lift.home();
            }
            lift.update();
        }
    }
}