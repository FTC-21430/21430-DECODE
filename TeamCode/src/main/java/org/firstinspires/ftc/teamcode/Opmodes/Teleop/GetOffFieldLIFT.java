package org.firstinspires.ftc.teamcode.Opmodes.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Firmware.Systems.Lifter;
import org.firstinspires.ftc.teamcode.Opmodes.BaseTeleOp;

@TeleOp
public class GetOffFieldLIFT extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Lifter lifter = new Lifter(hardwareMap,telemetry);
        waitForStart();
        lifter.setLiftPosition(1.1);
        while (opModeIsActive()){
            if (gamepad1.dpadDownWasPressed()){
                lifter.home();
            }
            if (gamepad1.left_trigger>0.2){
                lifter.testDrop();
            }
            lifter.update();
            telemetry.update();

        }
    }
}
