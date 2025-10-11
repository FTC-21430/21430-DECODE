package org.firstinspires.ftc.teamcode.Opmodes.Demo;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Opmodes.BaseTeleOp;

@TeleOp
public class SpeedcontrolTesting extends BaseTeleOp {

    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true, false);

        telemetry.addLine("Waiting for start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()){
            robot.launcher.flywheel.setTargetSpeed(robot.launcher.flywheel.getTargetSpeed()+(gamepad1.left_stick_y/200));
            robot.launcher.flywheel.updateSpeedControl();

            telemetry.addData("Target Speed: ", robot.launcher.flywheel.getTargetSpeed());
            telemetry.addData("Current Speed: ", robot.launcher.flywheel.getCurrentSpeed());
            telemetry.addData("Acceleration: ", robot.launcher.flywheel.getCurrentAcceleration());
            telemetry.addData("At speed: ", robot.launcher.flywheel.isAtSpeed());

            telemetry.update();
        }



    }
}
