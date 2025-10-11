package org.firstinspires.ftc.teamcode.Opmodes.Testing;

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
            // Adjust the flywheel target speed using the left stick Y axis
            robot.launcher.setSpeed(robot.launcher.getTargetSpeed() + (gamepad1.left_stick_y * 10));
            // Update the flywheel's speed control
            robot.launcher.updateSpeedControl();

            // Telemetry output for debugging and monitoring
            telemetry.addData("Target Speed: ", robot.launcher.getTargetSpeed());
            telemetry.addData("Current Speed: ", robot.launcher.getSpeed());
            telemetry.addData("At speed: ", robot.launcher.isUpToSpeed());
            telemetry.update();
        }



    }
}