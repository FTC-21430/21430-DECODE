package org.firstinspires.ftc.teamcode.Opmodes.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Opmodes.BaseTeleOp;
@Config
@TeleOp
@Disabled
public class SpeedcontrolTesting extends BaseTeleOp {

    public static double targetSpeed = 0.0;
    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true, false,false);
        telemetry.addLine("Waiting for start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()){
            // Adjust the flywheel target speed using the left stick Y axis
            robot.launcher.setSpeed(targetSpeed);
            // Update the flywheel's speed control
            robot.launcher.update();

            // Telemetry output for debugging and monitoring
            telemetry.addData("Target Speed: ", robot.launcher.getTargetSpeed());
            telemetry.addData("Current Speed: ", robot.launcher.getSpeed());
            telemetry.addData("At speed: ", robot.launcher.isUpToSpeed());
            telemetry.update();
        }



    }
}