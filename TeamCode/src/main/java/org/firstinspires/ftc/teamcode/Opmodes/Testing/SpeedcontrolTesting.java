package org.firstinspires.ftc.teamcode.Opmodes.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Opmodes.BaseTeleOp;
@Config
@TeleOp
public class SpeedcontrolTesting extends BaseTeleOp {

    public static double targetSpeed = 100;
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
            robot.bulkSensorBucket.clearCache();
        }

//        DcMotor flywheel = null;
//        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
//        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        waitForStart();
//        while (opModeIsActive()){
//            telemetry.addData("pos", flywheel.getCurrentPosition());
//            telemetry.update();
//        }


    }
}