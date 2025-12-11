package org.firstinspires.ftc.teamcode.Opmodes.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Firmware.Systems.AprilTag;
import org.firstinspires.ftc.teamcode.Firmware.Systems.Launcher;

@Config
@TeleOp
public class LaunchTuningTesting extends LinearOpMode {

    Launcher launcher;
    AprilTag aprilTagProcessing;

    public static double angle = 0;
    public static double speed = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        // Init launcher
        launcher = new Launcher(hardwareMap, telemetry);
        // We don't want the flywheel running right now
        launcher.setSpeed(0);

        aprilTagProcessing = new AprilTag();
        aprilTagProcessing.init(hardwareMap,telemetry);
        waitForStart();
        while (opModeIsActive()){

            launcher.setLaunchAngle(angle);
            launcher.setSpeed(speed);
            launcher.updateSpeedControl();
            telemetry.addData("readyForLaunch", launcher.isUpToSpeed());

            telemetry.addLine("-----------------------------");
            telemetry.addData("angle", angle);
            telemetry.addData("speed", speed);
            telemetry.addData("distance", aprilTagProcessing.getDistance("red"));
            telemetry.update();
        }
    }
}
