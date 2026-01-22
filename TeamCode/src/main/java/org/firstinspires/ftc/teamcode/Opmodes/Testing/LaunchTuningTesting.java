package org.firstinspires.ftc.teamcode.Opmodes.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Firmware.Systems.AprilTag;
import org.firstinspires.ftc.teamcode.Firmware.Systems.GobildaPinpointModuleFirmware;
import org.firstinspires.ftc.teamcode.Firmware.Systems.Launcher;
import org.firstinspires.ftc.teamcode.Firmware.Systems.Spindexer;

@Config
@TeleOp
public class LaunchTuningTesting extends LinearOpMode {

    Launcher launcher;
    Spindexer spindexer;
    AprilTag aprilTagProcessing;

    public static double angle = 0;
    public static double speed = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        // Init launcher
        launcher = new Launcher(hardwareMap, telemetry);
        spindexer = new Spindexer(hardwareMap,telemetry,true,false);
        GobildaPinpointModuleFirmware odometry = new GobildaPinpointModuleFirmware(hardwareMap,4.9574,2.78,true);

        // We don't want the flywheel running right now
        launcher.setSpeed(0);

        aprilTagProcessing = new AprilTag();
        aprilTagProcessing.init(hardwareMap,telemetry,10);
        waitForStart();
        while (opModeIsActive()){

            launcher.setLaunchAngle(angle);
            launcher.setSpeed(speed);
            launcher.update();
            telemetry.addData("readyForLaunch", launcher.isUpToSpeed());

            telemetry.addLine("-----------------------------");
            telemetry.addData("angle", angle);
            telemetry.addData("speed", speed);
            telemetry.addData("distance", aprilTagProcessing.getDistance("red",odometry.getRobotX(),odometry.getRobotY()));
            telemetry.addLine("------------------------------");
            if (aprilTagProcessing.isTag("red")) {
                telemetry.addData("robotX", aprilTagProcessing.getRobotX());
                telemetry.addData("robotY", aprilTagProcessing.getRobotY());
                telemetry.addData("robotAngle", aprilTagProcessing.getRobotAngle());
            }
            telemetry.update();

            if (gamepad1.circleWasPressed()){
                spindexer.moveToNextIndex();
            }
            if (gamepad1.crossWasPressed()){
                spindexer.eject();
            }

            spindexer.updateSpindexer();
        }
    }
}
