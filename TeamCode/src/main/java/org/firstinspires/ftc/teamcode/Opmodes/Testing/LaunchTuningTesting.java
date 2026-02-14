package org.firstinspires.ftc.teamcode.Opmodes.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Firmware.Systems.AprilTag;
import org.firstinspires.ftc.teamcode.Firmware.Systems.GobildaPinpointModuleFirmware;
import org.firstinspires.ftc.teamcode.Firmware.Systems.Launcher;
import org.firstinspires.ftc.teamcode.Firmware.Systems.Spindexer;
import org.firstinspires.ftc.teamcode.Opmodes.BaseTeleOp;

@Config
@Disabled
@TeleOp
public class LaunchTuningTesting extends BaseTeleOp {
    public static double angle = 0;
    public static double speed = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true, false,false);
        // Init launcher
        // We don't want the flywheel running right now
        robot.launcher.setSpeed(0);
        robot.rotationControl.setTargetAngle(0);
        robot.launcher.setGatePosition(true);
        waitForStart();
        while (opModeIsActive()){
            robot.intake.setIntakePower(-0.6);
            robot.spindexer.setIndexOffset(Spindexer.INDEX_TYPE.LAUNCH);
            robot.odometry.updateOdometry();
            robot.updateOdometryOnTags(false);
            robot.aimAtGoal();
            robot.launcher.setLaunchAngle(angle);
            robot.launcher.setSpeed(speed);
            robot.launcher.update();
            telemetry.addData("readyForLaunch", robot.launcher.isUpToSpeed());
            telemetry.addLine("-----------------------------");
            telemetry.addData("angle", angle);
            telemetry.addData("speed", speed);
            telemetry.addData("distance", robot.aprilTags.getDistance("red",robot.odometry.getRobotX(),robot.odometry.getRobotY()));
            telemetry.addLine("------------------------------");
            if (robot.aprilTags.isTag("red")) {
                telemetry.addData("robotX", robot.aprilTags.getRobotX());
                telemetry.addData("robotY", robot.aprilTags.getRobotY());
                telemetry.addData("robotAngle", robot.aprilTags.getRobotAngle());
            }
            telemetry.update();
            if (gamepad1.circleWasPressed()){
                robot.spindexer.moveToNextIndex();
            }
            if (gamepad1.crossWasPressed()){
                robot.spindexer.eject();
            }
            robot.driveTrain.setDrivePower(0, 0, robot.rotationControl.getOutputPower(robot.odometry.getRobotAngle()), robot.odometry.getRobotAngle());
            robot.bulkSensorBucket.clearCache();
            robot.spindexer.updateSpindexer();
            robot.aprilTags.clearCache();
        }
    }
}
