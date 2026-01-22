package org.firstinspires.ftc.teamcode.Opmodes.Testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Opmodes.BaseTeleOp;

@TeleOp
public class ApriltagLocalizationTesting extends BaseTeleOp {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true, true);
        robot.odometry.recalibrateIMU();
        waitForStart();
        robot.odometry.overridePosition(0, 0, 0);
        robot.setAlliance("red");
        while (opModeIsActive()) {
            robot.odometry.updateOdometry();
            telemetry.addData("odometryX", robot.odometry.getRobotX());
            telemetry.addData("odometryY", robot.odometry.getRobotY());
            telemetry.addData("odometryYaw", robot.odometry.getRobotAngle());
            robot.aprilTags.locateAprilTags("red");
            if (robot.aprilTags.isTag("red")) {
                telemetry.addData("aprilX", robot.aprilTags.getRobotX());
                telemetry.addData("aprilY", robot.aprilTags.getRobotY());
                telemetry.addData("aprilYaw", robot.aprilTags.getRobotAngle());
            }
            robot.aimBasedOnTags();
            robot.bulkSensorBucket.clearCache();
            robot.driveTrain.setDrivePower(0, 0, robot.rotationControl.getOutputPower(robot.odometry.getRobotAngle()), robot.odometry.getRobotAngle());
            robot.aprilTags.clearCache();
            telemetry.update();


        }
    }
}
