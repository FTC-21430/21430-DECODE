package org.firstinspires.ftc.teamcode.Opmodes.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Opmodes.BaseTeleOp;

@TeleOp
//@Disabled
public class ApriltagLocalizationTesting extends BaseTeleOp {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true, false,false,"blue");
        robot.odometry.recalibrateIMU();
        waitForStart();
        robot.odometry.overridePosition(-64.62,-40.5,180);
        robot.setAlliance("blue");
        while (opModeIsActive()) {
            robot.odometry.updateOdometry();
            robot.aprilTags.updateAprilValues(robot.odometry.getRobotX(), robot.odometry.getRobotY(),robot.odometry.getRobotAngle(),true, "blue");
            robot.telemetry.addData("aprilX", robot.aprilTags.getRobotX());
            robot.telemetry.addData("aprilY", robot.aprilTags.getRobotY());
            robot.telemetry.addData("aprilYaw", robot.aprilTags.getRobotAngle());
            robot.telemetry.addData("odomX", robot.odometry.getRobotX());
            robot.telemetry.addData("odomY",robot.odometry.getRobotY());
            robot.telemetry.addData("odomAngle", robot.odometry.getRobotAngle());
            robot.aimAtGoal();
            robot.bulkSensorBucket.clearCache();
//            robot.driveTrain.setDrivePower(0, 0, robot.rotationControl.getOutputPower(robot.odometry.getRobotAngle()), robot.odometry.getRobotAngle());
            robot.aprilTags.clearCache();
            telemetry.update();
        }
    }
}
