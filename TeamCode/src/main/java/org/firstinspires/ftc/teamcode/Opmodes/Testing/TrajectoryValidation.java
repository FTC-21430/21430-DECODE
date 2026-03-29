package org.firstinspires.ftc.teamcode.Opmodes.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Opmodes.BaseTeleOp;
@Config
@TeleOp
public class TrajectoryValidation extends BaseTeleOp {

    public static String alliance = "blue";
    @Override
    public void runOpMode() throws InterruptedException {
        initialize(false,false,false);
        waitForStart();
        robot.odometry.resetPositionAndIMU();
        while (opModeIsActive()){
            robot.odometry.updateOdometry();
            double distance = robot.trajectoryKinematics.getDistance(alliance,robot.odometry.getRobotX(),robot.odometry.getRobotY());
            robot.trajectoryKinematics.calculateTrajectory(distance,0);
            telemetry.addData("distance", distance);
            telemetry.addData("launchSpeed",robot.trajectoryKinematics.getLaunchMagnitude());
            telemetry.addData("LaunchAngle", robot.trajectoryKinematics.getInitialAngle());
            telemetry.update();

            // straight and simple drive commands
            robot.driveTrain.setDrivePower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x,0);
            robot.bulkSensorBucket.clearCache();
        }
    }
}
