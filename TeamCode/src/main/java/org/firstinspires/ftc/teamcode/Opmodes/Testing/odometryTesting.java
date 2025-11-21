package org.firstinspires.ftc.teamcode.Opmodes.Testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Opmodes.BaseTeleOp;

@TeleOp
public class odometryTesting extends BaseTeleOp {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true, false);
        telemetry.addLine("Initialized");
        telemetry.update();
        waitForStart();

        robot.odometry.resetPositionAndIMU();
        while(opModeIsActive()){
            robot.odometry.updateOdometry();
            telemetry.addData("X", robot.odometry.getRobotX());
            telemetry.addData("Y", robot.odometry.getRobotY());
            telemetry.addData("Angle", robot.odometry.getRobotAngle());
            telemetry.update();
        }
    }
}
