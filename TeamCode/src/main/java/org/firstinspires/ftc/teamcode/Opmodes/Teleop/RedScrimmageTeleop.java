package org.firstinspires.ftc.teamcode.Opmodes.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Opmodes.BaseTeleOp;

@TeleOp
public class RedScrimmageTeleop extends BaseTeleOp {

    // The main code that runs during init
    @Override
    public void runOpMode() throws InterruptedException {

        // initializes the robot without resetting the odometry
        initialize(true, false);
        robot.setAlliance("red");

        waitForStart();

        while(opModeIsActive()) {

            // get and update functions
            robot.updateLoopTime();
            robot.odometry.updateOdometry();

            // resets Field Centric Driving
            if (gamepad1.share) {
                robot.odometry.resetIMU();
            }
            if (gamepad2.square){
                robot.spindexer.moveToNextIndex();
            }
            if (gamepad2.share) {
                robot.spindexer.eject();
            }
            if (gamepad2.left_trigger > 0.4){
                robot.aimBasedOnTags();
            } else {
                robot.launcher.retractRamp();
                robot.launcher.setSpeed(1000);
            }

            robot.rotationControl.changeTargetByJoystick(gamepad1.right_stick_x);
            //sets drive power and what gamepad does
            robot.driveTrain.setDrivePower(-gamepad1.left_stick_y, gamepad1.left_stick_x, robot.rotationControl.getOutputPower(robot.odometry.getRobotAngle()), robot.odometry.getRobotAngle());
            robot.updateRobot(false, false, false);

            robot.bulkSensorBucket.clearCache();
        }
    }
}