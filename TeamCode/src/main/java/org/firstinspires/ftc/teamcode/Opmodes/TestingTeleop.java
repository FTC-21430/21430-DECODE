package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TestingTeleop extends BaseTeleOp{

    // The main code that runs during init
    @Override
    public void runOpMode() throws InterruptedException {

        // initializes the robot without resetting the odometry
        initialize(false, false);

        waitForStart();

        while(opModeIsActive()) {

            if (gamepad2.square){
                robot.spindexer.moveToNextIndex();
            }
            if (gamepad2.share) {
                robot.spindexer.eject();
            }
            // get and update functions
            robot.updateLoopTime();
            robot.odometry.updateOdometry();

            // resets Field Centric Driving
            if (gamepad1.share) {
                robot.odometry.resetIMU();
            }

            robot.driveTrain.setDrivePower(-gamepad1.left_stick_y, gamepad1.left_stick_x, robot.anglePID.getPower(), robot.odometry.getRobotAngle());
            robot.updateRobot(false, false, false);
        }
    }
}