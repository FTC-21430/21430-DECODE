package org.firstinspires.ftc.teamcode.Opmodes.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Opmodes.BaseTeleOp;

@TeleOp
public class BlueScrimmageTeleop extends BaseTeleOp {

    // The main code that runs during init
    @Override
    public void runOpMode() throws InterruptedException {

        // initializes the robot without resetting the odometry
        initialize(true, false);
        robot.setAlliance("blue");
        robot.driveTrain.fieldCentricDriving(true);

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
            if (gamepad2.dpadDownWasPressed()){
                robot.launchFrom("close");
            } else if(gamepad2.dpadLeftWasPressed()){
                robot.launchFrom("mid");
            } else if(gamepad2.dpadUpWasPressed()){
                robot.launchFrom("far");
            } else if (gamepad2.dpadRightWasPressed()){
                robot.launcher.retractRamp();
                robot.launcher.setSpeed(600);
            }
            if (gamepad1.right_trigger > 0.4){
                robot.driveTrain.setSpeedMultiplier(0.5);
            } else if (robot.driveTrain.getSpeedMultiplier() != 1){
                robot.driveTrain.setSpeedMultiplier(1);
            }
            if (gamepad2.left_bumper){
                robot.intake.setIntakePower(-1);
            } else {
                robot.intake.setIntakePower(0);
            }
            if (gamepad1.cross){
                robot.aimAtGoal();
            }else{
                robot.rotationControl.changeTargetByJoystick(gamepad1.right_stick_x,robot.odometry.getRobotAngle());
            }

            //sets drive power and what gamepad does
            robot.driveTrain.setDrivePower(-gamepad1.left_stick_y, gamepad1.left_stick_x, robot.rotationControl.getOutputPower(robot.odometry.getRobotAngle()), robot.odometry.getRobotAngle());
            robot.updateRobot(false, false, false);
            robot.bulkSensorBucket.clearCache();
        }
    }
}