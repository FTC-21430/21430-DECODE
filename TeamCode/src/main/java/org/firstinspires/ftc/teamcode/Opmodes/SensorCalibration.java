package org.firstinspires.ftc.teamcode.Opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "SensorCalibration", group = "Tools")
public class SensorCalibration extends BaseTeleOp{
    public static float COLOR_SENSOR_GAIN = 10; //10 Is the default?
    public static double SPINDEXER_DEGREES = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true,true,false,"red");
        telemetry.addLine("calibrationBegan - INIT");
        telemetry.update();
        waitForStart();
        while (!gamepad1.cross && opModeIsActive()){
            telemetry.addLine("Limit Switch Calibration");
            telemetry.addData("LiftSwitch1", robot.lifter.liftSwitchOne());
            telemetry.addData("LiftSwitch2", robot.lifter.liftSwitchTwo());
            telemetry.addLine("----------------------------------");
            telemetry.addData("IntakeSwitch1", robot.spindexer.getIntakeSwitch1());
            telemetry.addData("IntakeSwitch2", robot.spindexer.getIntakeSwitch2());
            telemetry.addLine("press GP1 X to continue to next stage");
            telemetry.update();
            robot.bulkSensorBucket.clearCache();
        }
        while (!gamepad1.circleWasPressed() && opModeIsActive()){
            telemetry.addLine("Color Sensor Calibration");
            robot.intake.openGate();
            telemetry.addData("Hue", robot.spindexer.COLOR_SENSOR.getRawData()[0]);
            telemetry.addData("Saturation", robot.spindexer.COLOR_SENSOR.getRawData()[1]);
            telemetry.addData("Value", robot.spindexer.COLOR_SENSOR.getRawData()[2]);
            robot.spindexer.COLOR_SENSOR.setGain(COLOR_SENSOR_GAIN);
            telemetry.addData("Current Detected Color", robot.spindexer.COLOR_SENSOR.getDetectedColor());
            telemetry.addLine("press GP1 Circle to continue to the next stage");
            telemetry.update();
            robot.bulkSensorBucket.clearCache();
        }
        while (!gamepad1.triangleWasPressed() && opModeIsActive()){
            telemetry.addLine("Spindexer Encoder Calibration");
            robot.spindexer.setSpindexerPos(SPINDEXER_DEGREES);

            telemetry.addLine("Use FTC Dashboard,to change spindexer pos");
            telemetry.addData("SpindexerEncoderValue",robot.spindexer.getEncoderPosition());
            telemetry.addData("SpindexerVelocity", robot.spindexer.getVelocity());
            telemetry.addLine("Press GP1 Circle to re-calibrate the spindexer");
            if (gamepad1.circleWasPressed()){
                robot.spindexer.recalibrateSpindexerPosition();
            }
            robot.spindexer.updateSpindexer();
            telemetry.addLine("Press GP1 triangle to continue to the next stage");
            telemetry.update();
            robot.bulkSensorBucket.clearCache();
        }
        while (!gamepad1.squareWasPressed() && opModeIsActive()){
            telemetry.addLine("Localization Calibration");
            telemetry.addLine("Please move robot to red goal starting position (close).");
            telemetry.addLine("Once the robot is at position and stationary, press GP1 square");
            telemetry.update();
            robot.bulkSensorBucket.clearCache();
        }
        robot.odometry.recalibrateIMU();
        robot.odometry.resetPositionAndIMU();


        ElapsedTime runtime = new ElapsedTime();
        while(runtime.seconds() < 10.0){
            telemetry.addLine("LEAVE ROBOT BE FOR 10 SECONDS");
            telemetry.update();
        }
        robot.odometry.overridePosition(-63.22,37.5,180);
        gamepad1.crossWasPressed(); // consume any stale cross press from earlier stages
        gamepad1.circleWasPressed();
        gamepad1.squareWasPressed();
        gamepad1.triangleWasPressed();
        while (!gamepad1.crossWasPressed() && opModeIsActive()){

            telemetry.addLine("Once 10 seconds has passed, please move toward center of field pointing camera at goal");
            telemetry.addLine("In addition to checking AprilTag values, varify odometry position is generally correct across the entire field by moving around and checking coordinates");
            telemetry.addLine("####################################");
            robot.odometry.updateOdometry();
            robot.aprilTags.updateAprilValues(robot.odometry.getRobotX(), robot.odometry.getRobotY(),robot.odometry.getRobotAngle(),true, "red");
            robot.telemetry.addData("aprilX", robot.aprilTags.getRobotX());
            robot.telemetry.addData("aprilY", robot.aprilTags.getRobotY());
            robot.telemetry.addData("aprilYaw", robot.aprilTags.getRobotAngle());
            telemetry.addLine("%%%%%%%%%%%%%%%%%%%%%%%%%");
            robot.telemetry.addData("odomX", robot.odometry.getRobotX());
            robot.telemetry.addData("odomY",robot.odometry.getRobotY());
            robot.telemetry.addData("odomAngle", robot.odometry.getRobotAngle());
            robot.telemetry.addLine("If april tag values are not working perfectly or seem to be hit or miss, go to the AprilTag testing mode for exposure");
            robot.telemetry.addLine("Camera exposure settings found in DECODE Bot");
            robot.telemetry.addLine("Once general difference measurements are complete, press GP1 Cross to continue to next stage");
            robot.bulkSensorBucket.clearCache();
            robot.aprilTags.clearCache();
            telemetry.update();
        }

        while (!gamepad1.circleWasPressed() && opModeIsActive()){
            telemetry.addLine("Lift Encoder Calibration");
            telemetry.addLine("wiggle lifts up and down and ensure that the direction remains correct");
            telemetry.addData("heightL", robot.lifter.getLiftPositionLeft());
            telemetry.addData("heightR", robot.lifter.getLiftPositionRight());
            robot.lifter.update();
            robot.bulkSensorBucket.clearCache();
            telemetry.addLine("press GP1 Circle to continue to next stage");
            telemetry.update();
        }
        while (!gamepad1.triangleWasPressed() && opModeIsActive()){
            telemetry.addLine("Flywheel Encoder Calibration");
            telemetry.addLine("Spin the flywheel with your hand to check if the velocity measurement is correct");
            telemetry.addLine("Don't Worry, it is not powered.");
            robot.launcher.SoftUpdate();
            telemetry.addData("flywheel Velocity", robot.launcher.getSpeed());
            robot.bulkSensorBucket.clearCache();
            telemetry.update();
        }
    }
}
