package org.firstinspires.ftc.teamcode.Opmodes.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Firmware.OperatorStateMachine;
import org.firstinspires.ftc.teamcode.Firmware.Systems.SpindexerColorSensor;
import org.firstinspires.ftc.teamcode.Opmodes.BaseTeleOp;

@TeleOp
public class RedTeleop extends BaseTeleOp {

    private boolean manualMode = false;

    // The main code that runs during init
    @Override
    public void runOpMode() throws InterruptedException {

        // initializes the robot without resetting the odometry
        initialize(true, false);
        robot.setAlliance("red");
        robot.driveTrain.fieldCentricDriving(true);



        waitForStart();
        robot.odometry.resetIMU();
        robot.rotationControl.setTargetAngle(0);
        while(opModeIsActive()) {

            // get and update functions
            robot.updateLoopTime();
            robot.odometry.updateOdometry();

            // resets Field Centric Driving
            if (gamepad1.shareWasPressed()) {
                robot.odometry.resetIMU();
                robot.rotationControl.setTargetAngle(0);
            }
            if (gamepad2.squareWasPressed()){
                if (manualMode){
                    manualMode = false;
                }else{
                    manualMode = true;
                }
            }



            if (manualMode){
                if (gamepad2.squareWasPressed()){
                    robot.spindexer.moveToNextIndex();
                }
                if (gamepad2.rightBumperWasPressed()) {
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
                    robot.launcher.setSpeed(-400);
                    robot.spindexer.setSpindexerPos(60);
                }
                if (gamepad1.right_bumper){
                    robot.driveTrain.setSpeedMultiplier(0.5);
                } else if (robot.driveTrain.getSpeedMultiplier() != 1){
                    robot.driveTrain.setSpeedMultiplier(1);
                }

                if (gamepad2.left_bumper){
                    robot.intake.setIntakePower(-1);
                } else {
                    robot.intake.setIntakePower(0);
                }
                if (gamepad2.left_trigger > 0.4){
                    robot.intake.setIntakePower(0.4);
                }
            }else{
                if (gamepad1.right_bumper){
                    robot.driveTrain.setSpeedMultiplier(0.5);
                } else if (robot.driveTrain.getSpeedMultiplier() != 1){
                    robot.driveTrain.setSpeedMultiplier(1);
                }
                if (gamepad2.triangleWasPressed()){
                    robot.operatorStateMachine.moveToState(OperatorStateMachine.State.LAUNCH);
                }
                if (gamepad2.circleWasPressed()){
                    robot.operatorStateMachine.moveToState(OperatorStateMachine.State.INTAKE);
                }
                if (gamepad2.crossWasPressed()){
                    robot.operatorStateMachine.moveToState(OperatorStateMachine.State.IDLE);
                }
                if (gamepad2.dpadUpWasPressed()){
                    // launch all of the balls without sort
                    robot.operatorStateMachine.addToQueue(SpindexerColorSensor.COLORS.NONE);
                    robot.operatorStateMachine.addToQueue(SpindexerColorSensor.COLORS.NONE);
                    robot.operatorStateMachine.addToQueue(SpindexerColorSensor.COLORS.NONE);
                }
                if (gamepad2.dpadLeftWasPressed()){
                    robot.operatorStateMachine.addToQueue(SpindexerColorSensor.COLORS.PURPLE);
                }
                if (gamepad2.dpadRightWasPressed()){
                    robot.operatorStateMachine.addToQueue(SpindexerColorSensor.COLORS.GREEN);
                }
                if (gamepad2.touchpadWasPressed()){
                    for (int i = 0; i < 3; i++){
                        robot.spindexer.clearColor(i);
                    }
                }

                robot.operatorStateMachine.updateStateMachine();
            }

            if (gamepad1.crossWasPressed()){
                robot.aimBasedOnTags();
            }else{
                robot.rotationControl.changeTargetByJoystick(gamepad1.right_stick_x,robot.odometry.getRobotAngle());
            }

            //sets drive power and what gamepad does
            robot.driveTrain.setDrivePower(-gamepad1.left_stick_y, gamepad1.left_stick_x, robot.rotationControl.getOutputPower(robot.odometry.getRobotAngle()), robot.odometry.getRobotAngle());
            robot.updateRobot(false, false, false);
            telemetry.addData("current robot heading", robot.odometry.getRobotAngle());

            robot.bulkSensorBucket.clearCache();
            telemetry.update();
        }
    }
}
