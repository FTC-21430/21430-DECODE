package org.firstinspires.ftc.teamcode.Opmodes.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Firmware.OperatorStateMachine;
import org.firstinspires.ftc.teamcode.Firmware.Systems.SpindexerColorSensor;
import org.firstinspires.ftc.teamcode.Opmodes.BaseTeleOp;

@TeleOp
public class BlueTeleop extends BaseTeleOp {

    private boolean manualMode = false;

    // The main code that runs during init
    @Override
    public void runOpMode() throws InterruptedException {

        initialize(true,false, false);
        robot.setAlliance("blue");
        robot.driveTrain.fieldCentricDriving(true);
        robot.aprilTags.setExposure(10);
        waitForStart();
        robot.rotationControl.setTargetAngle(0);
        while(opModeIsActive()) {

            // get and update functions
            robot.updateLoopTime();
            robot.odometry.updateOdometry();
            telemetry.addData("X",robot.odometry.getRobotX());
            telemetry.addData("Y", robot.odometry.getRobotY());
            telemetry.addLine("-----------------------------");
            robot.updateTrajectories();

            // resets Field Centric Driving
            if (gamepad1.shareWasPressed()) {
                robot.odometry.resetIMU();
                robot.rotationControl.setTargetAngle(0);
            }
            if (gamepad2.shareWasPressed()){
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
                if (gamepad1.crossWasPressed()){
                    robot.operatorStateMachine.moveToState(OperatorStateMachine.State.LAUNCH);
                }
                if (gamepad1.circleWasPressed()){
                    robot.operatorStateMachine.moveToState(OperatorStateMachine.State.IDLE);
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
                if (gamepad2.triangle){
                    robot.launcher.revFlywheel();
                } else if(robot.operatorStateMachine.getCurrentState() != OperatorStateMachine.State.LAUNCH){
                    robot.launcher.idleFlywheel();
                }

                robot.operatorStateMachine.updateStateMachine();
            }
            if (gamepad2.leftBumperWasPressed() && gamepad2.square){
                robot.lifter.lift();
            }
            if (gamepad2.rightBumperWasPressed() && gamepad2.square){
                robot.lifter.lockLatches();
            }
            if (gamepad2.dpadDownWasPressed() && gamepad2.square){
                robot.lifter.home();
            }
            if (gamepad1.left_bumper){
                robot.updateOdometryOnTags(true);
            }else{
                robot.updateOdometryOnTags(false);
            }
            if (gamepad1.left_trigger > 0.2){
                robot.aimAtGoal();
                robot.driveTrain.setTurnPriority((gamepad1.left_trigger/2)+0.8);
            }else{
                robot.rotationControl.changeTargetByJoystick(gamepad1.right_stick_x,robot.odometry.getRobotAngle());
                robot.driveTrain.setTurnPriority(1.0);
            }
            if (gamepad1.dpad_down){
                robot.park();

            }else {
                //sets drive power and what gamepad does
                robot.driveTrain.setDrivePower(-gamepad1.left_stick_y, gamepad1.left_stick_x, robot.rotationControl.getOutputPower(robot.odometry.getRobotAngle()), robot.odometry.getRobotAngle());
            }

            robot.updateRobot(false, false, false);
            telemetry.addData("current robot heading", robot.odometry.getRobotAngle());

            robot.bulkSensorBucket.clearCache();
            telemetry.update();
        }
    }
}
