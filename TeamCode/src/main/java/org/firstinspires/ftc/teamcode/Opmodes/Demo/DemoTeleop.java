package org.firstinspires.ftc.teamcode.Opmodes.Demo;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Firmware.OperatorStateMachine;
import org.firstinspires.ftc.teamcode.Firmware.Systems.Launcher;
import org.firstinspires.ftc.teamcode.Firmware.Systems.SpindexerColorSensor;
import org.firstinspires.ftc.teamcode.Opmodes.BaseTeleOp;

@TeleOp
//@Disabled
public class DemoTeleop extends BaseTeleOp {

    // The main code that runs during init
    @Override
    public void runOpMode() throws InterruptedException {

        initialize(true, false,false);
        robot.setAlliance("blue");
        robot.driveTrain.fieldCentricDriving(false);

        waitForStart();
        robot.rotationControl.setTargetAngle(0);
        while(opModeIsActive()) {
            // get and update functions
            robot.updateLoopTime();
            robot.odometry.updateOdometry();
            robot.updateTrajectories();

            //Speed control for driver
            if (gamepad1.right_bumper){
                robot.driveTrain.setSpeedMultiplier(0.5);
            } else if (robot.driveTrain.getSpeedMultiplier() != 1){
                robot.driveTrain.setSpeedMultiplier(1);
            }
            //State machine for handling artifacts control on driver pad
            if (gamepad1.crossWasPressed()){
                robot.operatorStateMachine.moveToState(OperatorStateMachine.State.LAUNCH);
            }
            //Robot goes to Idle
            if (gamepad1.circleWasPressed()){
                robot.operatorStateMachine.moveToState(OperatorStateMachine.State.IDLE);
            }
            if (gamepad2.circleWasPressed()){
                robot.operatorStateMachine.moveToState(OperatorStateMachine.State.INTAKE);
            }
            //Idle state for operator
            if (gamepad2.crossWasPressed()){
                robot.operatorStateMachine.moveToState(OperatorStateMachine.State.IDLE);
            }
            if (gamepad2.dpadUpWasPressed()){
                //Launch all the balls in a random order
                robot.operatorStateMachine.addToQueue(SpindexerColorSensor.COLORS.NONE);
                robot.operatorStateMachine.addToQueue(SpindexerColorSensor.COLORS.NONE);
                robot.operatorStateMachine.addToQueue(SpindexerColorSensor.COLORS.NONE);
            }
            if (gamepad2.dpadLeftWasPressed()){
                robot.operatorStateMachine.addToQueue(SpindexerColorSensor.COLORS.PURPLE);
            }
            //Robot color sorting for operator
            if (gamepad2.dpadRightWasPressed()){
                robot.operatorStateMachine.addToQueue(SpindexerColorSensor.COLORS.GREEN);
            }
            if (gamepad2.touchpadWasPressed()){
                for (int i = 0; i < 3; i++){
                    robot.spindexer.clearColor(i);
                }
            }

            if (gamepad1.dpadDownWasPressed()){
                robot.launcher.setLaunchState(Launcher.LAUNCH_STATES.CLOSE);
            }
            if (gamepad1.dpadLeftWasPressed()){
                robot.launcher.setLaunchState(Launcher.LAUNCH_STATES.MID);
            }
            if(gamepad1.dpadUpWasPressed()){
                robot.launcher.setLaunchState(Launcher.LAUNCH_STATES.FAR);
            }
            if (gamepad2.leftBumperWasPressed() && gamepad2.dpad_down){
                robot.lifter.lift();
                robot.led.discoParty();
            }


            robot.operatorStateMachine.updateStateMachine();
            robot.lifter.update();

            // end of automated mode code

            robot.rotationControl.changeTargetByJoystick(gamepad1.right_stick_x,robot.odometry.getRobotAngle());
            robot.driveTrain.setTurnPriority(1.0);
            //sets drive power and what gamepad does
            robot.driveTrain.setDrivePower(-gamepad1.left_stick_y, gamepad1.left_stick_x, robot.rotationControl.getOutputPower(robot.odometry.getRobotAngle()), robot.odometry.getRobotAngle());


            robot.updateRobot(false, false, false);

            //Ramping up flywheel
            robot.launcher.revFlywheel();
            robot.bulkSensorBucket.clearCache();
            telemetry.update();
        }
    }
}
