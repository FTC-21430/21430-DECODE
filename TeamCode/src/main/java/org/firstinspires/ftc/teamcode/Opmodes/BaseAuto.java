package org.firstinspires.ftc.teamcode.Opmodes;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Firmware.OperatorStateMachine;
import org.firstinspires.ftc.teamcode.Firmware.Robot;
import org.firstinspires.ftc.teamcode.Firmware.Systems.SpindexerColorSensor;

// This class will have the autonomous functions applicable to every auto. All autos extend BaseAuto.
@Config
abstract public class BaseAuto extends org.firstinspires.ftc.teamcode.Opmodes.GeneralOpMode {
    public int motifId = 21;

    public void detectMotifWhileMoveTo(double targetX, double targetY, double robotAngle, double targetCircle) {
        robot.pathFollowing.setTargetPosition(targetX, targetY);
        robot.pathFollowing.setFollowTolerance(targetCircle);
        robot.rotationControl.setTargetAngle(robotAngle);
        robot.driveTrain.fieldCentricDriving(false);
        while (!robot.pathFollowing.isWithinTargetTolerance(robot.odometry.getRobotX(), robot.odometry.getRobotY()) && robot.opMode.opModeIsActive()) {
            int tempId = robot.aprilTags.getMotifID();
            if (tempId != 0) {
                motifId = tempId;
            }
            robot.updateRobot(false, false, false);
            robot.pathFollowing.followPath(robot.odometry.getRobotX(), robot.odometry.getRobotY(), robot.odometry.getRobotAngle());
            robot.driveTrain.setDrivePower(robot.pathFollowing.getPowerS(), robot.pathFollowing.getPowerF(), robot.rotationControl.getOutputPower(robot.odometry.getRobotAngle()), robot.odometry.getRobotAngle());
            telemetry.update();
        }
    }

    public void autonomousLaunching(int motifId) {

        switch (motifId){
            case 0:
                for (int i = 0; i < 3; i++){
                    robot.operatorStateMachine.addToQueue(SpindexerColorSensor.COLORS.NONE);
                }
                break;
            case 21:
                robot.operatorStateMachine.addToQueue(SpindexerColorSensor.COLORS.GREEN);
                robot.operatorStateMachine.addToQueue(SpindexerColorSensor.COLORS.PURPLE);
                robot.operatorStateMachine.addToQueue(SpindexerColorSensor.COLORS.PURPLE);
                break;
            case 22:
                robot.operatorStateMachine.addToQueue(SpindexerColorSensor.COLORS.PURPLE);
                robot.operatorStateMachine.addToQueue(SpindexerColorSensor.COLORS.GREEN);
                robot.operatorStateMachine.addToQueue(SpindexerColorSensor.COLORS.PURPLE);
                break;
            case 23:
                robot.operatorStateMachine.addToQueue(SpindexerColorSensor.COLORS.PURPLE);
                robot.operatorStateMachine.addToQueue(SpindexerColorSensor.COLORS.PURPLE);
                robot.operatorStateMachine.addToQueue(SpindexerColorSensor.COLORS.GREEN);
                break;
        }

        robot.operatorStateMachine.moveToState(OperatorStateMachine.State.LAUNCH);
        while(robot.operatorStateMachine.getCurrentState() == OperatorStateMachine.State.LAUNCH) {
            robot.updateRobot(false, false, false);
            robot.pathFollowing.followPath(robot.odometry.getRobotX(), robot.odometry.getRobotY(), robot.odometry.getRobotAngle());
            robot.driveTrain.setDrivePower(robot.pathFollowing.getPowerS(), robot.pathFollowing.getPowerF(), robot.rotationControl.getOutputPower(robot.odometry.getRobotAngle()), robot.odometry.getRobotAngle());
            robot.operatorStateMachine.updateStateMachine();
            telemetry.update();
        }
    }
}
