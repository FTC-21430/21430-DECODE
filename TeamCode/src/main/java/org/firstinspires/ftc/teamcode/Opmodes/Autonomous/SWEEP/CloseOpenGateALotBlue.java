package org.firstinspires.ftc.teamcode.Opmodes.Autonomous.SWEEP;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Firmware.OperatorStateMachine;
import org.firstinspires.ftc.teamcode.Opmodes.BaseAuto;
import org.firstinspires.ftc.teamcode.Resources.SWEEP.GlobalPositions.POS;
import org.firstinspires.ftc.teamcode.Resources.SWEEP.PathPlanning;
import org.firstinspires.ftc.teamcode.Resources.SWEEP.RobotActions;
import org.firstinspires.ftc.teamcode.Resources.SWEEP.Waypoint;

@Autonomous(name = "BlueCloseGateCycle", group = "BlueAutonomous", preselectTeleOp = "BlueTeleopPostAuto")
public class CloseOpenGateALotBlue extends BaseAuto {

    /// Route definition methods:
    /// all units are in inches, degrees, and seconds.
    /// path.splineStart(x,y,angle) the start of the path, call once
    /// path.splineTo(x,y,speedRatio) go through this point and the angle will be the direction the robot is travelling
    /// path.splineTo(x,y,angle,speedRatio) go through a specified point while keeping the angle of the robot at a constant.
    /// path.chill(x,y,angle,duration) Wait at a specified position with a given time in seconds
    private void defineRoute(){
        PathPlanning path = robot.SWEEP.pathPlanner;
        Waypoint launchPOS = new Waypoint(-16,-16,-134,1,true);
        path.splineStart(POS.CLOSE_START);
        path.addAction(RobotActions.Actions.SET_CONSTANT_TRAJECTORY_CLOSE);
        path.addAction(RobotActions.Actions.PREPPING);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.splineToConstantAngle(launchPOS,0.7);
        // get to launch position
        path.chill(0.3); // stablize rotation
        path.addAction(RobotActions.Actions.LAUNCH);
        path.chill(0.6);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.addAction(RobotActions.Actions.INTAKE);
        // go for the second set
        path.splineToConstantAngle(11.5, -18, -270, 1);
        path.splineToConstantAngle(14.5, -57.5, -270, 0.8);
        path.chill(0.1);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.splineToConstantAngle(POS.INTAKE_START_2, 1);
        path.splineToConstantAngle(launchPOS, 0.8);
        path.addAction(RobotActions.Actions.PREPPING);
        path.chill(0.3);
        path.addAction(RobotActions.Actions.LAUNCH);
        path.chill(0.8);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);


        for (int i = 0; i < 2 ; i ++){
            path.addAction(RobotActions.Actions.INTAKE);
            path.splineToConstantAngle(11, -40, 20,0.6);
            path.splineToConstantAngle(10, -55.6, 20,0.55);
            path.splineToConstantAngle(12.5, -57.8, 55,0.5);
            path.chill(1.8);
            path.splineToConstantAngle(10.3, -36,0,0.9);
//            path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
            path.addAction(RobotActions.Actions.PREPPING);
            path.addAction(RobotActions.Actions.SET_CONSTANT_TRAJECTORY_CLOSE);
            path.splineToConstantAngle(launchPOS, 0.6);
            path.chill(0.4);
            path.addAction(RobotActions.Actions.LAUNCH);
            path.chill(0.72);
//            path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        }
        path.addAction(RobotActions.Actions.INTAKE);
        path.splineToConstantAngle(POS.INTAKE_START_1, 0.8);
        path.chill(0.5);
        path.splineToConstantAngle(POS.INTAKE_END_1, 0.63);
        path.chill(0.35);

        path.addAction(RobotActions.Actions.SET_CONSTANT_TRAJECTORY_CLOSE);
        path.splineToConstantAngle(launchPOS, 0.6);
        path.addAction(RobotActions.Actions.PREPPING);
        path.chill(0.4);
        path.addAction(RobotActions.Actions.LAUNCH);
        path.chill(0.72);



        path.addAction(RobotActions.Actions.IDLE);
        path.splineEnd(10,-30,-124);
    }


    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true,true,true,"blue");
        defineRoute();
        waitForStart();
        robot.SWEEP.computeSplines();
        robot.odometry.overridePosition(-62.22,-38.5,180);
        robot.setAlliance("blue");
//        robot.odometry.overridePosition(0,0,0);
        robot.SWEEP.startPath();
        robot.rotationControl.setPIDController(0.0202,0.0005,0.00076);
        while (opModeIsActive() && !robot.SWEEP.isPathComplete()){
            robot.odometry.updateOdometry();
            robot.SWEEP.update(robot.odometry.getOdometryPacket());
            robot.updateRobot(false,false,false);
            robot.operatorStateMachine.updateStateMachine();
            robot.driveTrain.setDrivePower(robot.SWEEP.getForwardPower(),robot.SWEEP.getSidePower(),robot.SWEEP.getRotationPower(),robot.odometry.getRobotAngle());
            telemetry.update();
            robot.bulkSensorBucket.clearCache();
        }
        robot.operatorStateMachine.moveToState(OperatorStateMachine.State.IDLE);
        robot.operatorStateMachine.updateStateMachine();
    }
}
