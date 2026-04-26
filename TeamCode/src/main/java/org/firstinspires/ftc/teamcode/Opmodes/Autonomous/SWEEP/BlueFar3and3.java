package org.firstinspires.ftc.teamcode.Opmodes.Autonomous.SWEEP;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Opmodes.BaseAuto;
import org.firstinspires.ftc.teamcode.Resources.SWEEP.GlobalPositions.POS;
import org.firstinspires.ftc.teamcode.Resources.SWEEP.PathPlanning;
import org.firstinspires.ftc.teamcode.Resources.SWEEP.RobotActions;
import org.firstinspires.ftc.teamcode.Resources.SWEEP.Waypoint;

@Autonomous(name = "BlueFar3+3", group = "BlueAutonomous", preselectTeleOp = "BlueTeleopPostAuto")
public class BlueFar3and3 extends BaseAuto {

    /// Route definition methods:
    /// all units are in inches, degrees, and seconds.
    /// path.splineStart(x,y,angle) the start of the path, call once
    /// path.splineTo(x,y,speedRatio) go through this point and the angle will be the direction the robot is travelling
    /// path.splineTo(x,y,angle,speedRatio) go through a specified point while keeping the angle of the robot at a constant.
    /// path.chill(x,y,angle,duration) Wait at a specified position with a given time in seconds
    private void defineRoute(){
        PathPlanning path = robot.SWEEP.pathPlanner;
        Waypoint launchPoint = new Waypoint(56,-25,-158.3, 0.9,true);
        path.splineStart(POS.FAR_START);
        path.addAction(RobotActions.Actions.SET_CONSTANT_TRAJECTORY_FAR);
        path.addAction(RobotActions.Actions.SCAN_MOTIF);

        path.chill(1.4);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);

        path.splineToConstantAngle(launchPoint, 0.7);
        path.addAction(RobotActions.Actions.PREPPING);
        path.chill(1);
        path.addAction(RobotActions.Actions.LAUNCH);
        path.chill(2);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.addAction(RobotActions.Actions.INTAKE);
        path.addAction(RobotActions.Actions.SCAN_MOTIF);
        path.splineToConstantAngle(POS.INTAKE_START_CORNER);
        path.splineToConstantAngle(POS.INTAKE_END_CORNER, 0.6);
        path.chill(0.9);
        path.splineToConstantAngle(launchPoint, 0.8);
        path.addAction(RobotActions.Actions.PREPPING);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.chill(1.5);
        path.addAction(RobotActions.Actions.LAUNCH);
        path.chill(2);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.splineEnd(POS.FAR_END);
    }


    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true,true,true,"blue");
        defineRoute();
        waitForStart();
        robot.SWEEP.computeSplines();
        robot.odometry.overridePosition(60.5, -21.5, -180);
        robot.setAlliance("blue");
//        robot.odometry.overridePosition(0,0,0);
        robot.SWEEP.startPath();
        robot.motifId = 21;
        robot.rotationControl.setPIDController(0.0202,0.0005,0.00076);
        robot.operatorStateMachine.SUPER_FAR_MODE = true;
        while (opModeIsActive() && !robot.SWEEP.isPathComplete()){
            robot.odometry.updateOdometry();
            robot.SWEEP.update(robot.odometry.getOdometryPacket());
            robot.updateRobot(false,false,false);
            robot.operatorStateMachine.updateStateMachine();
            robot.driveTrain.setDrivePower(robot.SWEEP.getForwardPower(),robot.SWEEP.getSidePower(),robot.SWEEP.getRotationPower(),robot.odometry.getRobotAngle());
            telemetry.update();
            robot.bulkSensorBucket.clearCache();
        }
    }
}