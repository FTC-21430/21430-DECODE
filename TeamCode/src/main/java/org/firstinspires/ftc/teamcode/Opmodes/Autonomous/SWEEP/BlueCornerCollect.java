package org.firstinspires.ftc.teamcode.Opmodes.Autonomous.SWEEP;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Opmodes.BaseAuto;
import org.firstinspires.ftc.teamcode.Resources.SWEEP.PathPlanning;
import org.firstinspires.ftc.teamcode.Resources.SWEEP.RobotActions;

@Autonomous
public class BlueCornerCollect extends BaseAuto {


    /// Route definition methods:
    /// all units are in inches, degrees, and seconds.
    /// path.splineStart(x,y,angle) the start of the path, call once
    /// path.splineTo(x,y,speedRatio) go through this point and the angle will be the direction the robot is travelling
    /// path.splineTo(x,y,angle,speedRatio) go through a specified point while keeping the angle of the robot at a constant.
    /// path.chill(x,y,angle,duration) Wait at a specified position with a given time in seconds
    private void defineRoute(){
        PathPlanning path = robot.SWEEP.pathPlanner; // have a shorthand variable name to make typing this easier.
        //TODO: All positions are guessed with a slightly large auto pather, where goes off the map, fine tune!!!!!!
        path.splineStart(63.5, -20.5, 0);
        path.addAction(RobotActions.Actions.PREPPING);
        //Wait to rev flywheel
        path.splineTo(56.5, -11.5, 0.7);
        path.addAction(RobotActions.Actions.LAUNCH);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        //Wait to launch balls
        path.splineTo(57, -19.5, 0.7);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.addAction(RobotActions.Actions.INTAKE);
        path.splineToConstantAngle(63.5, -65.5, 90, 0.7);
        path.splineTo(64.5, -47.5, 0.7);
        //Wait to intake artifacts
        path.addAction(RobotActions.Actions.PREPPING);
        path.splineTo(66, -15, 0.7);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.addAction(RobotActions.Actions.LAUNCH);
        //Wait to launch
        path.splineTo(67.5, -26, 0.7);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.splineEnd(68.5, -41, 0);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true,true,true);
        defineRoute();
        waitForStart();
        robot.SWEEP.computeSplines();
        robot.odometry.overridePosition(0,0,0);
        robot.SWEEP.startPath();
        while (opModeIsActive() && !robot.SWEEP.isPathComplete()){
            robot.odometry.updateOdometry();
            robot.SWEEP.update(robot.odometry.getOdometryPacket());
            //TODO: update robot system loops
            robot.driveTrain.setDrivePower(robot.SWEEP.getForwardPower(),robot.SWEEP.getSidePower(),robot.SWEEP.getRotationPower(),robot.odometry.getRobotAngle());
            telemetry.update();
            robot.bulkSensorBucket.clearCache();
        }
    }
}