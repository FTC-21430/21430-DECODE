package org.firstinspires.ftc.teamcode.Opmodes.Autonomous.SWEEP;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Opmodes.BaseAuto;
import org.firstinspires.ftc.teamcode.Resources.SWEEP.PathPlanning;
import org.firstinspires.ftc.teamcode.Resources.SWEEP.RobotActions;

@Disabled
@Autonomous
public class RedFar6and9 extends BaseAuto {


    /// Route definition methods:
    /// all units are in inches, degrees, and seconds.
    /// path.splineStart(x,y,angle) the start of the path, call once
    /// path.splineTo(x,y,speedRatio) go through this point and the angle will be the direction the robot is travelling
    /// path.splineTo(x,y,angle,speedRatio) go through a specified point while keeping the angle of the robot at a constant.
    /// path.chill(x,y,angle,duration) Wait at a specified position with a given time in seconds
    private void defineRoute(){
        PathPlanning path = robot.SWEEP.pathPlanner; // have a shorthand variable name to make typing this easier.
        path.splineStart(63.5, 20.5, 0);
        path.addAction(RobotActions.Actions.PREPPING);
        //Wait to prep
        path.splineTo(56.5, 11.5, 0.7);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.addAction(RobotActions.Actions.LAUNCH);
        //Wait to shoot
        path.splineToConstantAngle(57, 19.5, -90, 0.7);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.addAction(RobotActions.Actions.INTAKE);
        path.splineToConstantAngle(63.5, 65.5, -90, 0.7);
        //Wait to intake
        path.splineToConstantAngle(64.5, 47.5, -90, 0.7);
        path.addAction(RobotActions.Actions.PREPPING);
        path.splineTo(60, 9, 0.7);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.addAction(RobotActions.Actions.LAUNCH);
        //Wait to launch
        path.splineTo(50, 9, 1);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.splineToConstantAngle(21, 19.5, 180, 1);
        path.addAction(RobotActions.Actions.SCAN_MOTIF);
        path.addAction(RobotActions.Actions.SCAN_MOTIF);
        path.addAction(RobotActions.Actions.INTAKE);
        path.splineToConstantAngle(14, 31.5, 270, 1);
        path.splineToConstantAngle(12, 53, 270, 1);
        //Wait to intake
        path.splineToConstantAngle(3, 58, 180, 1);
        path.splineTo(-2.5, 49, 1);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.addAction(RobotActions.Actions.PREPPING);
        path.splineTo(-7.5, 22, 1);
        path.addAction(RobotActions.Actions.LAUNCH);
        path.splineTo(-13, 26.5, 1);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.addAction(RobotActions.Actions.INTAKE);
        path.splineToConstantAngle(-14, 36, 270, 1);
        path.splineToConstantAngle(-12.5, 52.5, 270, 1);
        path.splineTo(-21.5, 42, 1);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.addAction(RobotActions.Actions.PREPPING);
        path.splineTo(-13, 20, 1);
        path.addAction(RobotActions.Actions.LAUNCH);
        //Wait to launch
        path.splineTo(-3, 19.5, 1);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.splineToConstantAngle(34, 18, 0, 1);
        path.addAction(RobotActions.Actions.INTAKE);
        path.splineToConstantAngle(38.5, 26, 270, 1);
        path.splineToConstantAngle(38, 54.5, 270, 0.7);
        //Wait to intake
        path.addAction(RobotActions.Actions.PREPPING);
        path.splineTo(44.5, 35, 1);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.splineTo(55.5, 11, 1);
        path.addAction(RobotActions.Actions.LAUNCH);
        //Wait to launch
        path.splineTo(61.5, 15.5, 1);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.splineEnd(68, 42.5, 0);
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