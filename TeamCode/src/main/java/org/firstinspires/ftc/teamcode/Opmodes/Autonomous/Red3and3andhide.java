package org.firstinspires.ftc.teamcode.Opmodes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Opmodes.BaseAuto;
import org.firstinspires.ftc.teamcode.Resources.SWEEP.PathPlanning;
import org.firstinspires.ftc.teamcode.Resources.SWEEP.RobotActions;

@Autonomous
public class Red3and3andhide extends BaseAuto {

    /// Route definition methods:
    /// all units are in inches, degrees, and seconds.
    /// path.splineStart(x,y,angle) the start of the path, call once
    /// path.splineTo(x,y,speedRatio) go through this point and the angle will be the direction the robot is travelling
    /// path.splineTo(x,y,angle,speedRatio) go through a specified point while keeping the angle of the robot at a constant.
    /// path.chill(x,y,angle,duration) Wait at a specified position with a given time in seconds
    private void defineRoute(){
        PathPlanning path = robot.SWEEP.pathPlanner;
        path.splineStart(70, 14.5, 180);
        path.addAction(RobotActions.Actions.PREPPING);
        path.splineToConstantAngle(62.5, 13, 180, 1);
        path.chill(62.5, 13, 180, 1);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.addAction(RobotActions.Actions.LAUNCH);
        path.splineToConstantAngle(62.5, 13, 180, 1);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.chill(62.5, 13, 180, 1);
        path.addAction(RobotActions.Actions.INTAKE);
        path.splineToConstantAngle(61.5, 65.5, 270, 1);
        path.splineToConstantAngle(62.5, 8.5, 180, 1);
        path.chill(62.5, 8.5, 180, 1);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.addAction(RobotActions.Actions.LAUNCH);
        path.splineToConstantAngle(62.5, 8.5, 180, 1);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.chill(62.5, 8.5, 180, 1);
        path.splineToConstantAngle(62.5, 8.5, 180, 1);
        path.chill(62.5, 8.5, 180, 1);
        path.splineEnd(64.5, 38, 18);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true,true,true);
        defineRoute();
        waitForStart();
        robot.SWEEP.computeSplines();
        robot.odometry.overridePosition(-64.22,34.88,180);
        robot.SWEEP.startPath();
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