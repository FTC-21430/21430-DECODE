package org.firstinspires.ftc.teamcode.Opmodes.Autonomous.OldRoutes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Opmodes.BaseAuto;
import org.firstinspires.ftc.teamcode.Resources.SWEEP.PathPlanning;
import org.firstinspires.ftc.teamcode.Resources.SWEEP.RobotActions;

@Disabled
@Autonomous
public class RedBeverSWEEP extends BaseAuto {

    /// Route definition methods:
    /// all units are in inches, degrees, and seconds.
    /// path.splineStart(x,y,angle) the start of the path, call once
    /// path.splineTo(x,y,speedRatio) go through this point and the angle will be the direction the robot is travelling
    /// path.splineTo(x,y,angle,speedRatio) go through a specified point while keeping the angle of the robot at a constant.
    /// path.chill(x,y,angle,duration) Wait at a specified position with a given time in seconds
    private void defineRoute(){
        PathPlanning path = robot.SWEEP.pathPlanner;
        path.splineStart(-64, 35, -180);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.splineToConstantAngle(-49, 21.5, 124, 0.5);
        path.addAction(RobotActions.Actions.PREPPING);
        path.chill(-49, 21.5, 124, 0.3);
        path.splineToConstantAngle(-49, 21.5, 124, 1);
        path.addAction(RobotActions.Actions.LAUNCH);
        path.chill(-49, 21.5, 124, 0.5);
        path.splineToConstantAngle(-49, 21.5, 124, 0.5);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.addAction(RobotActions.Actions.INTAKE);
        path.splineToConstantAngle(-12.5, 36, 270, 0.5);
        path.splineToConstantAngle(-10, 55.5, 270, 0.5);
        path.chill(-10, 55.5, 270, 0.5);
        path.addAction(RobotActions.Actions.PREPPING);
        path.splineToConstantAngle(0.5, 50, 180, 0.7);
        path.splineToConstantAngle(2.5, 58, 180, 0.5);
        path.chill(2.5, 58, 180, 0.5);
        path.splineToConstantAngle(-20.5, 18.5, 140, 0.75);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.chill(-20.5, 18.5, 140, 0.75);
        path.addAction(RobotActions.Actions.LAUNCH);
        path.splineToConstantAngle(-20.5, 18.5, 140, 1);
        path.chill(-20.5, 18.5, 140, 1.4);
        path.addAction(RobotActions.Actions.INTAKE);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.splineToConstantAngle(11, 34, 270, 0.8);
        path.splineToConstantAngle(16.5, 60, 270, 0.8);
        path.chill(16.5, 60, 270, 0.4);
        path.splineToConstantAngle(-20.5, 18.5, 140, 0.5);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.chill(-20.5, 18.5, 140, 0.75);
        path.addAction(RobotActions.Actions.LAUNCH);
        path.splineToConstantAngle(-20.5, 18.5, 140, 1);
        path.chill(-20.5, 18.5, 140, 1.4);
        path.addAction(RobotActions.Actions.INTAKE);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.splineToConstantAngle(39, 30.5, 270, 0.5);
        path.splineToConstantAngle(38.5, 64.5, 270, 0.5);
        path.splineToConstantAngle(35, 40.5, 140, 1);
        path.addAction(RobotActions.Actions.PREPPING);
        path.splineToConstantAngle(-13, 12.5, 140, 0.5);
        path.splineToConstantAngle(-20.5, 19, 140, 1);
        path.chill(-20.5, 19, 140, 1);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.splineToConstantAngle(-20.5, 18.5, 140, 0.5);
        path.addAction(RobotActions.Actions.LAUNCH);
        path.chill(-20.5, 18.5, 140, 0.75);
        path.splineToConstantAngle(-20.5, 18.5, 140, 1);
        path.addAction(RobotActions.Actions.IDLE);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.chill(-20.5, 18.5, 140, 1.4);

        path.addAction(RobotActions.Actions.INTAKE);

        path.splineToConstantAngle(-9,43,176,0.7);
        path.splineToConstantAngle(15,43.5,177,0.7);

        path.chill(15,43.5,177, 0.28);

        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.splineToConstantAngle(-20.5, 18.5, 140, 0.5);
        path.addAction(RobotActions.Actions.LAUNCH);
        path.chill(-20.5, 18.5, 140, 0.75);
        path.splineToConstantAngle(-20.5, 18.5, 140, 1);
        path.addAction(RobotActions.Actions.IDLE);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);

//        // sorted cycle 2
//
//        //The robot moves to the place to intake the balls
        path.addAction(RobotActions.Actions.INTAKE);
        path.splineToConstantAngle(18.5,43,175,0.8);
        path.chill(34.3,43,175,0.2);
        path.splineToConstantAngle(34.3,43,175,0.8);
        path.splineToConstantAngle(18.5,40,175,0.8);
        path.chill(34.3,38.5,175,0.2);
        path.splineToConstantAngle(34.3,38.5,175,0.7);
        path.chill(34.3,38.5,175, 0.2);

        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.splineToConstantAngle(-20.5, 18.5, 140, 0.5);
        path.addAction(RobotActions.Actions.LAUNCH);
        path.chill(-20.5, 18.5, 140, 0.75);
        path.splineToConstantAngle(-20.5, 18.5, 140, 1);
        path.addAction(RobotActions.Actions.IDLE);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
//
//        // sorted cycle 3
//
////      park off of launch line and close to the gate to clear the classifier at teleop start
        path.splineToConstantAngle(-50, 18, 90, 0.7);

        path.splineEnd(-50, 18, 90);
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