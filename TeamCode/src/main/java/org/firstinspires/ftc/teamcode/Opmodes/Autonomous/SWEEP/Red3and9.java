package org.firstinspires.ftc.teamcode.Opmodes.Autonomous.SWEEP;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Opmodes.BaseAuto;
import org.firstinspires.ftc.teamcode.Resources.SWEEP.PathPlanning;
import org.firstinspires.ftc.teamcode.Resources.SWEEP.RobotActions;

@Autonomous
public class Red3and9 extends BaseAuto {

    /// Route definition methods:
    /// all units are in inches, degrees, and seconds.
    /// path.splineStart(x,y,angle) the start of the path, call once
    /// path.splineTo(x,y,speedRatio) go through this point and the angle will be the direction the robot is travelling
    /// path.splineTo(x,y,angle,speedRatio) go through a specified point while keeping the angle of the robot at a constant.
    /// path.chill(x,y,angle,duration) Wait at a specified position with a given time in seconds
    private void defineRoute(){
        PathPlanning path = robot.SWEEP.pathPlanner;
        path.splineStart(-64.22,34.88,180);

        path.addAction(RobotActions.Actions.PREPPING);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.splineToConstantAngle(-26, 17.5, 124, 0.7);
        // get to launch position
        path.chill(-26, 17.5, 124, 0.4); // stablize rotation

        path.addAction(RobotActions.Actions.LAUNCH);
        path.chill(-26,17.5,124,0.6);

        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);

        path.addAction(RobotActions.Actions.INTAKE);

        // go for the first set
        path.splineToConstantAngle(-14, 15, 270, 0.6);
        path.splineToConstantAngle(-14.5, 50.5, 270, 0.4);
        path.chill(-14.5, 51.5, 270, 2);
        path.splineToConstantAngle(-14.5, 50.5, 270, 0.8);
        path.splineToConstantAngle(-12.5, 38.5, 180, 1);
        path.chill(-12.5, 38.5, 180, 0.2);

        path.splineToConstantAngle(-12.5, 38.5, 180, 1); // stops intaking
        path.addAction(RobotActions.Actions.PREPPING);
        path.splineToConstantAngle(-12.5,38.5,180,0.5);
        path.chill(-12.5, 38.5, 180, 0.7); // holding gate

        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);

        path.splineToConstantAngle(-20.5, 18.5, 140, 0.75);


        path.chill(-20.5, 18.5, 140, 1.4);
        path.addAction(RobotActions.Actions.LAUNCH);

        path.chill(-20,18.5,140,2);


        path.splineEnd(0, 37, 90);
    }

//    PathPlanning path = robot.SWEEP.pathPlanner;
//        path.splineStart(-64, 35, -180);
//        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
//        path.splineToConstantAngle(-49, 21.5, 124, 0.5);
//        path.addAction(RobotActions.Actions.PREPPING);
//        path.chill(-49, 21.5, 124, 0.3);
//        path.splineToConstantAngle(-49, 21.5, 124, 0.5);
//        path.addAction(RobotActions.Actions.LAUNCH);
//        path.chill(-49, 21.5, 124, 0.5);
//        path.splineToConstantAngle(-49, 21.5, 124, 0.8);
//        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
//        path.addAction(RobotActions.Actions.INTAKE);
//        path.splineToConstantAngle(-12.5, 26.5, 270, 0.8);
//        path.splineToConstantAngle(-14.5, 54.5, 270, 0.8);
//        path.chill(-14.5, 54.5, 270, 0.5);
//        path.splineToConstantAngle(-12.5, 38.5, 180, 0.8);
//        path.splineToConstantAngle(-4, 54.5, 180, 0.8);
//        path.chill(-4, 54.5, 180, 0.5);
//        path.splineToConstantAngle(-20.5, 18.5, 140, 0.75);
//        path.addAction(RobotActions.Actions.PREPPING);
//        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
//        path.chill(-20.5, 18.5, 140, 0.9);
//        path.addAction(RobotActions.Actions.LAUNCH);
//        path.splineToConstantAngle(-20.5, 18.5, 140, 1);
//        path.chill(-20.5, 18.5, 140, 1.4);
//        path.addAction(RobotActions.Actions.INTAKE);
//        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
//        path.splineToConstantAngle(12.5, 28.5, 278, 0.9);
//        path.chill(12.5, 28.5, 278, 0.4);
//        path.splineToConstantAngle(16.5, 56.5, 270, 0.9);
//        path.chill(16.5, 56.5, 270, 0.4);
//        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
//        path.splineToConstantAngle(-20.5, 18.5, 140, 0.9);
//        path.addAction(RobotActions.Actions.LAUNCH);
//        path.chill(-20.5, 18.5, 140, 0.75);
//        path.splineToConstantAngle(-20.5, 18.5, 140, 1);
//        path.addAction(RobotActions.Actions.INTAKE);
//        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
//        path.chill(-20.5, 18.5, 140, 1.4);
//        path.splineToConstantAngle(32, 29, 270, 0.9);
//        path.splineToConstantAngle(38, 55, 270, 0.9);
//        path.splineToConstantAngle(31.5, 18.5, 140, 1);
//        path.splineToConstantAngle(-13, 12.5, 140, 0.9);
//        path.addAction(RobotActions.Actions.PREPPING);
//        path.splineToConstantAngle(-20.5, 19, 140, 1);
//        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
//        path.chill(-20.5, 19, 140, 1);
//        path.addAction(RobotActions.Actions.LAUNCH);
//        path.splineToConstantAngle(-20.5, 18.5, 140, 0.9);
//        path.chill(-20.5, 18.5, 140, 0.75);
//        path.addAction(RobotActions.Actions.IDLE);
//        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
//        path.splineToConstantAngle(-20.5, 18.5, 140, 1);
//        path.chill(-20.5, 18.5, 140, 1.4);
//        path.splineEnd(0, 37, 90);

    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true,true,true);
        defineRoute();
        waitForStart();
        robot.SWEEP.computeSplines();
        robot.odometry.overridePosition(-64.22,34.88,180);
//        robot.odometry.overridePosition(0,0,0);
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