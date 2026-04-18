package org.firstinspires.ftc.teamcode.Opmodes.Autonomous.SWEEP;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Opmodes.BaseAuto;
import org.firstinspires.ftc.teamcode.Resources.SWEEP.PathPlanning;
import org.firstinspires.ftc.teamcode.Resources.SWEEP.RobotActions;

@Autonomous
public class Blue3and9 extends BaseAuto {

    /// Route definition methods:
    /// all units are in inches, degrees, and seconds.
    /// path.splineStart(x,y,angle) the start of the path, call once
    /// path.splineTo(x,y,speedRatio) go through this point and the angle will be the direction the robot is travelling
    /// path.splineTo(x,y,angle,speedRatio) go through a specified point while keeping the angle of the robot at a constant.
    /// path.chill(x,y,angle,duration) Wait at a specified position with a given time in seconds
    private void defineRoute(){
        PathPlanning path = robot.SWEEP.pathPlanner;
        path.splineStart(-64.22,-34.88,-180);
        path.addAction(RobotActions.Actions.PREPPING);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.splineToConstantAngle(-20.5, -14.5, -140, 0.6);
        // get to launch position
        path.chill(0.3); // stablize rotation
        path.addAction(RobotActions.Actions.LAUNCH);
        path.chill(0.7);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.addAction(RobotActions.Actions.INTAKE);
        // go for the first set
        path.splineToConstantAngle(-14, -10, -270, 0.6);
        path.splineToConstantAngle(-14.5, -45.5, -270, 0.62);
        path.chill( 0.8);
        path.splineToConstantAngle(-6, -30.5, -180, 0.6);
        path.chill( 0.2);
        path.splineToConstantAngle(-5.5, -52.5, -180, 0.6); // stops intaking
        path.addAction(RobotActions.Actions.SCAN_MOTIF);
        path.addAction(RobotActions.Actions.PREPPING);
        path.chill( 1); // holding gate
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.splineToConstantAngle(-20.5, -18.5, -140, 0.75);
        path.addAction(RobotActions.Actions.SCAN_MOTIF);
        path.chill( 0.3);
        path.addAction(RobotActions.Actions.LAUNCH);
        path.chill(1.5);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.addAction(RobotActions.Actions.INTAKE);
        // go for the second set
        path.splineToConstantAngle(12.5, -10, -270, 0.95);
        path.splineToConstantAngle(12.5, -58.5, -270, 0.62);
        path.chill( 0.5);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.splineTo(10, -30.5, 0.95);
        path.splineToConstantAngle(-20.5, -18.5, -140, 0.75);
        path.addAction(RobotActions.Actions.PREPPING);
        path.chill( 0.3);
        path.addAction(RobotActions.Actions.LAUNCH);
        path.chill(1.5);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.addAction(RobotActions.Actions.INTAKE);
        // go for the third set
        path.splineTo(15,-15,1);
        path.splineToConstantAngle(34, -12, -270, 0.85);
        path.splineToConstantAngle(39, -58.5, -270, 0.62);
        path.chill( 0.6);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.splineToConstantAngle(-43, -13, -90, 0.9);
        path.addAction(RobotActions.Actions.PREPPING);
        path.chill( 0.3);
        path.addAction(RobotActions.Actions.LAUNCH);
        path.chill(1.3);
        path.addAction(RobotActions.Actions.IDLE);
        path.splineEnd(-43,-13,-90);
    }


    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true,true,true);
        defineRoute();
        waitForStart();
        robot.SWEEP.computeSplines();
        robot.odometry.overridePosition(-64.22,-34.88,-180);
        robot.setAlliance("blue");
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