package org.firstinspires.ftc.teamcode.Opmodes.Autonomous.SWEEP;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Opmodes.BaseAuto;
import org.firstinspires.ftc.teamcode.Resources.SWEEP.PathPlanning;
import org.firstinspires.ftc.teamcode.Resources.SWEEP.RobotActions;

@Autonomous
public class CloseOpenGateALotBlue extends BaseAuto {

    /// Route definition methods:
    /// all units are in inches, degrees, and seconds.
    /// path.splineStart(x,y,angle) the start of the path, call once
    /// path.splineTo(x,y,speedRatio) go through this point and the angle will be the direction the robot is travelling
    /// path.splineTo(x,y,angle,speedRatio) go through a specified point while keeping the angle of the robot at a constant.
    /// path.chill(x,y,angle,duration) Wait at a specified position with a given time in seconds
    private void defineRoute(){
        PathPlanning path = robot.SWEEP.pathPlanner;
        path.splineStart(-64, -35, -180);
       path.splineToConstantAngle(-23.5, -16, -140, 1);
//        path.addAction(RobotActions.Actions.PREPPING);
//        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
//        path.addAction(RobotActions.Actions.LAUNCH);
//        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.chill(-6.5, -16, -140, 1.5);

        path.splineToConstantAngle(11, -29, -270, 1);
        path.addAction(RobotActions.Actions.INTAKE);
        path.splineToConstantAngle(15, -57, -270, 0.3);
        path.splineTo(11, -29, 1);
      // path.splineToConstantAngle(-23.5, -16, -140, 1);
//        path.addAction(RobotActions.Actions.PREPPING);
//        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
//        path.addAction(RobotActions.Actions.LAUNCH);
//        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.chill(-6.5, -16, -140, 1.5);
        path.splineToConstantAngle(11, -29, -270, 1);
        path.addAction(RobotActions.Actions.INTAKE);
        path.splineToConstantAngle(11, -57, -300, 0.3);
        path.chill(11,-58,-300,1);
        path.splineTo(11, -29, 1);
//        path.splineToConstantAngle(-23.5, -16, -140, 1);
//        path.addAction(RobotActions.Actions.PREPPING);
//        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
//        path.addAction(RobotActions.Actions.LAUNCH);
//        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.chill(-6.5, -16, -140, 1.5);
        path.splineToConstantAngle(11, -29, -270, 1);
        path.addAction(RobotActions.Actions.INTAKE);
        path.splineToConstantAngle(11, -57, -300, 0.3);
        path.chill(11,-58,-300,1);
        path.splineTo(11, -29, 1);
        path.splineToConstantAngle(-6.5, -16, -140, 1);
//        path.addAction(RobotActions.Actions.PREPPING);
//        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
//        path.addAction(RobotActions.Actions.LAUNCH);
//        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.chill(-6.5, -16, -140, 1.5);
        path.splineToConstantAngle(11, -29, -270, 1);
        path.addAction(RobotActions.Actions.INTAKE);
        path.splineToConstantAngle(11, -57, -300, 0.3);
        path.chill(11,-58,-300,1);
        path.splineTo(11, -29, 1);
//        path.splineToConstantAngle(-23.5, -16, -140, 1);
//        path.addAction(RobotActions.Actions.PREPPING);
//        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
//        path.addAction(RobotActions.Actions.LAUNCH);
//        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.chill(-6.5, -16, -140, 1.5);
        path.splineToConstantAngle(11, -29, -270, 1);
        path.addAction(RobotActions.Actions.INTAKE);
        path.splineToConstantAngle(11, -57, -300, 0.3);
        path.chill(11,-58,-300,1);
        path.splineTo(11, -29, 1);
        path.splineToConstantAngle(-6.5, -16, -140, 1);
//        path.addAction(RobotActions.Actions.PREPPING);
//        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
//        path.addAction(RobotActions.Actions.LAUNCH);
//        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.chill(-6.5, -16, -140, 1.5);
        path.addAction(RobotActions.Actions.INTAKE);
        path.splineToConstantAngle(-9, -49, -270, 0.3);
        path.chill(-9, -49, -270, 1);
        path.splineEnd(-36.5, -16, -140);
//        path.addAction(RobotActions.Actions.PREPPING);
//        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
//        path.addAction(RobotActions.Actions.LAUNCH);
//        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.chill(-36.5, -16, -140, 1.5);
    }


    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true,true,true);
        defineRoute();
        waitForStart();
        robot.SWEEP.computeSplines();
        robot.odometry.overridePosition(-64.22,-34.88,180);
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