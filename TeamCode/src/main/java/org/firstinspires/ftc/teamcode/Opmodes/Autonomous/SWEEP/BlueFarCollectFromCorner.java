package org.firstinspires.ftc.teamcode.Opmodes.Autonomous.SWEEP;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Opmodes.BaseAuto;
import org.firstinspires.ftc.teamcode.Resources.SWEEP.GlobalPositions.POS;
import org.firstinspires.ftc.teamcode.Resources.SWEEP.PathPlanning;
import org.firstinspires.ftc.teamcode.Resources.SWEEP.RobotActions;

@Autonomous
public class BlueFarCollectFromCorner extends BaseAuto {

    /// Route definition methods:
    /// all units are in inches, degrees, and seconds.
    /// path.splineStart(x,y,angle) the start of the path, call once
    /// path.splineTo(x,y,speedRatio) go through this point and the angle will be the direction the robot is travelling
    /// path.splineTo(x,y,angle,speedRatio) go through a specified point while keeping the angle of the robot at a constant.
    /// path.chill(x,y,angle,duration) Wait at a specified position with a given time in seconds
    private void defineRoute(){
        PathPlanning path = robot.SWEEP.pathPlanner;
        path.splineStart(POS.FAR_START);
        path.addAction(RobotActions.Actions.PREPPING);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.splineToConstantAngle(POS.FAR_3, 0.4);
        path.chill(POS.FAR_3, 1.2);

        path.addAction(RobotActions.Actions.LAUNCH);
        path.chill(POS.FAR_3,0.8);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        for (int i = 0; i < 5; i++) {
            path.addAction(RobotActions.Actions.INTAKE);
            path.splineToConstantAngle(POS.INTAKE_START_CORNER);
            path.splineToConstantAngle(POS.INTAKE_END_CORNER, 0.8);
            path.chill(0.65);
            path.splineToConstantAngle(POS.FAR_3, 0.75);
            path.addAction(RobotActions.Actions.PREPPING);
            path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
            path.chill(0.5);
            path.addAction(RobotActions.Actions.LAUNCH);
            path.chill(0.9);
            path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
            path.addAction(RobotActions.Actions.IDLE);
        }
        path.splineEnd(POS.FAR_END);
    }


    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true,true,true,"blue)");
        defineRoute();
        waitForStart();
        robot.SWEEP.computeSplines();
        robot.odometry.overridePosition(63.5, -20.5, -180);
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
