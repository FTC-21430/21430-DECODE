package org.firstinspires.ftc.teamcode.Opmodes.Autonomous.SWEEP;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Opmodes.BaseAuto;
import org.firstinspires.ftc.teamcode.Resources.SWEEP.GlobalPositions;
import org.firstinspires.ftc.teamcode.Resources.SWEEP.PathPlanning;
import org.firstinspires.ftc.teamcode.Resources.SWEEP.RobotActions;

@Autonomous
public class BlueFar3and3 extends BaseAuto {

    /// Route definition methods:
    /// all units are in inches, degrees, and seconds.
    /// path.splineStart(x,y,angle) the start of the path, call once
    /// path.splineTo(x,y,speedRatio) go through this point and the angle will be the direction the robot is travelling
    /// path.splineTo(x,y,angle,speedRatio) go through a specified point while keeping the angle of the robot at a constant.
    /// path.chill(x,y,angle,duration) Wait at a specified position with a given time in seconds
    private void defineRoute(){
        PathPlanning path = robot.SWEEP.pathPlanner;
        path.splineStart(GlobalPositions.POS.FAR_START);
        path.addAction(RobotActions.Actions.PREPPING);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.chill(1);
        path.addAction(RobotActions.Actions.LAUNCH);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.splineToConstantAngle(GlobalPositions.POS.INTAKE_START_CORNER);
        path.splineToConstantAngle(GlobalPositions.POS.INTAKE_END_CORNER);
        path.chill(0.5);
        path.addAction(RobotActions.Actions.INTAKE);
        path.addAction(RobotActions.Actions.PREPPING);
        path.splineToConstantAngle(GlobalPositions.POS.FAR_3);
        path.chill(1);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.addAction(RobotActions.Actions.LAUNCH);
        path.addAction(RobotActions.Actions.TOGGLE_GOAL_AIMING);
        path.splineEnd(GlobalPositions.POS.FAR_START);
    }


    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true,true,true);
        defineRoute();
        waitForStart();
        robot.SWEEP.computeSplines();
        robot.odometry.overridePosition(-64.22,34.88,180);
        robot.setAlliance("red");
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
