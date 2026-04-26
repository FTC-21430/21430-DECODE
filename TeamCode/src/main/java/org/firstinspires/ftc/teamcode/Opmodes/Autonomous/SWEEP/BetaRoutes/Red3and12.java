package org.firstinspires.ftc.teamcode.Opmodes.Autonomous.SWEEP.BetaRoutes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Opmodes.BaseAuto;
import org.firstinspires.ftc.teamcode.Resources.SWEEP.PathPlanning;
import org.firstinspires.ftc.teamcode.Resources.SWEEP.RobotActions;
@Autonomous
@Disabled
public class Red3and12 extends BaseAuto {

    /// Route definition methods:
    /// all units are in inches, degrees, and seconds.
    /// path.splineStart(x,y,angle) the start of the path, call once
    /// path.splineTo(x,y,speedRatio) go through this point and the angle will be the direction the robot is travelling
    /// path.splineTo(x,y,angle,speedRatio) go through a specified point while keeping the angle of the robot at a constant.
    /// path.chill(x,y,angle,duration) Wait at a specified position with a given time in seconds
    private void defineRoute(){
        PathPlanning path = robot.SWEEP.pathPlanner;
        path.splineStart(-64.22, 34.88, 180);
        path.splineToConstantAngle(-48.75, 21.49, 124, 0.5);
        path.addAction(RobotActions.Actions.PREPPING);
        path.chill(-48.75, 21.49, 124, 1);
        path.addAction(RobotActions.Actions.LAUNCH);
        path.chill(-48.75, 21.49, 124, 2);
        path.splineToConstantAngle(-11.2, 25.6, 270, 0.5);
        path.addAction(RobotActions.Actions.INTAKE);
        path.splineToConstantAngle(-2.2, 49.5, 270, 0.3);
        path.splineToConstantAngle(-1.191, 49.73, 270, 0.3);
        path.addAction(RobotActions.Actions.PREPPING);
        path.chill(-1.7, 53.6, 270, 0.25);
        path.splineToConstantAngle(-21.36, 17.65, 133.8, 1);
        path.chill(-21.36, 17.65, 133.8, 1);
        path.addAction(RobotActions.Actions.LAUNCH);
        path.chill(-21.36, 17.65, 136.8, 1);
        path.chill(-21.36, 17.65, 136.8, 2);
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