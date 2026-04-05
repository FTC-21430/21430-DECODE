package org.firstinspires.ftc.teamcode.Opmodes.Autonomous;

import androidx.core.content.res.FontResourcesParserCompat;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Opmodes.BaseAuto;
import org.firstinspires.ftc.teamcode.Resources.SWEEP.PathPlanning;
import org.firstinspires.ftc.teamcode.Resources.SWEEP.RobotActions;

@Autonomous
public class RedBeverSWEEP extends BaseAuto {


    /// Route definition methods:
    /// all units are in inches, degrees, and seconds.
    /// path.splineStart(x,y,angle) the start of the path, call once
    /// path.splineTo(x,y,speedRatio) go through this point and the angle will be the direction the robot is travelling
    /// path.splineTo(x,y,angle,speedRatio) go through a specified point while keeping the angle of the robot at a constant.
    /// path.chill(x,y,angle,duration) Wait at a specified position with a given time in seconds
    private void defineRoute(){
        PathPlanning path = robot.SWEEP.pathPlanner; // have a shorthand variable name to make typing this easier.
        path.splineStart(-64.22,34.88,180); // the start of the path (0)
        path.addAction(RobotActions.Actions.PREPPING);
        path.splineToConstantAngle(-48.75,21.49,124,0.50); //launch spot (1)
        path.chill(-48.75,21.49,124, 1.5);
        path.addAction(RobotActions.Actions.LAUNCH);
        path.chill(-48.75,21.49,124, 1);
        path.addAction(RobotActions.Actions.INTAKE);
        path.splineToConstantAngle(-9.57,22.08,270,1); //pre-intake spot (1.5)
        path.splineToConstantAngle(-10.99,32.74,270,0.30); //intake spot (2)
        path.chill(-13.65,46.5,231,1.5);
        path.splineToConstantAngle(-6.18,41.85,180,1); // (2.5)
        path.splineToConstantAngle(-6.18,51.85,180,1); //gate spot (3)
        path.addAction(RobotActions.Actions.PREPPING);
        path.chill(-6.18,51.85,180,1);
        path.splineToConstantAngle(-21.36,17.65,133.8,1); //launch spot (4)
        path.chill(-21.36,17.65,136.8,1);
        path.addAction(RobotActions.Actions.LAUNCH);
        path.chill(-21.36,17.65,136.8,1);
        path.addAction(RobotActions.Actions.INTAKE);
        path.splineToConstantAngle(12.59,23.94,270,1); //pre-intake (5)
        path.splineToConstantAngle(12.73,40.7,270,0.30); //intake (6)
        path.chill(12.73,40.7,270,1);
        path.addAction(RobotActions.Actions.PREPPING);
        path.splineToConstantAngle(-4.6,12.5,145.4,1);//launch (7)
        path.chill(-4.6,12.5,145.4,1);
        path.addAction(RobotActions.Actions.LAUNCH);
        path.chill(-4.6,12.5,145.4,1);
        path.addAction(RobotActions.Actions.INTAKE);
        path.splineToConstantAngle(37.14,24.5,272,1);//pre-intake (8)
        path.splineToConstantAngle(36.4,39.9,271,0.3);//intake (9)
        path.addAction(RobotActions.Actions.PREPPING);
        path.splineToConstantAngle(-4.6,12.5,145.4,1);//launch (10, same as 7)
        path.chill(-4.6,12.5,145.4,1);
        path.addAction(RobotActions.Actions.LAUNCH);
        path.splineEnd(0,15,0);
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