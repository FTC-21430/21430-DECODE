package org.firstinspires.ftc.teamcode.Opmodes.Testing.SWEEP;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Opmodes.BaseAuto;
import org.firstinspires.ftc.teamcode.Resources.SWEEP.PathPlanning;

@Autonomous
public class SWEEPFullAutoTest extends BaseAuto {


    /// Route definition methods:
    /// all units are in inches, degrees, and seconds.
    /// path.splineStart(x,y,angle) the start of the path, call once
    /// path.splineTo(x,y,speedRatio) go through this point and the angle will be the direction the robot is travelling
    /// path.splineTo(x,y,angle,speedRatio) go through a specified point while keeping the angle of the robot at a constant.
    /// path.chill(x,y,angle,duration) Wait at a specified position with a given time in seconds
    private void defineRoute(){
        PathPlanning path = robot.SWEEP.pathPlanner; // have a shorthand variable name to make typing this easier.
        path.splineStart(64,16,90); // the start of the path

        path.splineToConstantAngle(49,10,150,1);
        path.chill(49,10,150,2);

        path.splineTo(55,30,0.8);
        path.splineToConstantAngle(64,62,270,1);
        path.chill(64,62,270,0.3);
        path.splineTo(34,14,1);
        path.splineToConstantAngle(-14,13,140,1);
        path.chill(-14,13,140,2);

        path.splineToConstantAngle(-11,54,250,0.6);

        path.splineToConstantAngle(-14,13,140,1);
        path.chill(-14,13,140,2);

        path.splineToConstantAngle(12,54,250,0.6);

        path.splineToConstantAngle(-14,13,140,1);
        path.chill(-14,13,140,2);

        path.splineToConstantAngle(49,54,250,0.6);

        path.splineTo(49,12,1);




        path.splineEnd(49,10,150);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true,true,true);
        defineRoute();
        waitForStart();
        robot.SWEEP.computeSplines();
        robot.odometry.overridePosition(64,16,90);
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