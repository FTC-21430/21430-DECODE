package org.firstinspires.ftc.teamcode.Opmodes.Testing.SWEEP;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Opmodes.BaseAuto;
import org.firstinspires.ftc.teamcode.Resources.SWEEP.PathPlanning;

@Autonomous
public class SWEEPGateCollectTest extends BaseAuto {


    /// Route definition methods:
    /// all units are in inches, degrees, and seconds.
    /// path.splineStart(x,y,angle) the start of the path, call once
    /// path.splineTo(x,y,speedRatio) go through this point and the angle will be the direction the robot is travelling
    /// path.splineTo(x,y,angle,speedRatio) go through a specified point while keeping the angle of the robot at a constant.
    /// path.chill(x,y,angle,duration) Wait at a specified position with a given time in seconds
    private void defineRoute(){
        PathPlanning path = robot.SWEEP.pathPlanner; // have a shorthand variable name to make typing this easier.
        path.splineStart(-64,39,180); // the start of the path
        path.splineToConstantAngle(-35,34,145,1);
        path.chill(-35,34,145,2);
        path.splineTo(-5,26,1);
        path.splineToConstantAngle(11,26,265,1);
        path.splineToConstantAngle(11,52,300,1);

        for (int i = 0; i< 30; i++){
            path.splineToConstantAngle(-35,34,145,1);
            path.chill(-35,34,145,2);

            path.splineTo(10,36,1);
            path.splineToConstantAngle(12,59,305,1);
            path.chill(12,59,305,3);
            path.splineTo(12,30,1);

        }

        // Put all next steps here
        path.splineEnd(0,0,0);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true,true,true);
        defineRoute();
        waitForStart();
        robot.SWEEP.computeSplines();
        robot.odometry.overridePosition(-64,39,180);
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